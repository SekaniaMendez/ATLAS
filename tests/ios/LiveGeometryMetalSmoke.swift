// Optional macOS GPU smoke test for the exact Metal source embedded in the iOS app.
// Run: xcrun swift -module-cache-path build/swift-geometry-module-cache \
//   tests/ios/LiveGeometryMetalSmoke.swift
import Darwin
import Foundation
import Metal
import simd

struct SmokeFailure: Error, CustomStringConvertible {
    let description: String
}

func require(_ condition: @autoclosure () -> Bool, _ message: String) throws {
    if !condition() { throw SmokeFailure(description: message) }
}

// Same explicit GPU ABI as the app; coordinates below are an independent fixture.
struct SmokeVertex { var position: SIMD4<Float>; var color: SIMD4<Float> }
struct SmokePoint {
    var x: Float; var y: Float; var z: Float
    var reflectivity: UInt8 = 0; var tag: UInt8 = 0; var reserved: UInt16 = 0x01ff
}
struct SmokeUniforms {
    var viewProjection = matrix_identity_float4x4
    var pointSize: Float = 1
    var worldScale: Float = 1
    var padding = SIMD2<Float>(0, .greatestFiniteMagnitude)
}

func runSmoke() throws {
    guard let device = MTLCreateSystemDefaultDevice(), let queue = device.makeCommandQueue() else {
        throw SmokeFailure(description: "Metal GPU unavailable; run this optional test outside the sandbox.")
    }
    let path = "apps/ios/ATLASViewer/ATLASViewer/Rendering/PointCloudRenderer.swift"
    let swiftSource = try String(contentsOfFile: path, encoding: .utf8)
    let parts = swiftSource.components(separatedBy: "private static let shaderSource = \"\"\"")
    try require(parts.count == 2, "Embedded Metal source was not found")
    let shader = parts[1].components(separatedBy: "\"\"\"")[0]
    let library = try device.makeLibrary(source: shader, options: nil)
    var pipelines = [String: MTLRenderPipelineState]()
    for (vertex, fragment) in [
        ("geometryVertex", "surfaceFragment"), ("geometryVertex", "axisFragment"),
        ("viewerPointVertex", "viewerPointFragment"),
        ("cameraPointVertex", "cameraPointFragment"),
        ("cameraBackgroundVertex", "cameraBackgroundFragment"),
    ] {
        let descriptor = MTLRenderPipelineDescriptor()
        descriptor.vertexFunction = library.makeFunction(name: vertex)
        descriptor.fragmentFunction = library.makeFunction(name: fragment)
        descriptor.colorAttachments[0].pixelFormat = .bgra8Unorm
        descriptor.depthAttachmentPixelFormat = .depth32Float
        pipelines[fragment] = try device.makeRenderPipelineState(descriptor: descriptor)
    }
    try require(MemoryLayout<SmokeVertex>.stride == 32 && MemoryLayout<SmokeUniforms>.stride == 80
                && MemoryLayout<SmokePoint>.stride == 16,
                "Swift/Metal buffer ABI changed")
    let depthDescriptor = MTLDepthStencilDescriptor()
    depthDescriptor.depthCompareFunction = .less
    depthDescriptor.isDepthWriteEnabled = true
    let depthState = device.makeDepthStencilState(descriptor: depthDescriptor)!

    func render(populated: Bool, includePoints: Bool = false,
                ceilingCut: Float = .greatestFiniteMagnitude) throws -> [UInt8] {
        let colorDescriptor = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .bgra8Unorm, width: 128, height: 128, mipmapped: false)
        colorDescriptor.usage = [.renderTarget]
        colorDescriptor.storageMode = .shared
        let color = device.makeTexture(descriptor: colorDescriptor)!
        let depthDescriptor = MTLTextureDescriptor.texture2DDescriptor(
            pixelFormat: .depth32Float, width: 128, height: 128, mipmapped: false)
        depthDescriptor.usage = [.renderTarget]
        depthDescriptor.storageMode = .private
        let depth = device.makeTexture(descriptor: depthDescriptor)!
        let pass = MTLRenderPassDescriptor()
        pass.colorAttachments[0].texture = color
        pass.colorAttachments[0].loadAction = .clear
        pass.colorAttachments[0].storeAction = .store
        pass.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 1)
        pass.depthAttachment.texture = depth
        pass.depthAttachment.loadAction = .clear
        pass.depthAttachment.storeAction = .dontCare
        pass.depthAttachment.clearDepth = 1
        let command = queue.makeCommandBuffer()!
        let encoder = command.makeRenderCommandEncoder(descriptor: pass)!
        encoder.setDepthStencilState(depthState)
        var uniforms = SmokeUniforms()
        uniforms.padding = SIMD2(includePoints ? 1 : 0, ceilingCut)
        uniforms.pointSize = includePoints ? 6 : 1
        encoder.setVertexBytes(&uniforms, length: MemoryLayout<SmokeUniforms>.stride, index: 1)
        if populated {
            // Map X=-0.5 becomes view depth 0.5; Y/Z spans a centered screen triangle.
            let red = SIMD4<Float>(0.9, 0.1, 0.08, 1)
            let vertices = [SmokeVertex(position: SIMD4(-0.5, 0.5, -0.5, 1), color: red),
                            SmokeVertex(position: SIMD4(-0.5, -0.5, -0.5, 1), color: red),
                            SmokeVertex(position: SIMD4(-0.5, 0, 0.5, 1), color: red)]
            vertices.withUnsafeBytes { bytes in
                encoder.setVertexBytes(bytes.baseAddress!, length: bytes.count, index: 0)
            }
            encoder.setRenderPipelineState(pipelines["surfaceFragment"]!)
            let indices: [UInt32] = [0, 1, 2]
            let indexBuffer = indices.withUnsafeBytes {
                device.makeBuffer(bytes: $0.baseAddress!, length: $0.count, options: .storageModeShared)!
            }
            encoder.drawIndexedPrimitives(type: .triangle, indexCount: 3, indexType: .uint32,
                                          indexBuffer: indexBuffer, indexBufferOffset: 0)
            // A nearer axis at depth 0.4 must overlay the surface, not be culled.
            let green = SIMD4<Float>(0, 1, 0, 1)
            let axes = [SmokeVertex(position: SIMD4(-0.4, 0, 0, 1), color: green),
                        SmokeVertex(position: SIMD4(-0.4, -0.6, 0, 1), color: green)]
            axes.withUnsafeBytes { bytes in
                encoder.setVertexBytes(bytes.baseAddress!, length: bytes.count, index: 0)
            }
            encoder.setRenderPipelineState(pipelines["axisFragment"]!)
            encoder.drawPrimitives(type: .line, vertexStart: 0, vertexCount: 2)
            if includePoints {
                // Exact wire layout; one blue return above the cut and another
                // below it, both nearer than the fitted surface.
                let points = [SmokePoint(x: -0.3, y: 0.5, z: 0.3),
                              SmokePoint(x: -0.3, y: 0.75, z: -0.6)]
                points.withUnsafeBytes { bytes in
                    encoder.setVertexBytes(bytes.baseAddress!, length: bytes.count, index: 0)
                }
                encoder.setRenderPipelineState(pipelines["viewerPointFragment"]!)
                encoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: points.count)
            }
        }
        encoder.endEncoding()
        command.commit()
        command.waitUntilCompleted()
        try require(command.status == .completed, "GPU command failed: \(String(describing: command.error))")
        var bytes = [UInt8](repeating: 0, count: 128 * 128 * 4)
        bytes.withUnsafeMutableBytes {
            color.getBytes($0.baseAddress!, bytesPerRow: 128 * 4,
                           from: MTLRegionMake2D(0, 0, 128, 128), mipmapLevel: 0)
        }
        return bytes
    }
    let filled = try render(populated: true)
    var redPixels = 0, greenPixels = 0
    for offset in stride(from: 0, to: filled.count, by: 4) {
        if filled[offset + 2] > 80 && filled[offset + 1] < 40 { redPixels += 1 }
        if filled[offset + 1] > 100 && filled[offset + 2] < 20 { greenPixels += 1 }
    }
    try require(redPixels > 1000, "Surface geometry or map-to-view transform failed: \(redPixels) red pixels")
    try require(greenPixels > 20, "Reference axis did not render: \(greenPixels) green pixels")
    try require(filled[0] == 0 && filled[1] == 0 && filled[2] == 0, "Geometry escaped expected screen bounds")
    let empty = try render(populated: false)
    try require(stride(from: 0, to: empty.count, by: 4).allSatisfy {
        empty[$0] == 0 && empty[$0 + 1] == 0 && empty[$0 + 2] == 0
    }, "Empty replacement frame retained geometry")
    func counts(_ bytes: [UInt8]) -> (red: Int, green: Int, blue: Int) {
        var result = (red: 0, green: 0, blue: 0)
        for offset in stride(from: 0, to: bytes.count, by: 4) {
            if bytes[offset + 2] > 80 && bytes[offset + 1] < 40 { result.red += 1 }
            if bytes[offset + 1] > 100 && bytes[offset + 2] < 20 { result.green += 1 }
            if bytes[offset] > 200 && bytes[offset + 1] < 20 && bytes[offset + 2] < 20 { result.blue += 1 }
        }
        return result
    }
    let allLayers = try render(populated: true, includePoints: true)
    let cutLayers = try render(populated: true, includePoints: true, ceilingCut: 0)
    let fullCount = counts(allLayers), cutCount = counts(cutLayers)
    try require(fullCount.blue > 30, "Measured point fixture did not render")
    try require(cutCount.red > 100 && cutCount.red < fullCount.red,
                "Upper cut must clip only the upper portion of a crossing triangle")
    try require(cutCount.green == 0 && fullCount.green > 20,
                "Reference axes at/above the cut must disappear")
    try require(cutCount.blue > 10 && cutCount.blue < fullCount.blue,
                "Upper cut must remove high measured points while retaining low ones")
    let restored = try render(populated: true, includePoints: true)
    try require(restored == allLayers, "Showing the ceiling did not restore the identical measured scene")
    let allCut = try render(populated: true, includePoints: true, ceilingCut: -1)
    try require(allCut == empty, "The same map-height cut was not applied to all three layers")
    print("Metal smoke passed: 5 pipelines, \(redPixels) surface pixels, \(greenPixels) axis pixels, empty frame cleared; reversible ceiling cut verified on points, crossing triangles and axes.")
}

do { try runSmoke() } catch { fputs("\(error)\n", stderr); exit(1) }
