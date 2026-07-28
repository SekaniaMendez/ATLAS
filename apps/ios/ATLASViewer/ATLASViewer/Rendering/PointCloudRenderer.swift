import ARKit
import CoreVideo
import MetalKit
import simd

private struct ViewerUniforms {
    var viewProjection: simd_float4x4
    var pointSize: Float
    var worldScale: Float
    var padding = SIMD2<Float>(repeating: 0)
}

private struct CameraPointUniforms {
    var projection: simd_float4x4
    var lidarToCamera: simd_float4x4
    var viewToImage: simd_float3x3
    var pointSize: Float
    var displayMode: UInt32
    var padding0: Float = 0
    var padding1: Float = 0
}

private struct CameraVertex {
    var position: SIMD2<Float>
    var textureCoordinate: SIMD2<Float>
}

final class PointCloudRenderer: NSObject, MTKViewDelegate {
    private let model: PointCloudModel
    private let camera: CameraSessionController
    private let calibration: CalibrationStore
    private let commandQueue: MTLCommandQueue
    private let viewerPipeline: MTLRenderPipelineState
    private let cameraBackgroundPipeline: MTLRenderPipelineState
    private let cameraPointPipeline: MTLRenderPipelineState
    private let depthState: MTLDepthStencilState
    private let backgroundDepthState: MTLDepthStencilState
    private let textureCache: CVMetalTextureCache
    private var pointBuffer: MTLBuffer?
    private var pointCount = 0
    private var loadedSequence: UInt32 = .max
    private var yaw: Float = 0
    private var pitch: Float = -0.15
    private var distance: Float = 7
    private let cameraLock = NSLock()

    init?(
        view: MTKView,
        model: PointCloudModel,
        camera: CameraSessionController,
        calibration: CalibrationStore
    ) {
        guard let device = view.device,
              let commandQueue = device.makeCommandQueue(),
              let library = try? device.makeLibrary(source: Self.shaderSource, options: nil)
        else { return nil }

        func makePipeline(vertex: String, fragment: String) -> MTLRenderPipelineState? {
            guard let vertexFunction = library.makeFunction(name: vertex),
                  let fragmentFunction = library.makeFunction(name: fragment)
            else { return nil }
            let descriptor = MTLRenderPipelineDescriptor()
            descriptor.vertexFunction = vertexFunction
            descriptor.fragmentFunction = fragmentFunction
            descriptor.colorAttachments[0].pixelFormat = view.colorPixelFormat
            descriptor.depthAttachmentPixelFormat = view.depthStencilPixelFormat
            return try? device.makeRenderPipelineState(descriptor: descriptor)
        }

        guard let viewerPipeline = makePipeline(
                  vertex: "viewerPointVertex",
                  fragment: "viewerPointFragment"
              ),
              let cameraBackgroundPipeline = makePipeline(
                  vertex: "cameraBackgroundVertex",
                  fragment: "cameraBackgroundFragment"
              ),
              let cameraPointPipeline = makePipeline(
                  vertex: "cameraPointVertex",
                  fragment: "cameraPointFragment"
              )
        else { return nil }

        let depthDescriptor = MTLDepthStencilDescriptor()
        depthDescriptor.depthCompareFunction = .less
        depthDescriptor.isDepthWriteEnabled = true
        guard let depthState = device.makeDepthStencilState(descriptor: depthDescriptor) else {
            return nil
        }

        let backgroundDepthDescriptor = MTLDepthStencilDescriptor()
        backgroundDepthDescriptor.depthCompareFunction = .always
        backgroundDepthDescriptor.isDepthWriteEnabled = false
        guard let backgroundDepthState = device.makeDepthStencilState(
            descriptor: backgroundDepthDescriptor
        ) else { return nil }

        var cache: CVMetalTextureCache?
        guard CVMetalTextureCacheCreate(nil, nil, device, nil, &cache) == kCVReturnSuccess,
              let cache
        else { return nil }

        self.model = model
        self.camera = camera
        self.calibration = calibration
        self.commandQueue = commandQueue
        self.viewerPipeline = viewerPipeline
        self.cameraBackgroundPipeline = cameraBackgroundPipeline
        self.cameraPointPipeline = cameraPointPipeline
        self.depthState = depthState
        self.backgroundDepthState = backgroundDepthState
        textureCache = cache
        super.init()
    }

    private static let shaderSource = """
    #include <metal_stdlib>
    using namespace metal;

    struct WirePoint {
        packed_float3 position;
        uchar reflectivity;
        uchar tag;
        ushort reserved;
    };

    struct ViewerUniforms {
        float4x4 viewProjection;
        float pointSize;
        float worldScale;
        float2 padding;
    };

    struct ViewerPointOut {
        float4 position [[position]];
        float pointSize [[point_size]];
        float4 color;
    };

    vertex ViewerPointOut viewerPointVertex(
        uint vertexID [[vertex_id]],
        const device WirePoint* points [[buffer(0)]],
        constant ViewerUniforms& uniforms [[buffer(1)]]) {
        const WirePoint point = points[vertexID];
        const float3 livox = float3(point.position);
        const float3 world = float3(-livox.y, livox.z, -livox.x) * uniforms.worldScale;
        const float intensity = float(point.reflectivity) / 255.0;
        const uint adjacency = point.tag & 0x03;
        const uint particles = (point.tag >> 2) & 0x03;
        const uint other = (point.tag >> 4) & 0x03;
        const bool lowConfidence = adjacency == 2 || particles == 2 || other == 2;
        const bool mediumConfidence = adjacency == 1 || particles == 1 || other == 1;

        float3 color = mix(float3(0.05, 0.35, 1.0), float3(1.0, 0.9, 0.2), intensity);
        if (lowConfidence) {
            color = float3(1.0, 0.1, 0.12);
        } else if (mediumConfidence) {
            color = float3(1.0, 0.45, 0.05);
        }

        ViewerPointOut output;
        output.position = uniforms.viewProjection * float4(world, 1.0);
        output.pointSize = uniforms.pointSize;
        output.color = float4(color, 1.0);
        return output;
    }

    fragment float4 viewerPointFragment(
        ViewerPointOut input [[stage_in]],
        float2 pointCoordinate [[point_coord]]) {
        if (distance(pointCoordinate, float2(0.5)) > 0.5) {
            discard_fragment();
        }
        return input.color;
    }

    struct CameraVertex {
        float2 position;
        float2 textureCoordinate;
    };

    struct CameraBackgroundOut {
        float4 position [[position]];
        float2 textureCoordinate;
    };

    vertex CameraBackgroundOut cameraBackgroundVertex(
        uint vertexID [[vertex_id]],
        const device CameraVertex* vertices [[buffer(0)]]) {
        CameraBackgroundOut output;
        output.position = float4(vertices[vertexID].position, 1.0, 1.0);
        output.textureCoordinate = vertices[vertexID].textureCoordinate;
        return output;
    }

    float3 sampleCameraRGB(
        float2 uv,
        texture2d<float, access::sample> textureY,
        texture2d<float, access::sample> textureCbCr) {
        constexpr sampler cameraSampler(address::clamp_to_edge, filter::linear);
        const float y = textureY.sample(cameraSampler, uv).r;
        const float2 chroma = textureCbCr.sample(cameraSampler, uv).rg - float2(0.5);
        return saturate(float3(
            y + 1.4020 * chroma.y,
            y - 0.3441 * chroma.x - 0.7141 * chroma.y,
            y + 1.7720 * chroma.x));
    }

    fragment float4 cameraBackgroundFragment(
        CameraBackgroundOut input [[stage_in]],
        texture2d<float, access::sample> textureY [[texture(0)]],
        texture2d<float, access::sample> textureCbCr [[texture(1)]]) {
        return float4(sampleCameraRGB(input.textureCoordinate, textureY, textureCbCr), 1.0);
    }

    struct CameraPointUniforms {
        float4x4 projection;
        float4x4 lidarToCamera;
        float3x3 viewToImage;
        float pointSize;
        uint displayMode;
        float padding0;
        float padding1;
    };

    struct CameraPointOut {
        float4 position [[position]];
        float pointSize [[point_size]];
        float2 imageUV;
        float confidence;
        uint displayMode [[flat]];
    };

    vertex CameraPointOut cameraPointVertex(
        uint vertexID [[vertex_id]],
        const device WirePoint* points [[buffer(0)]],
        constant CameraPointUniforms& uniforms [[buffer(1)]]) {
        const WirePoint point = points[vertexID];
        const float4 cameraPoint = uniforms.lidarToCamera * float4(float3(point.position), 1.0);
        const float4 clip = uniforms.projection * cameraPoint;
        const float2 ndc = clip.xy / max(0.00001, clip.w);
        const float2 viewUV = float2((ndc.x + 1.0) * 0.5, (1.0 - ndc.y) * 0.5);
        const float2 imageUV = (uniforms.viewToImage * float3(viewUV, 1.0)).xy;
        const bool visible = cameraPoint.z < -0.03 && clip.w > 0.0 &&
            all(imageUV >= float2(0.0)) && all(imageUV <= float2(1.0));

        const uint adjacency = point.tag & 0x03;
        const uint particles = (point.tag >> 2) & 0x03;
        const uint other = (point.tag >> 4) & 0x03;
        const bool lowConfidence = adjacency == 2 || particles == 2 || other == 2;
        const bool mediumConfidence = adjacency == 1 || particles == 1 || other == 1;

        CameraPointOut output;
        output.position = visible ? clip : float4(2.0, 2.0, 2.0, 1.0);
        output.pointSize = visible ? uniforms.pointSize : 0.0;
        output.imageUV = imageUV;
        output.confidence = lowConfidence ? 0.0 : (mediumConfidence ? 0.5 : 1.0);
        output.displayMode = uniforms.displayMode;
        return output;
    }

    fragment float4 cameraPointFragment(
        CameraPointOut input [[stage_in]],
        float2 pointCoordinate [[point_coord]],
        texture2d<float, access::sample> textureY [[texture(0)]],
        texture2d<float, access::sample> textureCbCr [[texture(1)]]) {
        if (distance(pointCoordinate, float2(0.5)) > 0.5) {
            discard_fragment();
        }
        float3 rgb = input.displayMode == 1
            ? float3(0.0, 1.0, 0.95)
            : sampleCameraRGB(input.imageUV, textureY, textureCbCr);
        if (input.confidence < 0.25) {
            rgb = mix(rgb, float3(1.0, 0.05, 0.05), 0.75);
        } else if (input.confidence < 0.75) {
            rgb = mix(rgb, float3(1.0, 0.45, 0.05), 0.45);
        }
        return float4(rgb, 1.0);
    }
    """

    func orbit(deltaX: Float, deltaY: Float) {
        guard !camera.shouldRenderCamera else { return }
        cameraLock.lock()
        yaw -= deltaX * 0.006
        pitch = min(1.35, max(-1.35, pitch - deltaY * 0.006))
        cameraLock.unlock()
    }

    func zoom(scale: Float) {
        guard scale > 0, !camera.shouldRenderCamera else { return }
        cameraLock.lock()
        distance = min(30, max(1.2, distance / scale))
        cameraLock.unlock()
    }

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

    func draw(in view: MTKView) {
        updatePointBuffer(device: view.device)
        guard let drawable = view.currentDrawable,
              let pass = view.currentRenderPassDescriptor,
              let commandBuffer = commandQueue.makeCommandBuffer()
        else { return }

        var renderedCamera = false
        if camera.shouldRenderCamera, let frame = camera.session.currentFrame {
            renderedCamera = encodeCamera(frame: frame, in: view, pass: pass, commandBuffer: commandBuffer)
        }
        if !renderedCamera {
            encodeViewer(in: view, pass: pass, commandBuffer: commandBuffer)
        }

        commandBuffer.present(drawable)
        commandBuffer.commit()
    }

    private func encodeCamera(
        frame: ARFrame,
        in view: MTKView,
        pass: MTLRenderPassDescriptor,
        commandBuffer: MTLCommandBuffer
    ) -> Bool {
        guard let textures = makeCameraTextures(pixelBuffer: frame.capturedImage),
              let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: pass)
        else { return false }

        let orientation = view.window?.windowScene?.interfaceOrientation ?? .portrait
        let viewportSize = view.bounds.size
        let displayMode = camera.displayMode
        if displayMode == .calibrationOverlay {
            let vertices = cameraVertices(
                frame: frame,
                orientation: orientation,
                viewportSize: viewportSize
            )
            encoder.setRenderPipelineState(cameraBackgroundPipeline)
            encoder.setDepthStencilState(backgroundDepthState)
            vertices.withUnsafeBytes { bytes in
                if let baseAddress = bytes.baseAddress {
                    encoder.setVertexBytes(
                        baseAddress,
                        length: bytes.count,
                        index: 0
                    )
                }
            }
            encoder.setFragmentTexture(textures.y, index: 0)
            encoder.setFragmentTexture(textures.cbcr, index: 1)
            encoder.drawPrimitives(
                type: .triangleStrip,
                vertexStart: 0,
                vertexCount: vertices.count
            )
        }

        if pointCount > 0, let pointBuffer {
            let displayToImage = frame.displayTransform(
                for: orientation,
                viewportSize: viewportSize
            ).inverted()
            var uniforms = CameraPointUniforms(
                projection: frame.camera.projectionMatrix(
                    for: orientation,
                    viewportSize: viewportSize,
                    zNear: 0.03,
                    zFar: 100
                ),
                lidarToCamera: lidarToCameraMatrix(),
                viewToImage: affineMatrix(displayToImage),
                pointSize: max(3.5, Float(view.contentScaleFactor) * 2.2),
                displayMode: displayMode == .calibrationOverlay ? 1 : 0
            )

            encoder.setRenderPipelineState(cameraPointPipeline)
            encoder.setDepthStencilState(depthState)
            encoder.setVertexBuffer(pointBuffer, offset: 0, index: 0)
            encoder.setVertexBytes(
                &uniforms,
                length: MemoryLayout<CameraPointUniforms>.stride,
                index: 1
            )
            encoder.setFragmentTexture(textures.y, index: 0)
            encoder.setFragmentTexture(textures.cbcr, index: 1)
            encoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: pointCount)
        }
        encoder.endEncoding()

        let retainedTextures = (textures.yReference, textures.cbcrReference)
        commandBuffer.addCompletedHandler { _ in
            _ = retainedTextures
        }
        return true
    }

    private func encodeViewer(
        in view: MTKView,
        pass: MTLRenderPassDescriptor,
        commandBuffer: MTLCommandBuffer
    ) {
        guard pointCount > 0,
              let pointBuffer,
              let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: pass)
        else { return }

        let aspect = Float(view.drawableSize.width / max(1, view.drawableSize.height))
        var uniforms = ViewerUniforms(
            viewProjection: projectionMatrix(aspect: aspect) * viewMatrix(),
            pointSize: max(1.5, Float(view.contentScaleFactor) * 1.25),
            worldScale: 0.18
        )
        encoder.setRenderPipelineState(viewerPipeline)
        encoder.setDepthStencilState(depthState)
        encoder.setVertexBuffer(pointBuffer, offset: 0, index: 0)
        encoder.setVertexBytes(&uniforms, length: MemoryLayout<ViewerUniforms>.stride, index: 1)
        encoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: pointCount)
        encoder.endEncoding()
    }

    private func makeCameraTextures(pixelBuffer: CVPixelBuffer) -> (
        y: MTLTexture,
        cbcr: MTLTexture,
        yReference: CVMetalTexture,
        cbcrReference: CVMetalTexture
    )? {
        guard CVPixelBufferGetPlaneCount(pixelBuffer) >= 2 else { return nil }
        var yReference: CVMetalTexture?
        var cbcrReference: CVMetalTexture?
        let yStatus = CVMetalTextureCacheCreateTextureFromImage(
            nil,
            textureCache,
            pixelBuffer,
            nil,
            .r8Unorm,
            CVPixelBufferGetWidthOfPlane(pixelBuffer, 0),
            CVPixelBufferGetHeightOfPlane(pixelBuffer, 0),
            0,
            &yReference
        )
        let cbcrStatus = CVMetalTextureCacheCreateTextureFromImage(
            nil,
            textureCache,
            pixelBuffer,
            nil,
            .rg8Unorm,
            CVPixelBufferGetWidthOfPlane(pixelBuffer, 1),
            CVPixelBufferGetHeightOfPlane(pixelBuffer, 1),
            1,
            &cbcrReference
        )
        guard yStatus == kCVReturnSuccess,
              cbcrStatus == kCVReturnSuccess,
              let yReference,
              let cbcrReference,
              let y = CVMetalTextureGetTexture(yReference),
              let cbcr = CVMetalTextureGetTexture(cbcrReference)
        else {
            CVMetalTextureCacheFlush(textureCache, 0)
            return nil
        }
        return (y, cbcr, yReference, cbcrReference)
    }

    private func cameraVertices(
        frame: ARFrame,
        orientation: UIInterfaceOrientation,
        viewportSize: CGSize
    ) -> [CameraVertex] {
        let transform = frame.displayTransform(
            for: orientation,
            viewportSize: viewportSize
        ).inverted()
        let corners: [(SIMD2<Float>, CGPoint)] = [
            (SIMD2(-1, -1), CGPoint(x: 0, y: 1)),
            (SIMD2(1, -1), CGPoint(x: 1, y: 1)),
            (SIMD2(-1, 1), CGPoint(x: 0, y: 0)),
            (SIMD2(1, 1), CGPoint(x: 1, y: 0)),
        ]
        return corners.map { position, viewPoint in
            let imagePoint = viewPoint.applying(transform)
            return CameraVertex(
                position: position,
                textureCoordinate: SIMD2(Float(imagePoint.x), Float(imagePoint.y))
            )
        }
    }

    private func affineMatrix(_ transform: CGAffineTransform) -> simd_float3x3 {
        simd_float3x3(columns: (
            SIMD3(Float(transform.a), Float(transform.b), 0),
            SIMD3(Float(transform.c), Float(transform.d), 0),
            SIMD3(Float(transform.tx), Float(transform.ty), 1)
        ))
    }

    private func lidarToCameraMatrix() -> simd_float4x4 {
        let transform = calibration.selectedProfile.lidarToCamera
        let r = transform.rotationMatrixRowMajor
        let t = transform.translationM
        return simd_float4x4(columns: (
            SIMD4(r[0], r[3], r[6], 0),
            SIMD4(r[1], r[4], r[7], 0),
            SIMD4(r[2], r[5], r[8], 0),
            SIMD4(t[0], t[1], t[2], 1)
        ))
    }

    private func updatePointBuffer(device: MTLDevice?) {
        guard let device, let snapshot = model.snapshot(after: loadedSequence) else { return }
        loadedSequence = snapshot.sequence
        pointCount = snapshot.pointCount
        pointBuffer = snapshot.data.withUnsafeBytes { bytes in
            guard let baseAddress = bytes.baseAddress else { return nil }
            return device.makeBuffer(bytes: baseAddress, length: bytes.count, options: .storageModeShared)
        }
    }

    private func viewMatrix() -> simd_float4x4 {
        cameraLock.lock()
        let localYaw = yaw
        let localPitch = pitch
        let localDistance = distance
        cameraLock.unlock()

        let target = SIMD3<Float>(0, 0, -1.4)
        let eye = target + SIMD3<Float>(
            localDistance * sin(localYaw) * cos(localPitch),
            localDistance * sin(localPitch),
            localDistance * cos(localYaw) * cos(localPitch)
        )
        return lookAt(eye: eye, center: target, up: SIMD3<Float>(0, 1, 0))
    }

    private func projectionMatrix(aspect: Float) -> simd_float4x4 {
        let fieldOfView: Float = 60 * .pi / 180
        let y = 1 / tan(fieldOfView * 0.5)
        let x = y / max(0.1, aspect)
        let near: Float = 0.05
        let far: Float = 200
        let z = far / (near - far)
        return simd_float4x4(columns: (
            SIMD4<Float>(x, 0, 0, 0),
            SIMD4<Float>(0, y, 0, 0),
            SIMD4<Float>(0, 0, z, -1),
            SIMD4<Float>(0, 0, z * near, 0)
        ))
    }

    private func lookAt(
        eye: SIMD3<Float>,
        center: SIMD3<Float>,
        up: SIMD3<Float>
    ) -> simd_float4x4 {
        let forward = simd_normalize(center - eye)
        let right = simd_normalize(simd_cross(forward, up))
        let correctedUp = simd_cross(right, forward)
        return simd_float4x4(columns: (
            SIMD4<Float>(right.x, correctedUp.x, -forward.x, 0),
            SIMD4<Float>(right.y, correctedUp.y, -forward.y, 0),
            SIMD4<Float>(right.z, correctedUp.z, -forward.z, 0),
            SIMD4<Float>(
                -simd_dot(right, eye),
                -simd_dot(correctedUp, eye),
                simd_dot(forward, eye),
                1
            )
        ))
    }
}
