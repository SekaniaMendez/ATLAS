import ARKit
import CoreVideo
import MetalKit
import simd

private struct ViewerUniforms {
    var viewProjection: simd_float4x4
    var pointSize: Float
    var worldScale: Float
    // x = RGB enabled, y = optional upper map-Z cut in metres. The greatest
    // finite Float disables the cut without changing the 80-byte Metal ABI.
    var padding = SIMD2<Float>(0, .greatestFiniteMagnitude)
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

// Explicit float4 fields keep Swift and Metal layouts identical (32-byte stride).
private struct GeometryVertex {
    var position: SIMD4<Float>
    var color: SIMD4<Float>
}

final class PointCloudRenderer: NSObject, MTKViewDelegate {
    private let model: PointCloudModel
    private let geometry: LiveGeometryModel
    private let camera: CameraSessionController
    private let calibration: CalibrationStore
    private let commandQueue: MTLCommandQueue
    private let viewerPipeline: MTLRenderPipelineState
    private let surfacePipeline: MTLRenderPipelineState
    private let axisPipeline: MTLRenderPipelineState
    private let cameraBackgroundPipeline: MTLRenderPipelineState
    private let cameraPointPipeline: MTLRenderPipelineState
    private let depthState: MTLDepthStencilState
    private let backgroundDepthState: MTLDepthStencilState
    private let textureCache: CVMetalTextureCache
    private var pointBuffer: MTLBuffer?
    private var pointCount = 0
    private var loadedSequence: UInt32 = .max
    private var loadedGeometryGeneration: UInt64 = .max
    private var geometryVertexBuffer: MTLBuffer?
    private var geometryIndexBuffer: MTLBuffer?
    private var geometryIndexCount = 0
    private var axisBuffer: MTLBuffer?
    private var axisVertexCount = 0
    private var fittedGeometry = false
    private var yaw: Float = 0
    private var pitch: Float = -0.15
    private var distance: Float = 7
    private var viewTarget = SIMD3<Float>(0, 0, -1.4)
    private var isWorldFrame = false
    private var mapViewActive = false
    private var hasRgb = false
    private var showPoints = true
    private var showSurfaces = true
    private var showAxes = true
    private var showCeiling = false
    private var ceilingCutHeightM: Float?
    private let cameraLock = NSLock()

    init?(
        view: MTKView,
        model: PointCloudModel,
        geometry: LiveGeometryModel,
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
              ),
              let surfacePipeline = makePipeline(
                  vertex: "geometryVertex",
                  fragment: "surfaceFragment"
              ),
              let axisPipeline = makePipeline(
                  vertex: "geometryVertex",
                  fragment: "axisFragment"
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
        self.geometry = geometry
        self.camera = camera
        self.calibration = calibration
        self.commandQueue = commandQueue
        self.viewerPipeline = viewerPipeline
        self.surfacePipeline = surfacePipeline
        self.axisPipeline = axisPipeline
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

        if (uniforms.padding.x > 0.5) {
            color = (point.reserved >> 8) != 0
                ? float3(point.reflectivity, point.tag, point.reserved & 255) / 255.0
                : mix(float3(0.05, 0.35, 1.0), float3(1.0, 0.9, 0.2), intensity);
        }
        ViewerPointOut output;
        output.position = uniforms.viewProjection * float4(world, 1.0);
        output.pointSize = uniforms.pointSize;
        output.color = float4(color, livox.z >= uniforms.padding.y ? 0.0 : 1.0);
        return output;
    }

    fragment float4 viewerPointFragment(
        ViewerPointOut input [[stage_in]],
        float2 pointCoordinate [[point_coord]]) {
        if (input.color.a < 0.5 || distance(pointCoordinate, float2(0.5)) > 0.5) {
            discard_fragment();
        }
        return input.color;
    }

    struct GeometryVertex {
        float4 position;
        float4 color;
    };

    struct GeometryOut {
        float4 position [[position]];
        float3 worldPosition;
        float4 color;
        float mapHeight;
        float ceilingCutHeight;
    };

    vertex GeometryOut geometryVertex(
        uint vertexID [[vertex_id]],
        const device GeometryVertex* vertices [[buffer(0)]],
        constant ViewerUniforms& uniforms [[buffer(1)]]) {
        const GeometryVertex source = vertices[vertexID];
        const float3 map = source.position.xyz;
        // Identical map-to-view conversion and scale as the point shader.
        const float3 world = float3(-map.y, map.z, -map.x) * uniforms.worldScale;
        GeometryOut output;
        output.position = uniforms.viewProjection * float4(world, 1.0);
        output.worldPosition = world;
        output.color = source.color;
        output.mapHeight = map.z;
        output.ceilingCutHeight = uniforms.padding.y;
        return output;
    }

    fragment float4 surfaceFragment(GeometryOut input [[stage_in]]) {
        if (input.mapHeight >= input.ceilingCutHeight) discard_fragment();
        const float3 crossProduct = cross(dfdx(input.worldPosition), dfdy(input.worldPosition));
        const float3 normal = crossProduct / max(length(crossProduct), 0.000001);
        const float diffuse = abs(dot(normal, normalize(float3(0.4, 0.7, 0.6))));
        return float4(input.color.rgb * (0.45 + 0.55 * diffuse), 1.0);
    }

    fragment float4 axisFragment(GeometryOut input [[stage_in]]) {
        if (input.mapHeight >= input.ceilingCutHeight) discard_fragment();
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
        guard !shouldRenderCamera else { return }
        cameraLock.lock()
        yaw -= deltaX * 0.006
        pitch = min(.pi / 2, max(-1.35, pitch - deltaY * 0.006))
        cameraLock.unlock()
    }

    func zoom(scale: Float) {
        guard scale > 0, !shouldRenderCamera else { return }
        cameraLock.lock()
        distance = min(30, max(1.2, distance / scale))
        cameraLock.unlock()
    }

    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {}

    func setVisibility(points: Bool, surfaces: Bool, axes: Bool,
                       ceiling: Bool, ceilingCutHeightM: Float?) {
        cameraLock.lock()
        showPoints = points
        showSurfaces = surfaces
        showAxes = axes
        showCeiling = ceiling
        self.ceilingCutHeightM = ceilingCutHeightM.flatMap { $0.isFinite ? $0 : nil }
        cameraLock.unlock()
    }

    /// A user action, not a persistent mode: subsequent drags may orbit freely.
    func showTopView() {
        guard !shouldRenderCamera else { return }
        cameraLock.lock()
        if isWorldFrame || mapViewActive {
            yaw = 0
            pitch = .pi / 2
        }
        cameraLock.unlock()
    }

    private var shouldRenderCamera: Bool {
        cameraLock.lock()
        let worldFrame = isWorldFrame || mapViewActive
        cameraLock.unlock()
        // World-frame camera preview is opt-in and background-only: a fixed map
        // cannot use the live LiDAR-to-camera extrinsic as an AR overlay.
        return camera.shouldRenderCamera && (!worldFrame || camera.cameraPreviewRequested)
    }

    func draw(in view: MTKView) {
        geometry.expireIfStale()
        updatePointBuffer(device: view.device)
        updateGeometryBuffers(device: view.device)
        guard let drawable = view.currentDrawable,
              let pass = view.currentRenderPassDescriptor,
              let commandBuffer = commandQueue.makeCommandBuffer()
        else { return }

        var renderedCamera = false
        if shouldRenderCamera, let frame = camera.session.currentFrame {
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
        cameraLock.lock()
        let canProjectSensorPoints = !isWorldFrame && !mapViewActive
        cameraLock.unlock()
        // Showing the camera while SLAM is running is for framing/recording.
        // Never project world coordinates using the sensor extrinsic transform.
        if displayMode == .calibrationOverlay || !canProjectSensorPoints {
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

        if canProjectSensorPoints, pointCount > 0, let pointBuffer {
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
        guard let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: pass)
        else { return }

        let aspect = Float(view.drawableSize.width / max(1, view.drawableSize.height))
        cameraLock.lock()
        // A geometry message can precede the first world-frame cloud. Do not
        // mix an old sensor/demo buffer with map surfaces or a map-height cut.
        let drawPoints = showPoints && (!mapViewActive || isWorldFrame)
        let drawSurfaces = showSurfaces
        let drawAxes = showAxes
        let cutHeight = !showCeiling && (isWorldFrame || mapViewActive)
            ? ceilingCutHeightM ?? .greatestFiniteMagnitude : .greatestFiniteMagnitude
        cameraLock.unlock()
        var uniforms = ViewerUniforms(
            viewProjection: projectionMatrix(aspect: aspect) * viewMatrix(),
            pointSize: max(1.5, Float(view.contentScaleFactor) * 1.25),
            worldScale: 0.18,
            padding: SIMD2<Float>(hasRgb ? 1 : 0, cutHeight)
        )
        encoder.setDepthStencilState(depthState)
        encoder.setVertexBytes(&uniforms, length: MemoryLayout<ViewerUniforms>.stride, index: 1)
        if drawSurfaces, geometryIndexCount > 0,
           let geometryVertexBuffer, let geometryIndexBuffer {
            encoder.setRenderPipelineState(surfacePipeline)
            encoder.setVertexBuffer(geometryVertexBuffer, offset: 0, index: 0)
            // Let measured points remain visible where they coincide with a fitted plane.
            encoder.setDepthBias(0.0001, slopeScale: 0.5, clamp: 0.001)
            encoder.drawIndexedPrimitives(type: .triangle, indexCount: geometryIndexCount,
                                          indexType: .uint32, indexBuffer: geometryIndexBuffer,
                                          indexBufferOffset: 0)
            encoder.setDepthBias(0, slopeScale: 0, clamp: 0)
        }
        if drawPoints, pointCount > 0, let pointBuffer {
            encoder.setRenderPipelineState(viewerPipeline)
            encoder.setVertexBuffer(pointBuffer, offset: 0, index: 0)
            encoder.drawPrimitives(type: .point, vertexStart: 0, vertexCount: pointCount)
        }
        if drawAxes, axisVertexCount > 0, let axisBuffer {
            encoder.setRenderPipelineState(axisPipeline)
            encoder.setVertexBuffer(axisBuffer, offset: 0, index: 0)
            encoder.drawPrimitives(type: .line, vertexStart: 0, vertexCount: axisVertexCount)
        }
        encoder.endEncoding()
    }

    private func updateGeometryBuffers(device: MTLDevice?) {
        guard let device,
              let snapshot = geometry.snapshot(after: loadedGeometryGeneration) else { return }
        loadedGeometryGeneration = snapshot.generation
        // Clearing or replacing a snapshot never leaves previous triangles behind.
        geometryVertexBuffer = nil
        geometryIndexBuffer = nil
        axisBuffer = nil
        geometryIndexCount = 0
        axisVertexCount = 0
        cameraLock.lock()
        if !snapshot.surfaces.isEmpty {
            if !mapViewActive { yaw = 0; pitch = .pi / 2 }
            mapViewActive = true
        } else if snapshot.sourceRevision == 0 && mapViewActive {
            // New connection/session: the next map starts from above. Temporary
            // empty/stale snapshots retain their source revision and view angle.
            yaw = 0
            pitch = .pi / 2
        }
        cameraLock.unlock()
        guard !snapshot.surfaces.isEmpty else {
            // Reset ordering marks a new connection/session. A stale or temporarily
            // empty surface snapshot in the same map must not reframe the camera.
            if snapshot.sourceRevision == 0 { fittedGeometry = false }
            return
        }

        var vertices = [GeometryVertex]()
        var indices = [UInt32]()
        var axes = [GeometryVertex]()
        vertices.reserveCapacity(snapshot.surfaces.reduce(0) { $0 + $1.vertices.count })
        indices.reserveCapacity(snapshot.surfaces.reduce(0) { $0 + $1.indices.count })
        axes.reserveCapacity(snapshot.surfaces.count * 6)
        var lower = SIMD3<Float>(repeating: .greatestFiniteMagnitude)
        var upper = SIMD3<Float>(repeating: -.greatestFiniteMagnitude)
        for surface in snapshot.surfaces {
            let base = UInt32(vertices.count)
            let color = Self.surfaceColors[Int(surface.id % UInt32(Self.surfaceColors.count))]
            for point in surface.vertices {
                vertices.append(GeometryVertex(position: SIMD4(point, 1), color: color))
                lower = simd_min(lower, point)
                upper = simd_max(upper, point)
            }
            indices.append(contentsOf: surface.indices.map { $0 + base })
            let length = min(0.6, max(0.25, sqrt(surface.areaSquareMeters) * 0.15))
            // A small normal offset prevents tangential axes from z-fighting the plane.
            let origin = surface.origin + surface.axes[2] * 0.008
            for axis in 0..<3 {
                let color = Self.axisColors[axis]
                axes.append(GeometryVertex(position: SIMD4(origin, 1), color: color))
                axes.append(GeometryVertex(position: SIMD4(origin + surface.axes[axis] * length, 1),
                                           color: color))
            }
        }
        geometryVertexBuffer = Self.makeBuffer(vertices, device: device)
        geometryIndexBuffer = Self.makeBuffer(indices, device: device)
        axisBuffer = Self.makeBuffer(axes, device: device)
        if geometryVertexBuffer != nil && geometryIndexBuffer != nil {
            geometryIndexCount = indices.count
        }
        if axisBuffer != nil { axisVertexCount = axes.count }
        if !fittedGeometry, !vertices.isEmpty {
            let center = (lower + upper) * 0.5
            let radius = simd_length(upper - lower) * 0.5 * 0.18
            cameraLock.lock()
            viewTarget = SIMD3(-center.y, center.z, -center.x) * 0.18
            distance = min(30, max(1.2, radius * 2.8))
            cameraLock.unlock()
            fittedGeometry = true
        }
    }

    private static func makeBuffer<T>(_ values: [T], device: MTLDevice) -> MTLBuffer? {
        guard !values.isEmpty else { return nil }
        return values.withUnsafeBytes { bytes in
            guard let base = bytes.baseAddress else { return nil }
            return device.makeBuffer(bytes: base, length: bytes.count, options: .storageModeShared)
        }
    }

    private static let surfaceColors: [SIMD4<Float>] = [
        SIMD4(0.18, 0.72, 0.86, 1), SIMD4(0.95, 0.57, 0.22, 1),
        SIMD4(0.62, 0.52, 0.88, 1), SIMD4(0.28, 0.75, 0.52, 1),
        SIMD4(0.87, 0.38, 0.49, 1), SIMD4(0.35, 0.57, 0.91, 1),
    ]
    private static let axisColors: [SIMD4<Float>] = [
        SIMD4(1, 0.18, 0.18, 1), SIMD4(0.2, 1, 0.25, 1), SIMD4(0.2, 0.5, 1, 1),
    ]

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
        cameraLock.lock()
        if snapshot.isWorldFrame && !mapViewActive {
            yaw = 0
            pitch = .pi / 2
            viewTarget = .zero
        }
        if mapViewActive && !snapshot.isWorldFrame {
            // A sensor/demo cloud is centered near its own origin. Retaining a
            // distant map's orbit target can put that cloud completely offscreen.
            viewTarget = SIMD3(0, 0, -1.4)
            distance = 7
            yaw = 0
            pitch = -0.15
            fittedGeometry = false
        }
        mapViewActive = snapshot.isWorldFrame
        isWorldFrame = snapshot.isWorldFrame
        hasRgb = snapshot.hasRgb
        cameraLock.unlock()
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
        let target = viewTarget
        cameraLock.unlock()
        let eye = target + SIMD3<Float>(
            localDistance * sin(localYaw) * cos(localPitch),
            localDistance * sin(localPitch),
            localDistance * cos(localYaw) * cos(localPitch)
        )
        // At exactly +Z in the SLAM map, forward and the usual up vector are
        // parallel. Use the continuous yaw-dependent limit of the orbit basis.
        let up = abs(cos(localPitch)) < 0.001
            ? SIMD3<Float>(-sin(localYaw), 0, -cos(localYaw)) : SIMD3<Float>(0, 1, 0)
        return lookAt(eye: eye, center: target, up: up)
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
