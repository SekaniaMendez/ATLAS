import Foundation

enum AtlasMessageType: UInt16 {
    case hello = 1
    case sensorStatus = 2
    case sensorWarning = 3
    case pointCloud = 10
    case slamPose = 11
    case control = 20
    case arkitPose = 30
}

enum AtlasStreamSourceKind: UInt8 {
    case unknown = 0
    case liveLidar = 1
    case recorded = 2
}

enum AtlasSensorWarningCode: UInt16 {
    case abruptAcceleration = 1
    case abruptRotation = 2
    case excessiveSpeed = 3
}

enum AtlasSensorWarningSeverity: UInt8 {
    case information = 0
    case caution = 1
    case critical = 2
}

struct AtlasSensorWarning {
    let sequence: UInt32
    let sensorID: UInt32
    let timestampNanoseconds: UInt64
    let code: AtlasSensorWarningCode
    let severity: AtlasSensorWarningSeverity
    let measuredValue: Float
    let threshold: Float
}

enum AtlasSlamTrackingState: UInt8 {
    case uninitialized = 0
    case initializing = 1
    case tracking = 2
    case lost = 3

    var title: String {
        switch self {
        case .uninitialized: "SLAM · Esperando datos"
        case .initializing: "SLAM · Inicializando"
        case .tracking: "SLAM · Seguimiento activo"
        case .lost: "SLAM · Seguimiento perdido"
        }
    }
}

struct AtlasSlamStatus {
    let state: AtlasSlamTrackingState
    let fatalError: Bool
    let acceptedScans: UInt64
    let rejectedScans: UInt64
    let message: String
    let timestampNanoseconds: UInt64
}

struct AtlasMessageHeader {
    let version: UInt16
    let type: AtlasMessageType
    let payloadSize: Int
    let sequence: UInt32
    let timestampNanoseconds: UInt64
    let elementCount: Int
    let flags: UInt32
}

enum LiveGeometryKind: UInt32 {
    case horizontal = 1
    case vertical = 2
    case inclined = 3
    case freeform = 4
}

struct LiveGeometrySurface {
    let id: UInt32
    let kind: LiveGeometryKind
    let origin: SIMD3<Float>
    /// Three orthonormal, right-handed columns in the unchanged SLAM map.
    let axes: [SIMD3<Float>]
    let areaSquareMeters: Float
    let rmseMeters: Float
    let supportPointCount: Int
    /// Positions already expressed in the SLAM map, never camera coordinates.
    let vertices: [SIMD3<Float>]
    let indices: [UInt32]
}

struct LiveGeometrySnapshot {
    // Local generations also advance on clearing; server sequence may restart.
    var generation: UInt64 = 0
    let sourceRevision: UInt64
    let sequence: UInt32
    let timestampNanoseconds: UInt64
    let inputPointCount: Int
    let processingMilliseconds: Float
    let truncated: Bool
    var receivedAt: Date? = nil
    let surfaces: [LiveGeometrySurface]
}

enum AtlasWireProtocol {
    static let magic: UInt32 = 0x534C_5441
    static let version: UInt16 = 1
    static let headerSize = 32
    static let pointSize = 16
    static let sensorWarningSize = 12
    static let sourceHelloVersion: UInt16 = 1
    static let sourceHelloSize = 4
    static let pointCloudRgb: UInt32 = 1 << 1
    static let pointCloudWorldFrame: UInt32 = 1 << 0
    static let sensorStatusSlam: UInt32 = 1
    // A reserved subtype keeps old viewers compatible: they ignore this status.
    static let sensorStatusGeometry: UInt32 = 2
    static let geometryVersion: UInt16 = 1
    static let geometryFixedSize = 32
    static let geometrySurfaceSize = 84
    static let maximumGeometrySurfaces = 32
    static let maximumGeometryVertices = 30_000
    static let maximumGeometryIndices = 90_000
    static let maximumGeometryPayloadSize = 1_048_576
    static let slamStatusVersion: UInt16 = 1
    static let slamStatusFixedSize = 24
    static let maximumPayloadSize = 16 * 1024 * 1024

    static func decodeSourceHello(_ data: Data, header: AtlasMessageHeader) -> AtlasStreamSourceKind? {
        guard header.version == version, header.type == .hello,
              header.payloadSize == sourceHelloSize, data.count == sourceHelloSize,
              header.elementCount == 1, header.flags == 0,
              readUInt16(data, at: 0) == sourceHelloVersion, data[3] == 0 else { return nil }
        return AtlasStreamSourceKind(rawValue: data[2])
    }

    static func decodeHeader(_ data: Data) -> AtlasMessageHeader? {
        guard data.count >= headerSize,
              readUInt32(data, at: 0) == magic,
              let type = AtlasMessageType(rawValue: readUInt16(data, at: 6))
        else { return nil }

        let payloadSize = Int(readUInt32(data, at: 8))
        let elementCount = Int(readUInt32(data, at: 24))
        let flags = readUInt32(data, at: 28)
        guard payloadSize >= 0, payloadSize <= maximumPayloadSize else { return nil }
        if type == .sensorStatus && flags == sensorStatusGeometry {
            guard payloadSize >= geometryFixedSize, payloadSize <= maximumGeometryPayloadSize,
                  elementCount <= maximumGeometrySurfaces else { return nil }
        }
        return AtlasMessageHeader(
            version: readUInt16(data, at: 4),
            type: type,
            payloadSize: payloadSize,
            sequence: readUInt32(data, at: 12),
            timestampNanoseconds: readUInt64(data, at: 16),
            elementCount: elementCount,
            flags: flags
        )
    }

    static func decodeSensorWarning(
        _ data: Data,
        header: AtlasMessageHeader
    ) -> AtlasSensorWarning? {
        guard data.count == sensorWarningSize,
              header.payloadSize == sensorWarningSize,
              let code = AtlasSensorWarningCode(rawValue: readUInt16(data, at: 0)),
              let severity = AtlasSensorWarningSeverity(rawValue: data[2])
        else { return nil }

        return AtlasSensorWarning(
            sequence: header.sequence,
            sensorID: header.flags,
            timestampNanoseconds: header.timestampNanoseconds,
            code: code,
            severity: severity,
            measuredValue: Float(bitPattern: readUInt32(data, at: 4)),
            threshold: Float(bitPattern: readUInt32(data, at: 8))
        )
    }

    static func decodeSlamStatus(_ data: Data, header: AtlasMessageHeader) -> AtlasSlamStatus? {
        guard header.type == .sensorStatus,
              header.flags == sensorStatusSlam,
              header.elementCount == 1,
              header.payloadSize == data.count,
              data.count >= slamStatusFixedSize,
              readUInt16(data, at: 0) == slamStatusVersion,
              let state = AtlasSlamTrackingState(rawValue: data[2]),
              data[3] & ~UInt8(1) == 0,
              Int(readUInt32(data, at: 20)) == data.count - slamStatusFixedSize,
              let message = String(data: data.dropFirst(slamStatusFixedSize), encoding: .utf8)
        else { return nil }
        return AtlasSlamStatus(
            state: state,
            fatalError: data[3] & 1 != 0,
            acceptedScans: readUInt64(data, at: 4),
            rejectedScans: readUInt64(data, at: 12),
            message: message,
            timestampNanoseconds: header.timestampNanoseconds
        )
    }

    static func decodeGeometry(_ data: Data, header: AtlasMessageHeader) -> LiveGeometrySnapshot? {
        guard header.version == version, header.type == .sensorStatus,
              header.flags == sensorStatusGeometry, header.payloadSize == data.count,
              data.count >= geometryFixedSize, data.count <= maximumGeometryPayloadSize,
              readUInt16(data, at: 0) == geometryVersion
        else { return nil }
        let flags = readUInt16(data, at: 2)
        let surfaceCount = Int(readUInt32(data, at: 4))
        let vertexCount = Int(readUInt32(data, at: 8))
        let indexCount = Int(readUInt32(data, at: 12))
        let inputPointCount = Int(readUInt32(data, at: 24))
        let processingTime = readFloat(data, at: 28)
        guard flags & 1 == 1, flags & ~UInt16(3) == 0,
              surfaceCount == header.elementCount,
              surfaceCount <= maximumGeometrySurfaces,
              vertexCount <= maximumGeometryVertices, indexCount <= maximumGeometryIndices,
              indexCount % 3 == 0, processingTime.isFinite, processingTime >= 0
        else { return nil }
        // All operands are bounded before arithmetic or allocation.
        let verticesOffset = geometryFixedSize + surfaceCount * geometrySurfaceSize
        let indicesOffset = verticesOffset + vertexCount * 12
        guard data.count == indicesOffset + indexCount * 4,
              surfaceCount > 0 || (vertexCount == 0 && indexCount == 0)
        else { return nil }

        var surfaces: [LiveGeometrySurface] = []
        var identifiers = Set<UInt32>()
        var expectedVertexOffset = 0
        var expectedIndexOffset = 0
        surfaces.reserveCapacity(surfaceCount)
        for surfaceIndex in 0..<surfaceCount {
            let offset = geometryFixedSize + surfaceIndex * geometrySurfaceSize
            let identifier = readUInt32(data, at: offset)
            guard identifier > 0, identifiers.insert(identifier).inserted,
                  let kind = LiveGeometryKind(rawValue: readUInt32(data, at: offset + 4))
            else { return nil }
            let origin = readVector(data, at: offset + 8)
            let axes = (0..<3).map { readVector(data, at: offset + 20 + $0 * 12) }
            let area = readFloat(data, at: offset + 56)
            let rmse = readFloat(data, at: offset + 60)
            let supportCount = Int(readUInt32(data, at: offset + 64))
            let firstVertex = Int(readUInt32(data, at: offset + 68))
            let surfaceVertices = Int(readUInt32(data, at: offset + 72))
            let firstIndex = Int(readUInt32(data, at: offset + 76))
            let surfaceIndices = Int(readUInt32(data, at: offset + 80))
            guard finite(origin), validAxes(axes), area.isFinite, area > 0,
                  rmse.isFinite, rmse >= 0, supportCount >= 3, supportCount <= inputPointCount,
                  firstVertex == expectedVertexOffset, firstIndex == expectedIndexOffset,
                  surfaceVertices >= 3, surfaceVertices <= vertexCount - firstVertex,
                  surfaceIndices >= 3, surfaceIndices % 3 == 0,
                  surfaceIndices <= indexCount - firstIndex
            else { return nil }
            var vertices: [SIMD3<Float>] = []
            vertices.reserveCapacity(surfaceVertices)
            for index in 0..<surfaceVertices {
                let vertex = readVector(data, at: verticesOffset + (firstVertex + index) * 12)
                guard finite(vertex) else { return nil }
                vertices.append(vertex)
            }
            var indices: [UInt32] = []
            indices.reserveCapacity(surfaceIndices)
            for index in 0..<surfaceIndices {
                let value = readUInt32(data, at: indicesOffset + (firstIndex + index) * 4)
                guard value < UInt32(surfaceVertices) else { return nil }
                indices.append(value)
            }
            for index in stride(from: 0, to: surfaceIndices, by: 3) {
                guard indices[index] != indices[index + 1],
                      indices[index] != indices[index + 2],
                      indices[index + 1] != indices[index + 2] else { return nil }
                guard validTriangle(vertices[Int(indices[index])],
                                    vertices[Int(indices[index + 1])],
                                    vertices[Int(indices[index + 2])]) else { return nil }
            }
            surfaces.append(LiveGeometrySurface(
                id: identifier, kind: kind, origin: origin, axes: axes,
                areaSquareMeters: area, rmseMeters: rmse, supportPointCount: supportCount,
                vertices: vertices, indices: indices
            ))
            expectedVertexOffset += surfaceVertices
            expectedIndexOffset += surfaceIndices
        }
        guard expectedVertexOffset == vertexCount, expectedIndexOffset == indexCount else { return nil }
        return LiveGeometrySnapshot(
            sourceRevision: readUInt64(data, at: 16), sequence: header.sequence,
            timestampNanoseconds: header.timestampNanoseconds,
            inputPointCount: inputPointCount,
            processingMilliseconds: processingTime, truncated: flags & 2 != 0,
            surfaces: surfaces
        )
    }

    private static func finite(_ vector: SIMD3<Float>) -> Bool {
        vector.x.isFinite && vector.y.isFinite && vector.z.isFinite
    }

    private static func validTriangle(_ a: SIMD3<Float>, _ b: SIMD3<Float>, _ c: SIMD3<Float>) -> Bool {
        // Validate the actual Float32 positions sent to Metal. Distinct indices
        // can still become coincident or collinear after coordinate conversion.
        let ux = Double(b.x) - Double(a.x), uy = Double(b.y) - Double(a.y), uz = Double(b.z) - Double(a.z)
        let vx = Double(c.x) - Double(a.x), vy = Double(c.y) - Double(a.y), vz = Double(c.z) - Double(a.z)
        let x = uy * vz - uz * vy, y = uz * vx - ux * vz, z = ux * vy - uy * vx
        let crossSquared = x * x + y * y + z * z
        return crossSquared.isFinite && crossSquared > 1e-16
    }

    private static func validAxes(_ axes: [SIMD3<Float>]) -> Bool {
        func dot(_ a: SIMD3<Float>, _ b: SIMD3<Float>) -> Float {
            a.x * b.x + a.y * b.y + a.z * b.z
        }
        guard axes.allSatisfy(finite), axes.allSatisfy({ abs(dot($0, $0) - 1) < 0.002 }),
              abs(dot(axes[0], axes[1])) < 0.002,
              abs(dot(axes[0], axes[2])) < 0.002,
              abs(dot(axes[1], axes[2])) < 0.002 else { return false }
        let a = axes[0], b = axes[1]
        let cross = SIMD3<Float>(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x)
        return dot(cross, axes[2]) > 0.998
    }

    private static func readFloat(_ data: Data, at offset: Int) -> Float {
        Float(bitPattern: readUInt32(data, at: offset))
    }

    private static func readVector(_ data: Data, at offset: Int) -> SIMD3<Float> {
        SIMD3<Float>(readFloat(data, at: offset), readFloat(data, at: offset + 4),
                     readFloat(data, at: offset + 8))
    }

    private static func readUInt16(_ data: Data, at offset: Int) -> UInt16 {
        UInt16(data[offset]) | (UInt16(data[offset + 1]) << 8)
    }

    private static func readUInt32(_ data: Data, at offset: Int) -> UInt32 {
        UInt32(data[offset]) |
            (UInt32(data[offset + 1]) << 8) |
            (UInt32(data[offset + 2]) << 16) |
            (UInt32(data[offset + 3]) << 24)
    }

    private static func readUInt64(_ data: Data, at offset: Int) -> UInt64 {
        var value: UInt64 = 0
        for index in 0..<8 {
            value |= UInt64(data[offset + index]) << UInt64(index * 8)
        }
        return value
    }
}
