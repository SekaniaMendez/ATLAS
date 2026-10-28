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

enum AtlasWireProtocol {
    static let magic: UInt32 = 0x534C_5441
    static let version: UInt16 = 1
    static let headerSize = 32
    static let pointSize = 16
    static let sensorWarningSize = 12
    static let pointCloudWorldFrame: UInt32 = 1 << 0
    static let sensorStatusSlam: UInt32 = 1
    static let slamStatusVersion: UInt16 = 1
    static let slamStatusFixedSize = 24
    static let maximumPayloadSize = 16 * 1024 * 1024

    static func decodeHeader(_ data: Data) -> AtlasMessageHeader? {
        guard data.count >= headerSize,
              readUInt32(data, at: 0) == magic,
              let type = AtlasMessageType(rawValue: readUInt16(data, at: 6))
        else { return nil }

        let payloadSize = Int(readUInt32(data, at: 8))
        let elementCount = Int(readUInt32(data, at: 24))
        guard payloadSize >= 0, payloadSize <= maximumPayloadSize else { return nil }
        return AtlasMessageHeader(
            version: readUInt16(data, at: 4),
            type: type,
            payloadSize: payloadSize,
            sequence: readUInt32(data, at: 12),
            timestampNanoseconds: readUInt64(data, at: 16),
            elementCount: elementCount,
            flags: readUInt32(data, at: 28)
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
