import Foundation

enum AtlasMessageType: UInt16 {
    case hello = 1
    case sensorStatus = 2
    case pointCloud = 10
    case slamPose = 11
    case control = 20
    case arkitPose = 30
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
