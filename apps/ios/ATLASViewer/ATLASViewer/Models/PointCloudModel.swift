import Foundation

struct PointCloudSnapshot {
    let data: Data
    let pointCount: Int
    let sequence: UInt32
    let timestampNanoseconds: UInt64
}

final class PointCloudModel: ObservableObject, @unchecked Sendable {
    @Published private(set) var pointCount = 0
    @Published private(set) var sequence: UInt32 = 0

    private let lock = NSLock()
    private var latestData = Data()
    private var latestTimestamp: UInt64 = 0
    private var latestSequence: UInt32 = 0

    func update(data: Data, pointCount: Int, sequence: UInt32, timestampNanoseconds: UInt64) {
        lock.lock()
        latestData = data
        latestTimestamp = timestampNanoseconds
        latestSequence = sequence
        lock.unlock()

        DispatchQueue.main.async { [weak self] in
            self?.pointCount = pointCount
            self?.sequence = sequence
        }
    }

    func snapshot(after previousSequence: UInt32) -> PointCloudSnapshot? {
        lock.lock()
        defer { lock.unlock() }
        guard latestSequence != previousSequence, !latestData.isEmpty else { return nil }
        return PointCloudSnapshot(
            data: latestData,
            pointCount: latestData.count / AtlasWireProtocol.pointSize,
            sequence: latestSequence,
            timestampNanoseconds: latestTimestamp
        )
    }

    func loadDemoCloud() {
        var points = [WirePoint]()
        points.reserveCapacity(12_000)
        let latitudeSteps = 80
        let longitudeSteps = 150
        for latitude in 0..<latitudeSteps {
            let theta = Float(latitude) / Float(latitudeSteps - 1) * .pi
            for longitude in 0..<longitudeSteps {
                let phi = Float(longitude) / Float(longitudeSteps) * 2 * .pi
                let radius = 2.2 + 0.12 * sin(phi * 5) * sin(theta * 3)
                let x = radius * sin(theta) * cos(phi)
                let y = radius * sin(theta) * sin(phi)
                let z = radius * cos(theta)
                let reflectivity = UInt8(clamping: Int((Float(longitude) / Float(longitudeSteps)) * 255))
                points.append(WirePoint(x: x, y: y, z: z, reflectivity: reflectivity, tag: 0, reserved: 0))
            }
        }

        let data = points.withUnsafeBytes { Data($0) }
        lock.lock()
        let nextSequence = latestSequence &+ 1
        lock.unlock()
        update(data: data, pointCount: points.count, sequence: nextSequence, timestampNanoseconds: 0)
    }
}

private struct WirePoint {
    var x: Float
    var y: Float
    var z: Float
    var reflectivity: UInt8
    var tag: UInt8
    var reserved: UInt16
}
