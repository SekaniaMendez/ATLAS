import Combine
import Foundation

/// A conservative display cut for an indoor, gravity-aligned (+Z) SLAM map.
/// These are geometric hints, not semantic labels or changes to the measurements.
enum CeilingCutaway {
    static func height(for surfaces: [LiveGeometrySurface]) -> Float? {
        let walls = surfaces.filter {
            $0.kind == .vertical && $0.areaSquareMeters >= 0.15 && $0.supportPointCount >= 80
        }
        let horizontal = surfaces.filter {
            $0.kind == .horizontal && $0.axes.count == 3 && abs($0.axes[2].z) >= 0.985
                && $0.rmseMeters <= 0.06 && $0.supportPointCount >= 80
                && !$0.vertices.isEmpty && $0.origin.z.isFinite
                && ($0.vertices.map(\.z).max()! - $0.vertices.map(\.z).min()!) <= 0.35
        }
        // A ceiling can be split into several occupied patches. Combine only
        // nearly coplanar patches, starting at the highest observed level.
        for candidate in horizontal.sorted(by: { $0.origin.z > $1.origin.z }) {
            let patches = horizontal.filter { abs($0.origin.z - candidate.origin.z) <= 0.20 }
            let area = patches.reduce(Float(0)) { $0 + $1.areaSquareMeters }
            guard area >= 1 else { continue }
            let height = patches.reduce(Float(0)) { $0 + $1.origin.z * $1.areaSquareMeters } / area
            let minimum = patches.flatMap(\.vertices).map(\.z).min()!
            // Do not cut at a tabletop while credible walls continue above it.
            if walls.contains(where: { ($0.vertices.map(\.z).max() ?? height) > height + 0.40 }) {
                continue
            }
            let lowerFloorArea = horizontal.filter { height - $0.origin.z >= 1.8 }
                .reduce(Float(0)) { $0 + $1.areaSquareMeters }
            let meetingWalls = walls.filter {
                guard let top = $0.vertices.map(\.z).max() else { return false }
                return abs(top - height) <= 0.35
            }
            let tallWallArea = meetingWalls.filter {
                height - ($0.vertices.map(\.z).min() ?? height) >= 1.8
            }.reduce(Float(0)) { $0 + $1.areaSquareMeters }
            // In a partial scan the floor may be absent; a tall adjoining wall
            // can supply the lower reference instead. An isolated plane cannot.
            guard tallWallArea >= 0.5 || (lowerFloorArea >= 1 && !meetingWalls.isEmpty) else {
                continue
            }
            return minimum - 0.10
        }
        return nil
    }
}

/// Complete map snapshots. A removal is an empty replacement, never an append.
final class LiveGeometryModel: ObservableObject, @unchecked Sendable {
    @Published private(set) var surfaceCount = 0
    @Published private(set) var triangleCount = 0
    @Published private(set) var processingMilliseconds: Float = 0
    @Published private(set) var truncated = false
    @Published private(set) var receivedAt: Date?
    @Published private(set) var statusText = "Geometría · Esperando superficies"
    @Published private(set) var generation: UInt64 = 0
    @Published private(set) var ceilingCutHeightM: Float?

    private let lock = NSLock()
    private var latest: LiveGeometrySnapshot?
    private var latestGeneration: UInt64 = 0
    private var lastSourceRevision: UInt64?
    private var lastSequence: UInt32?
    private var lastReceiveUptime: TimeInterval?
    private var latestStatusText = "Geometría · Esperando superficies"

    @discardableResult
    func update(_ incoming: LiveGeometrySnapshot) -> Bool {
        lock.lock()
        if let revision = lastSourceRevision, incoming.sourceRevision <= revision {
            lock.unlock()
            return false
        }
        if let sequence = lastSequence {
            let distance = incoming.sequence &- sequence
            guard distance > 0 && distance < 0x8000_0000 else {
                lock.unlock()
                return false
            }
        }
        latestGeneration &+= 1
        var snapshot = incoming
        snapshot.generation = latestGeneration
        snapshot.receivedAt = Date()
        latest = snapshot
        lastSourceRevision = incoming.sourceRevision
        lastSequence = incoming.sequence
        lastReceiveUptime = ProcessInfo.processInfo.systemUptime
        latestStatusText = incoming.surfaces.isEmpty
            ? "Geometría · Sin superficies confirmadas"
            : "Geometría · Actualización en vivo"
        lock.unlock()
        publishLatest()
        return true
    }

    func clear(statusText: String = "Geometría · Esperando superficies", resetOrdering: Bool = false) {
        lock.lock()
        if resetOrdering {
            lastSourceRevision = nil
            lastSequence = nil
        }
        if latest?.surfaces.isEmpty == true && latestStatusText == statusText && lastReceiveUptime == nil {
            lock.unlock()
            return
        }
        latestGeneration &+= 1
        latest = emptySnapshot(generation: latestGeneration)
        latestStatusText = statusText
        lastReceiveUptime = nil
        lock.unlock()
        publishLatest()
    }

    func snapshot(after previousGeneration: UInt64) -> LiveGeometrySnapshot? {
        lock.lock()
        defer { lock.unlock() }
        guard let latest, latest.generation != previousGeneration else { return nil }
        return latest
    }

    func age(at date: Date = Date()) -> TimeInterval? {
        lock.lock()
        defer { lock.unlock() }
        return latest?.receivedAt.map { max(0, date.timeIntervalSince($0)) }
    }

    /// Call from the renderer or a timer so a stalled stream cannot retain ghosts.
    @discardableResult
    func expireIfStale(maximumAge: TimeInterval = 3) -> Bool {
        guard maximumAge.isFinite, maximumAge >= 0 else { return false }
        lock.lock()
        guard let lastReceiveUptime,
              ProcessInfo.processInfo.systemUptime - lastReceiveUptime > maximumAge else {
            lock.unlock()
            return false
        }
        latestGeneration &+= 1
        latest = emptySnapshot(generation: latestGeneration)
        latestStatusText = "Geometría · Sin actualización reciente"
        self.lastReceiveUptime = nil
        lock.unlock()
        publishLatest()
        return true
    }

    private func emptySnapshot(generation: UInt64) -> LiveGeometrySnapshot {
        LiveGeometrySnapshot(generation: generation, sourceRevision: lastSourceRevision ?? 0,
                             sequence: lastSequence ?? 0, timestampNanoseconds: 0,
                             inputPointCount: 0, processingMilliseconds: 0,
                             truncated: false, surfaces: [])
    }

    private func publishLatest() {
        // Read on the main queue, rather than publishing a captured older frame
        // after a concurrent clear/reconnect. Coalescing is intentional.
        DispatchQueue.main.async { [weak self] in
            guard let self else { return }
            self.lock.lock()
            let snapshot = self.latest
            let text = self.latestStatusText
            self.lock.unlock()
            guard let snapshot else { return }
            self.surfaceCount = snapshot.surfaces.count
            self.triangleCount = snapshot.surfaces.reduce(0) { $0 + $1.indices.count / 3 }
            self.processingMilliseconds = snapshot.processingMilliseconds
            self.truncated = snapshot.truncated
            self.receivedAt = snapshot.receivedAt
            self.statusText = text
            self.ceilingCutHeightM = CeilingCutaway.height(for: snapshot.surfaces)
            self.generation = snapshot.generation
        }
    }
}
