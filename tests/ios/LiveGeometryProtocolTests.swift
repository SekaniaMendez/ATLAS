import Foundation

private enum TestFailure: Error { case failed(String) }

private func require(_ value: @autoclosure () throws -> Bool, _ message: String) throws {
    if try !value() { throw TestFailure.failed(message) }
}

private func write16(_ data: inout Data, _ offset: Int, _ value: UInt16) {
    for index in 0..<2 { data[offset + index] = UInt8(truncatingIfNeeded: value >> (index * 8)) }
}

private func write32(_ data: inout Data, _ offset: Int, _ value: UInt32) {
    for index in 0..<4 { data[offset + index] = UInt8(truncatingIfNeeded: value >> (index * 8)) }
}

private func write64(_ data: inout Data, _ offset: Int, _ value: UInt64) {
    for index in 0..<8 { data[offset + index] = UInt8(truncatingIfNeeded: value >> (index * 8)) }
}

private func writeFloat(_ data: inout Data, _ offset: Int, _ value: Float) {
    write32(&data, offset, value.bitPattern)
}

private func fixture(surfaceCount: Int = 1, revision: UInt64 = 7, sequence: UInt32 = 21)
    -> (Data, AtlasMessageHeader) {
    var payload = Data(count: 32 + surfaceCount * (84 + 36 + 12))
    write16(&payload, 0, 1)
    write16(&payload, 2, 1)
    write32(&payload, 4, UInt32(surfaceCount))
    write32(&payload, 8, UInt32(surfaceCount * 3))
    write32(&payload, 12, UInt32(surfaceCount * 3))
    write64(&payload, 16, revision)
    write32(&payload, 24, UInt32(surfaceCount * 100))
    writeFloat(&payload, 28, 18.5)
    let vertexStart = 32 + surfaceCount * 84
    let indexStart = vertexStart + surfaceCount * 36
    for surface in 0..<surfaceCount {
        let offset = 32 + surface * 84
        write32(&payload, offset, UInt32(surface + 1))
        write32(&payload, offset + 4, 1)
        for (index, value) in [Float(4 + surface), -2, 1].enumerated() {
            writeFloat(&payload, offset + 8 + index * 4, value)
        }
        for (index, value) in [Float(1), 0, 0, 0, 1, 0, 0, 0, 1].enumerated() {
            writeFloat(&payload, offset + 20 + index * 4, value)
        }
        writeFloat(&payload, offset + 56, 0.02)
        writeFloat(&payload, offset + 60, 0.008)
        write32(&payload, offset + 64, 100)
        write32(&payload, offset + 68, UInt32(surface * 3))
        write32(&payload, offset + 72, 3)
        write32(&payload, offset + 76, UInt32(surface * 3))
        write32(&payload, offset + 80, 3)
        let x = Float(4 + surface)
        for (index, value) in [x, -2, 1, x + 0.2, -2, 1, x, -1.8, 1].enumerated() {
            writeFloat(&payload, vertexStart + surface * 36 + index * 4, value)
        }
        for index in 0..<3 { write32(&payload, indexStart + surface * 12 + index * 4, UInt32(index)) }
    }
    let header = AtlasMessageHeader(version: 1, type: .sensorStatus, payloadSize: payload.count,
                                    sequence: sequence, timestampNanoseconds: 123_456_789,
                                    elementCount: surfaceCount, flags: 2)
    return (payload, header)
}

private func decoded(surfaceCount: Int = 1, revision: UInt64 = 7, sequence: UInt32 = 21)
    throws -> LiveGeometrySnapshot {
    let (data, header) = fixture(surfaceCount: surfaceCount, revision: revision, sequence: sequence)
    guard let snapshot = AtlasWireProtocol.decodeGeometry(data, header: header) else {
        throw TestFailure.failed("Valid independently built wire fixture rejected")
    }
    return snapshot
}

private func drainMainQueue() {
    RunLoop.main.run(until: Date().addingTimeInterval(0.01))
}

private func plane(_ id: UInt32, height: Float, area: Float = 2,
                   bottom: Float? = nil) -> LiveGeometrySurface {
    let vertical = bottom != nil
    let vertices: [SIMD3<Float>] = vertical
        ? [SIMD3(0, 0, bottom!), SIMD3(2, 0, bottom!), SIMD3(2, 0, height), SIMD3(0, 0, height)]
        : [SIMD3(0, 0, height), SIMD3(2, 0, height), SIMD3(2, 2, height), SIMD3(0, 2, height)]
    return LiveGeometrySurface(id: id, kind: vertical ? .vertical : .horizontal,
        origin: SIMD3(1, 1, vertical ? (height + bottom!) / 2 : height),
        axes: vertical ? [SIMD3(1,0,0), SIMD3(0,0,1), SIMD3(0,-1,0)]
                       : [SIMD3(1,0,0), SIMD3(0,1,0), SIMD3(0,0,1)],
        areaSquareMeters: area, rmseMeters: 0.01, supportPointCount: 200,
        vertices: vertices, indices: [0,1,2,0,2,3])
}

@main
struct LiveGeometryProtocolTests {
    static func main() throws {
        var count = 0
        func test(_ name: String, _ body: () throws -> Void) throws {
            try body()
            count += 1
            print("PASS \(name)")
        }

        try test("world coordinates, frames, local indices and legacy-compatible subtype") {
            let value = try decoded(surfaceCount: 2)
            try require(value.surfaces.count == 2 && value.sourceRevision == 7, "Missing surfaces/revision")
            try require(value.inputPointCount == 200 && value.processingMilliseconds == 18.5, "Wrong diagnostics")
            try require(value.surfaces[1].vertices[0] == SIMD3<Float>(5, -2, 1), "Map positions transformed twice")
            try require(value.surfaces[1].indices == [0, 1, 2], "Surface indices must remain local")
            try require(value.surfaces[0].axes[2] == SIMD3<Float>(0, 0, 1), "Axes must be columns")
            let (data, header) = fixture()
            try require(AtlasWireProtocol.decodeSlamStatus(data, header: header) == nil,
                        "Geometry must not impersonate SLAM status")
        }

        try test("empty replacement clears all surfaces") {
            let empty = try decoded(surfaceCount: 0)
            try require(empty.surfaces.isEmpty && empty.inputPointCount == 0, "Empty update is valid")
            let model = LiveGeometryModel()
            try require(model.update(try decoded()), "First snapshot rejected")
            let first = model.snapshot(after: 0)!
            try require(model.update(try decoded(surfaceCount: 0, revision: 8, sequence: 22)), "Clear rejected")
            let replacement = model.snapshot(after: first.generation)!
            try require(replacement.surfaces.isEmpty, "Deleted geometry was accumulated")
            drainMainQueue()
            try require(model.surfaceCount == 0 && model.triangleCount == 0, "Published state is stale")
        }

        try test("bounded binary lengths, counts, versions and flags") {
            let (valid, header) = fixture()
            for length in [0, 1, 31, valid.count - 1, valid.count + 1] {
                var broken = valid
                if length <= valid.count { broken = Data(valid.prefix(length)) }
                else { broken.append(0) }
                try require(AtlasWireProtocol.decodeGeometry(broken, header: header) == nil, "Length accepted: \(length)")
            }
            for (offset, value) in [(4, UInt32.max), (8, UInt32.max), (12, UInt32.max),
                                    (4, 0), (8, 0), (12, 2)] {
                var broken = valid
                write32(&broken, offset, value)
                try require(AtlasWireProtocol.decodeGeometry(broken, header: header) == nil, "Invalid count accepted")
            }
            for (offset, value) in [(0, UInt16(2)), (2, 0), (2, 5)] {
                var broken = valid
                write16(&broken, offset, value)
                try require(AtlasWireProtocol.decodeGeometry(broken, header: header) == nil, "Version/flags accepted")
            }
            var rawHeader = Data(count: 32)
            write32(&rawHeader, 0, AtlasWireProtocol.magic)
            write16(&rawHeader, 4, 1)
            write16(&rawHeader, 6, 2)
            write32(&rawHeader, 8, UInt32(valid.count))
            write32(&rawHeader, 24, 1)
            write32(&rawHeader, 28, 2)
            try require(AtlasWireProtocol.decodeHeader(rawHeader) != nil, "Valid header rejected")
            write32(&rawHeader, 8, UInt32(AtlasWireProtocol.maximumGeometryPayloadSize + 1))
            try require(AtlasWireProtocol.decodeHeader(rawHeader) == nil, "Oversized geometry buffered before rejection")
        }

        try test("invalid frames, ranges, indices and nonfinite values rejected") {
            let (valid, header) = fixture()
            let metadata = 32
            let vertexStart = 32 + 84
            let indexStart = vertexStart + 36
            for (offset, value) in [(28, Float.nan), (metadata + 8, Float.infinity),
                                    (metadata + 20, Float(2)), (metadata + 52, Float(-1)),
                                    (metadata + 56, Float(-1)), (metadata + 60, Float.nan),
                                    (vertexStart, Float.nan)] {
                var broken = valid
                writeFloat(&broken, offset, value)
                try require(AtlasWireProtocol.decodeGeometry(broken, header: header) == nil, "Invalid float/frame accepted")
            }
            for (offset, value) in [(metadata, UInt32(0)), (metadata + 4, 9), (metadata + 64, 0),
                                    (metadata + 64, UInt32.max),
                                    (metadata + 68, 1), (metadata + 72, UInt32.max),
                                    (metadata + 76, 1), (metadata + 80, 6),
                                    (indexStart, 3), (indexStart + 4, 0)] {
                var broken = valid
                write32(&broken, offset, value)
                try require(AtlasWireProtocol.decodeGeometry(broken, header: header) == nil, "Bad range/index accepted")
            }
            for thirdVertex in [[Float(4.4), -2, 1], [Float(4.2), -2, 1]] {
                var broken = valid
                for (axis, coordinate) in thirdVertex.enumerated() {
                    writeFloat(&broken, vertexStart + 24 + axis * 4, coordinate)
                }
                try require(AtlasWireProtocol.decodeGeometry(broken, header: header) == nil,
                            "Collinear or coincident Float32 vertices accepted")
            }
            var (two, twoHeader) = fixture(surfaceCount: 2)
            write32(&two, 32 + 84, 1)
            try require(AtlasWireProtocol.decodeGeometry(two, header: twoHeader) == nil, "Duplicate IDs accepted")
        }

        try test("model rejects out-of-order work and resets only for a new connection") {
            let model = LiveGeometryModel()
            try require(model.update(try decoded(revision: 9, sequence: 30)), "Initial update rejected")
            try require(!model.update(try decoded(revision: 8, sequence: 31)), "Older map revision accepted")
            try require(!model.update(try decoded(revision: 10, sequence: 29)), "Older message accepted")
            let first = model.snapshot(after: 0)!
            model.clear(statusText: "Lost tracking")
            try require(model.snapshot(after: first.generation)!.surfaces.isEmpty, "Clear must invalidate renderer")
            try require(!model.update(try decoded(revision: 9, sequence: 30)), "Old geometry resurrected")
            model.clear(resetOrdering: true)
            try require(model.update(try decoded(revision: 1, sequence: 0)), "New server session rejected")
            model.clear(resetOrdering: true)
            try require(model.update(try decoded(revision: 1, sequence: UInt32.max - 1)), "Initial wrap fixture rejected")
            try require(model.update(try decoded(revision: 2, sequence: 1)), "Legal sequence wrap rejected")
            try require(!model.update(try decoded(revision: 3, sequence: UInt32.max)), "Old wrapped sequence accepted")
        }

        try test("stalled geometry expires and queued publication cannot resurrect a clear") {
            let model = LiveGeometryModel()
            try require(model.update(try decoded()), "Update rejected")
            let first = model.snapshot(after: 0)!
            try require(!model.expireIfStale(maximumAge: 30), "Fresh geometry expired")
            try require(model.expireIfStale(maximumAge: 0), "Stalled geometry retained")
            try require(model.snapshot(after: first.generation)!.surfaces.isEmpty, "Renderer received stale surfaces")
            try require(!model.expireIfStale(maximumAge: 0), "Repeated empty clear changed generation")
            drainMainQueue()
            try require(model.surfaceCount == 0 && model.receivedAt == nil, "Queued old update resurrected geometry")
            try require(!model.update(try decoded()), "Expired revision must not be replayed")
        }

        try test("ceiling cut detects upper patches with floor and adjoining walls") {
            let room = [plane(1, height: -0.6), plane(2, height: 2.3, area: 0.6),
                        plane(3, height: 2.31, area: 0.6), plane(4, height: 2.3, bottom: -0.6)]
            try require(abs(CeilingCutaway.height(for: room)! - 2.2) < 0.001, "Upper patches were not joined")
            let partial = [plane(2, height: 2.3), plane(3, height: 2.28, bottom: -0.3)]
            try require(CeilingCutaway.height(for: partial) != nil, "A tall adjoining wall should suffice without floor")
        }

        try test("ceiling cut abstains for floor, isolated planes, tables and incomplete height") {
            try require(CeilingCutaway.height(for: []) == nil, "Empty scene acquired a roof")
            try require(CeilingCutaway.height(for: [plane(1, height: -0.6)]) == nil, "Floor was hidden")
            try require(CeilingCutaway.height(for: [plane(1, height: 2.3)]) == nil, "Isolated plane was assumed to be roof")
            try require(CeilingCutaway.height(for: [plane(1, height: 0), plane(2, height: 0.8),
                plane(3, height: 3, bottom: 0)]) == nil, "Table was mistaken for ceiling")
            try require(CeilingCutaway.height(for: [plane(1, height: 2), plane(2, height: 3, bottom: 0)]) == nil,
                        "Upper wall evidence must rule out tall furniture")
            try require(CeilingCutaway.height(for: [plane(1, height: 2.3), plane(2, height: 2.3, bottom: 1.5)]) == nil,
                        "Insufficient room height was accepted")
        }

        try test("ceiling display state is cleared on empty replacement, reconnect and expiry") {
            let model = LiveGeometryModel()
            func room(_ revision: UInt64) -> LiveGeometrySnapshot {
                LiveGeometrySnapshot(sourceRevision: revision, sequence: UInt32(revision),
                    timestampNanoseconds: 1, inputPointCount: 500, processingMilliseconds: 1,
                    truncated: false, surfaces: [plane(1, height: 2.3), plane(2, height: 2.3, bottom: 0)])
            }
            model.update(room(1)); drainMainQueue()
            try require(model.ceilingCutHeightM != nil, "Estimated ceiling not published")
            model.update(try decoded(surfaceCount: 0, revision: 2, sequence: 2)); drainMainQueue()
            try require(model.ceilingCutHeightM == nil, "Empty scene retained old cut")
            model.update(room(3)); model.clear(resetOrdering: true); drainMainQueue()
            try require(model.ceilingCutHeightM == nil, "Reconnect retained old cut")
            model.update(room(1)); drainMainQueue(); model.expireIfStale(maximumAge: 0); drainMainQueue()
            try require(model.ceilingCutHeightM == nil, "Expired geometry retained old cut")
        }

        for path in CommandLine.arguments.dropFirst() {
            try test("C++ encoded fixture interoperability") {
                let data = try Data(contentsOf: URL(fileURLWithPath: path))
                guard let header = AtlasWireProtocol.decodeHeader(Data(data.prefix(32))),
                      let snapshot = AtlasWireProtocol.decodeGeometry(Data(data.dropFirst(32)), header: header) else {
                    throw TestFailure.failed("C++ geometry packet did not decode")
                }
                print("C++ fixture: \(snapshot.surfaces.count) surfaces, \(snapshot.surfaces.reduce(0) { $0 + $1.indices.count / 3 }) triangles")
                if path.contains("live-ceiling-") {
                    guard let height = CeilingCutaway.height(for: snapshot.surfaces) else {
                        throw TestFailure.failed("Recorded indoor ceiling was not detected")
                    }
                    try require((2.1...2.3).contains(height), "Recorded ceiling cut is outside measured ceiling band")
                    print("Measured ceiling display cut: \(height) m (\(URL(fileURLWithPath: path).lastPathComponent))")
                }
            }
        }
        print("\(count) geometry protocol/model tests passed")
    }
}
