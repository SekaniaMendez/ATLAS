import Foundation
import Network

// Rendering is outside this network test. The production connection, address
// parser, ATLS decoder, and geometry model below are compiled unchanged.
final class PointCloudModel {
    func update(data: Data, pointCount: Int, sequence: UInt32,
                timestampNanoseconds: UInt64, isWorldFrame: Bool, hasRgb: Bool) {}
}

private enum Failure: Error { case failed(String) }

private func require(_ value: @autoclosure () -> Bool, _ message: String) throws {
    if !value() { throw Failure.failed(message) }
}

private func waitUntil(_ message: String, timeout: TimeInterval = 6,
                       _ predicate: () -> Bool) throws {
    let deadline = ProcessInfo.processInfo.systemUptime + timeout
    while !predicate() {
        guard ProcessInfo.processInfo.systemUptime < deadline else { throw Failure.failed(message) }
        RunLoop.main.run(until: Date().addingTimeInterval(0.01))
    }
}

private func observeFor(_ duration: TimeInterval, _ predicate: () -> Bool, _ message: String) throws {
    let deadline = ProcessInfo.processInfo.systemUptime + duration
    while ProcessInfo.processInfo.systemUptime < deadline {
        try require(predicate(), message)
        RunLoop.main.run(until: Date().addingTimeInterval(0.01))
    }
    try require(predicate(), message)
}

private func writeInteger<T: FixedWidthInteger>(_ data: inout Data, at offset: Int, _ value: T) {
    for index in 0..<MemoryLayout<T>.size {
        data[offset + index] = UInt8(truncatingIfNeeded: value >> (index * 8))
    }
}

// Independently assembled little-endian ATLS bytes, not the client's decoder.
// Each TCP session gets a unique acceptedScans count and message.
private func telemetry(_ session: Int) -> Data {
    let message = Data("loopback session \(session)".utf8)
    var frame = Data(count: 32 + 24)
    writeInteger(&frame, at: 0, UInt32(0x534C_5441))
    writeInteger(&frame, at: 4, UInt16(1))
    writeInteger(&frame, at: 6, UInt16(2))
    writeInteger(&frame, at: 8, UInt32(24 + message.count))
    writeInteger(&frame, at: 12, UInt32(session))
    writeInteger(&frame, at: 16, UInt64(session * 1_000_000))
    writeInteger(&frame, at: 24, UInt32(1))
    writeInteger(&frame, at: 28, UInt32(1))
    writeInteger(&frame, at: 32, UInt16(1))
    frame[34] = 2 // Tracking.
    writeInteger(&frame, at: 36, UInt64(session))
    writeInteger(&frame, at: 44, UInt64(0))
    writeInteger(&frame, at: 52, UInt32(message.count))
    frame.append(message)
    return frame
}

private final class LoopbackServer {
    private let queue = DispatchQueue(label: "atlas.tests.loopback")
    private let listener: NWListener
    private var connections: [Int: NWConnection] = [:]
    private var accepted = 0
    private var closed = Set<Int>()
    private var readyPort: UInt16?
    private var lastError: String?

    init() throws {
        let parameters = NWParameters.tcp
        parameters.requiredLocalEndpoint = .hostPort(host: "127.0.0.1", port: .any)
        listener = try NWListener(using: parameters)
        listener.stateUpdateHandler = { [weak self] state in
            guard let self else { return }
            switch state {
            case .ready: self.readyPort = self.listener.port?.rawValue
            case let .failed(error): self.lastError = "Listener failed: \(error)"
            default: break
            }
        }
        listener.newConnectionHandler = { [weak self] connection in self?.accept(connection) }
        listener.start(queue: queue)
    }

    var port: UInt16? { queue.sync { readyPort } }
    var count: Int { queue.sync { accepted } }
    var error: String? { queue.sync { lastError } }
    func didClose(_ session: Int) -> Bool { queue.sync { closed.contains(session) } }

    private func accept(_ connection: NWConnection) {
        accepted += 1
        let session = accepted
        connections[session] = connection
        connection.stateUpdateHandler = { [weak self, weak connection] state in
            guard let self, let connection else { return }
            if case .ready = state {
                // Deliberately split a header across two sends, then leave an
                // incomplete next message in session 1 before its remote close.
                let frame = telemetry(session)
                connection.send(content: Data(frame.prefix(11)), completion: .contentProcessed { error in
                    if let error { self.lastError = "Header send failed: \(error)"; return }
                    var remainder = Data(frame.dropFirst(11))
                    if session == 1 { remainder.append(frame.prefix(9)) }
                    connection.send(content: remainder, completion: .contentProcessed { error in
                        if let error { self.lastError = "Telemetry send failed: \(error)" }
                    })
                })
                self.receive(on: connection, session: session)
            }
        }
        connection.start(queue: queue)
    }

    private func receive(on connection: NWConnection, session: Int) {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 1024) { [weak self, weak connection] _, _, done, error in
            guard let self, let connection else { return }
            if done || error != nil { self.closed.insert(session) }
            else { self.receive(on: connection, session: session) }
        }
    }

    func close(_ session: Int) {
        queue.async { [weak self] in
            guard let connection = self?.connections[session] else { return }
            connection.send(content: nil, contentContext: .finalMessage, isComplete: true,
                            completion: .contentProcessed { _ in connection.cancel() })
        }
    }

    func stop() {
        queue.sync {
            listener.cancel()
            connections.values.forEach { $0.cancel() }
            connections.removeAll()
        }
    }
}

@main
struct AtlasConnectionIntegrationTests {
    static func main() throws {
        let suiteName = "atlas.tests.connection.\(UUID().uuidString)"
        guard let preferences = UserDefaults(suiteName: suiteName) else {
            throw Failure.failed("Could not create isolated preferences")
        }
        defer { preferences.removePersistentDomain(forName: suiteName) }
        let server = try LoopbackServer()
        defer { server.stop() }
        try waitUntil("Loopback listener did not become ready (error: \(server.error ?? "none"))") {
            server.port != nil || server.error != nil
        }
        try require(server.error == nil, server.error ?? "Unknown listener error")
        guard let port = server.port else { throw Failure.failed("No loopback port") }
        let address = "127.0.0.1:\(port)"
        let client = AtlasConnection(pointCloud: PointCloudModel(), geometry: LiveGeometryModel(),
                                     preferences: preferences)
        defer { client.disconnect() }
        client.connect(host: address)
        try waitUntil("First ATLS telemetry did not arrive: \(client.statusText)") {
            client.isConnected && client.slamStatus?.acceptedScans == 1
        }
        try require(client.connectedHost == "127.0.0.1", "Resolved host was not published")
        try require(client.slamStatus?.state == .tracking, "Tracking telemetry was not decoded")
        try require(preferences.string(forKey: AtlasServerAddress.preferencesKey) == address,
                    "Working endpoint was not remembered in isolated preferences")
        print("PASS real TCP stream publishes independently encoded ATLS telemetry")

        server.close(1)
        try waitUntil("Client did not reconnect after remote EOF: \(client.statusText)") {
            server.count == 2 && client.isConnected && client.slamStatus?.acceptedScans == 2
        }
        try require(client.slamStatus?.message == "loopback session 2", "Stale stream data survived reconnect")
        try require(server.error == nil, server.error ?? "Unknown server error")
        print("PASS remote EOF reconnects and clears partial bytes from the previous session")

        client.disconnect()
        try waitUntil("Deliberate disconnect did not close TCP / clear public state") {
            server.didClose(2) && !client.isConnected && client.statusText == "Desconectado"
                && client.connectedHost == nil && client.slamStatus == nil
        }
        client.startBrowsingIfNeeded()
        client.startBrowsingIfNeeded()
        try observeFor(2.5, { server.count == 2 && !client.isConnected && client.statusText == "Desconectado" },
                       "View appearance restarted a deliberately disconnected connection")
        print("PASS explicit disconnect stays disconnected when the map view appears again")

        client.connect(host: address)
        try waitUntil("Explicit Connect did not resume after deliberate Disconnect") {
            server.count == 3 && client.isConnected && client.slamStatus?.acceptedScans == 3
        }
        server.close(3)
        try waitUntil("Remote close did not schedule a retry") {
            !client.isConnected && client.statusText.contains("Reintento")
        }
        client.disconnect()
        try waitUntil("Disconnect did not cancel retry intent") { client.statusText == "Desconectado" }
        client.startBrowsingIfNeeded()
        try observeFor(2.5, { server.count == 3 && !client.isConnected && client.statusText == "Desconectado" },
                       "A pending reconnect fired after explicit Disconnect")
        print("PASS explicit Connect resumes and Disconnect cancels an already scheduled retry")
        try require(server.error == nil, server.error ?? "Unknown server error")
        print("AtlasConnectionIntegrationTests: 4 real-network regressions passed")
    }
}
