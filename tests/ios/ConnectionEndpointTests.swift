import Foundation

private enum Failure: Error { case failed(String) }
private func require(_ value: @autoclosure () -> Bool, _ message: String) throws {
    if !value() { throw Failure.failed(message) }
}

@main
struct ConnectionEndpointTests {
    static func main() throws {
        for (input, host, port) in [
            (" 172.20.10.2 \n", "172.20.10.2", UInt16(47_777)),
            ("atlas-mac.local", "atlas-mac.local", UInt16(47_777)),
            ("HTTP://Atlas-Mac.Local/", "atlas-mac.local", UInt16(47_777)),
            ("tcp://192.168.100.160:47778", "192.168.100.160", UInt16(47_778)),
            ("[::1]:47777", "::1", UInt16(47_777)),
            ("::1", "::1", UInt16(47_777)),
            ("https://[2001:db8::7]:50000/", "2001:db8::7", UInt16(50_000)),
        ] {
            guard let address = AtlasServerAddress.parse(input) else { throw Failure.failed("Rejected valid \(input)") }
            try require(address.host == host && address.port == port, "Incorrect host/port for \(input)")
            try require(AtlasServerAddress.parse(address.displayAddress) == address, "Display round-trip failed")
            try require(AtlasServerAddress.from(endpoint: address.endpoint) == address, "Network endpoint round-trip failed")
        }
        for input in ["", "   ", "256.168.1.1", "192.168.1", "192.168.1.1:0", "atlas.local:65536",
                      "atlas.local:", "atlas.local:-1", "atlas.local:1.5", "atlas.local/path", "https://atlas.local/a",
                      "https://user:password@atlas.local", "http://atlas.local/#token=secret", "atlas.local?token=secret",
                      "atlas.local\\evil", "atlas local", "atlas\n.local", "file://atlas.local", "-atlas.local",
                      "atlas..local", "atlas_.local", "[not-ipv6]:47777", "[::1]evil", "[::1]:", "::invalid"] {
            try require(AtlasServerAddress.parse(input) == nil, "Accepted malformed address \(input)")
        }
        var backoff = AtlasReconnectBackoff()
        let delays = (0..<10).map { _ in backoff.nextDelay() }
        try require(delays == [1, 2, 4, 8, 15, 30, 30, 30, 30, 30], "Reconnect delays are not bounded")
        backoff.reset()
        try require(backoff.nextDelay() == 1, "Reconnect delay did not reset after recovery")
        print("ConnectionEndpointTests: normalization, malformed inputs, endpoint round trips and bounded reconnect passed")
    }
}
