import Foundation
import Network

/// A server address, not an arbitrary browser URL. Never persists credentials,
/// pairing tokens, URL paths, or a hard-coded interface/IP address.
struct AtlasServerAddress: Equatable {
    static let defaultPort: UInt16 = 47_777
    static let preferencesKey = "atlas.server.lastHost.v1"

    let host: String
    let port: UInt16

    var displayAddress: String {
        let formatted = host.contains(":") ? "[\(host)]" : host
        return port == Self.defaultPort ? formatted : "\(formatted):\(port)"
    }

    var endpoint: NWEndpoint {
        .hostPort(host: NWEndpoint.Host(host), port: NWEndpoint.Port(rawValue: port)!)
    }

    static func parse(_ input: String) -> AtlasServerAddress? {
        var authority = input.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !authority.isEmpty, authority.count <= 512,
              authority.unicodeScalars.allSatisfy({ $0.isASCII && $0.value > 32 && $0.value != 127 }),
              !authority.contains(where: { "\\@?#%".contains($0) }) else { return nil }
        if let separator = authority.range(of: "://") {
            let scheme = authority[..<separator.lowerBound].lowercased()
            guard ["http", "https", "tcp", "atlas"].contains(scheme) else { return nil }
            authority = String(authority[separator.upperBound...])
        }
        if authority.hasSuffix("/") { authority.removeLast() }
        guard !authority.isEmpty, !authority.contains("/") else { return nil }

        var host: String
        var explicitPort: String?
        if authority.hasPrefix("[") {
            guard let end = authority.firstIndex(of: "]") else { return nil }
            host = String(authority[authority.index(after: authority.startIndex)..<end])
            let suffix = String(authority[authority.index(after: end)...])
            guard IPv6Address(host) != nil else { return nil }
            if !suffix.isEmpty {
                guard suffix.hasPrefix(":"), suffix.count > 1 else { return nil }
                explicitPort = String(suffix.dropFirst())
            }
        } else {
            let colons = authority.filter { $0 == ":" }.count
            if colons == 1 {
                let fields = authority.split(separator: ":", omittingEmptySubsequences: false)
                host = String(fields[0])
                explicitPort = String(fields[1])
            } else {
                host = authority
                if colons > 1 && IPv6Address(host) == nil { return nil }
            }
        }
        guard !host.isEmpty else { return nil }
        if host.allSatisfy({ $0.isNumber || $0 == "." }) {
            let octets = host.split(separator: ".", omittingEmptySubsequences: false)
            guard octets.count == 4, octets.allSatisfy({ octet in
                guard let value = UInt8(octet) else { return false }
                return String(value) == octet
            }) else { return nil }
        }
        if let v4 = IPv4Address(host) {
            host = v4.debugDescription
        } else if let v6 = IPv6Address(host) {
            host = v6.debugDescription.lowercased()
        } else {
            // Numeric dotted strings must be valid IPv4 addresses, not DNS names.
            guard !host.allSatisfy({ $0.isNumber || $0 == "." }) else { return nil }
            if host.hasSuffix(".") { host.removeLast() }
            let labels = host.split(separator: ".", omittingEmptySubsequences: false)
            guard host.count <= 253, labels.allSatisfy({ label in
                !label.isEmpty && label.count <= 63 && label.first != "-" && label.last != "-"
                    && label.allSatisfy { $0.isASCII && ($0.isLetter || $0.isNumber || $0 == "-") }
            }) else { return nil }
            host = host.lowercased()
        }
        var port = defaultPort
        if let raw = explicitPort {
            guard !raw.isEmpty, raw.allSatisfy({ $0 >= "0" && $0 <= "9" }),
                  let value = UInt16(raw), value > 0 else { return nil }
            port = value
        }
        return AtlasServerAddress(host: host, port: port)
    }

    static func from(endpoint: NWEndpoint?) -> AtlasServerAddress? {
        guard case let .hostPort(host, port) = endpoint else { return nil }
        let name: String
        switch host {
        case let .name(value, _): name = value
        case let .ipv4(value): name = value.debugDescription
        case let .ipv6(value): name = value.debugDescription
        @unknown default: return nil
        }
        let formatted = name.contains(":") ? "[\(name)]" : name
        return parse("\(formatted):\(port.rawValue)")
    }
}

/// Limits reconnect frequency without giving up when the Mac restarts.
/// A deliberate disconnect cancels the scheduled work rather than consuming it.
struct AtlasReconnectBackoff {
    private(set) var failures = 0

    mutating func nextDelay() -> TimeInterval {
        let delays: [TimeInterval] = [1, 2, 4, 8, 15, 30]
        let value = delays[min(failures, delays.count - 1)]
        failures = min(failures + 1, delays.count)
        return value
    }

    mutating func reset() { failures = 0 }
}
