import Foundation
import Network

final class AtlasConnection: ObservableObject {
    @Published private(set) var statusText = "Modo demo"
    @Published private(set) var isConnected = false
    @Published private(set) var activeWarning: AtlasSensorWarning?
    @Published private(set) var slamStatus: AtlasSlamStatus?

    private let pointCloud: PointCloudModel
    private let networkQueue = DispatchQueue(label: "atlas.viewer.network", qos: .userInitiated)
    private var browser: NWBrowser?
    private var connection: NWConnection?
    private var receiveBuffer = Data()

    init(pointCloud: PointCloudModel) {
        self.pointCloud = pointCloud
    }

    func startBrowsing() {
        disconnectConnectionOnly(clearSlamStatus: true)
        browser?.cancel()
        updateStatus("Buscando ATLAS en la red…", connected: false)

        let browser = NWBrowser(for: .bonjour(type: "_atlas._tcp", domain: nil), using: .tcp)
        self.browser = browser
        browser.stateUpdateHandler = { [weak self] state in
            if case let .failed(error) = state {
                self?.updateStatus("Error Bonjour: \(error.localizedDescription)", connected: false)
            }
        }
        browser.browseResultsChangedHandler = { [weak self] results, _ in
            guard let self, self.connection == nil, let endpoint = results.first?.endpoint else { return }
            self.connect(endpoint: endpoint, label: endpoint.debugDescription)
        }
        browser.start(queue: networkQueue)
    }

    func connect(host: String) {
        let trimmed = host.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty, let port = NWEndpoint.Port(rawValue: 47_777) else { return }
        browser?.cancel()
        browser = nil
        connect(endpoint: .hostPort(host: NWEndpoint.Host(trimmed), port: port), label: trimmed)
    }

    func disconnect() {
        browser?.cancel()
        browser = nil
        disconnectConnectionOnly(clearSlamStatus: true)
        updateStatus("Desconectado", connected: false)
    }

    private func connect(endpoint: NWEndpoint, label: String) {
        disconnectConnectionOnly(clearSlamStatus: true)
        let connection = NWConnection(to: endpoint, using: .tcp)
        self.connection = connection
        updateStatus("Conectando a \(label)…", connected: false)

        connection.stateUpdateHandler = { [weak self, weak connection] state in
            guard let self, self.connection === connection else { return }
            switch state {
            case .ready:
                self.updateStatus("Conectado a ATLAS", connected: true)
                self.receiveNext(on: connection)
            case let .waiting(error):
                self.updateStatus("Esperando red: \(error.localizedDescription)", connected: false)
            case let .failed(error):
                self.updateStatus("Conexión falló: \(error.localizedDescription)", connected: false)
                self.disconnectConnectionOnly(clearSlamStatus: false)
            case .cancelled:
                self.updateStatus("Desconectado", connected: false)
            default:
                break
            }
        }
        connection.start(queue: networkQueue)
    }

    private func receiveNext(on connection: NWConnection?) {
        guard let connection else { return }
        connection.receive(minimumIncompleteLength: 1, maximumLength: 1_048_576) { [weak self, weak connection] data, _, complete, error in
            guard let self, let connection, self.connection === connection else { return }
            if let data, !data.isEmpty {
                self.receiveBuffer.append(data)
                self.consumeMessages()
            }
            if let error {
                self.updateStatus("Recepción falló: \(error.localizedDescription)", connected: false)
                self.disconnectConnectionOnly(clearSlamStatus: false)
                return
            }
            if complete {
                self.updateStatus("La Mac cerró la conexión", connected: false)
                self.disconnectConnectionOnly(clearSlamStatus: false)
                return
            }
            self.receiveNext(on: connection)
        }
    }

    private func consumeMessages() {
        while receiveBuffer.count >= AtlasWireProtocol.headerSize {
            let headerData = receiveBuffer.prefix(AtlasWireProtocol.headerSize)
            guard let header = AtlasWireProtocol.decodeHeader(Data(headerData)),
                  header.version == AtlasWireProtocol.version
            else {
                receiveBuffer.removeAll(keepingCapacity: true)
                updateStatus("Protocolo ATLAS incompatible", connected: false)
                return
            }

            let messageSize = AtlasWireProtocol.headerSize + header.payloadSize
            guard receiveBuffer.count >= messageSize else { return }
            let payload = Data(receiveBuffer[AtlasWireProtocol.headerSize..<messageSize])
            receiveBuffer.removeSubrange(0..<messageSize)

            if header.type == .pointCloud,
               header.payloadSize == header.elementCount * AtlasWireProtocol.pointSize {
                pointCloud.update(
                    data: payload,
                    pointCount: header.elementCount,
                    sequence: header.sequence,
                    timestampNanoseconds: header.timestampNanoseconds,
                    isWorldFrame: header.flags & AtlasWireProtocol.pointCloudWorldFrame != 0
                )
            } else if header.type == .sensorWarning,
                      let warning = AtlasWireProtocol.decodeSensorWarning(payload, header: header) {
                publishWarning(warning)
            } else if let status = AtlasWireProtocol.decodeSlamStatus(payload, header: header) {
                DispatchQueue.main.async { [weak self] in
                    self?.slamStatus = status
                }
            }
        }
    }

    private func disconnectConnectionOnly(clearSlamStatus: Bool) {
        connection?.cancel()
        connection = nil
        receiveBuffer.removeAll(keepingCapacity: true)
        DispatchQueue.main.async { [weak self] in
            self?.activeWarning = nil
            if clearSlamStatus { self?.slamStatus = nil }
        }
    }

    private func publishWarning(_ warning: AtlasSensorWarning) {
        DispatchQueue.main.async { [weak self] in
            self?.activeWarning = warning
        }
        DispatchQueue.main.asyncAfter(deadline: .now() + 3) { [weak self] in
            guard self?.activeWarning?.sequence == warning.sequence else { return }
            self?.activeWarning = nil
        }
    }

    private func updateStatus(_ text: String, connected: Bool) {
        DispatchQueue.main.async { [weak self] in
            self?.statusText = text
            self?.isConnected = connected
        }
    }
}
