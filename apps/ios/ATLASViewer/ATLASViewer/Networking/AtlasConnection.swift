import Combine
import Foundation
import Network

final class AtlasConnection: ObservableObject {
    @Published private(set) var statusText = "Modo demo"
    @Published private(set) var isConnected = false
    @Published private(set) var activeWarning: AtlasSensorWarning?
    @Published private(set) var slamStatus: AtlasSlamStatus?
    @Published private(set) var sourceKind: AtlasStreamSourceKind = .unknown
    @Published private(set) var connectedHost: String?
    @Published private(set) var lastHost: String

    private let pointCloud: PointCloudModel
    private let geometry: LiveGeometryModel
    private let preferences: UserDefaults
    private let networkQueue = DispatchQueue(label: "atlas.viewer.network", qos: .userInitiated)
    private var browser: NWBrowser?
    private var connection: NWConnection?
    private var receiveBuffer = Data()
    private var cameraSendPending = false
    private var hasReceivedStreamMessage = false
    private var hasReceivedSourceHello = false
    private var warningPublicationToken: UUID? // Main queue only.
    private var geometryTrackingValid = false
    private var streamUsesWorldFrame: Bool?
    private var geometryInvalidationTimestamp: UInt64?
    // Connection intent and all Network state live exclusively on networkQueue.
    private var wantsConnection = false
    private var hasStartedAutomatically = false
    private var discoveryEnabled = false
    private var manualAddress: AtlasServerAddress?
    private var rememberedAddress: AtlasServerAddress?
    private var discoveredEndpoints: [NWEndpoint] = []
    private var rejectedBonjourUntil: [String: TimeInterval] = [:]
    private var activeBonjourKey: String?
    private var pathMonitor: NWPathMonitor?
    private var pathAvailable: Bool?
    private var backoff = AtlasReconnectBackoff()
    private var retryWork: DispatchWorkItem?
    private var connectionDeadline: DispatchWorkItem?
    private var retryGeneration: UInt64 = 0

    func sendCameraImage(_ data: Data, completion: @escaping () -> Void) {
        networkQueue.async { [weak self] in
            guard let self, !self.cameraSendPending, let connection = self.connection,
                  case .ready = connection.state else { completion(); return }
            self.cameraSendPending = true
            connection.send(content: data, completion: .contentProcessed { [weak self, weak connection] _ in
                if self?.connection === connection { self?.cameraSendPending = false }
                completion()
            })
        }
    }

    init(pointCloud: PointCloudModel, geometry: LiveGeometryModel, preferences: UserDefaults = .standard) {
        self.pointCloud = pointCloud
        self.geometry = geometry
        self.preferences = preferences
        rememberedAddress = preferences.string(forKey: AtlasServerAddress.preferencesKey).flatMap(AtlasServerAddress.parse)
        lastHost = rememberedAddress?.displayAddress ?? ""
    }

    deinit {
        retryWork?.cancel()
        connectionDeadline?.cancel()
        pathMonitor?.cancel()
        browser?.cancel()
        connection?.cancel()
    }

    func startBrowsing() {
        networkQueue.async { [weak self] in
            self?.hasStartedAutomatically = true
            self?.startBrowsingOnQueue()
        }
    }

    /// Use from view appearance. Explicit Search/Connect buttons remain able to
    /// restart, but returning from another tab does not undo user Disconnect.
    func startBrowsingIfNeeded() {
        networkQueue.async { [weak self] in
            guard let self, !self.hasStartedAutomatically else { return }
            self.hasStartedAutomatically = true
            self.startBrowsingOnQueue()
        }
    }

    private func startBrowsingOnQueue() {
        let wasRequested = wantsConnection
        wantsConnection = true
        discoveryEnabled = true
        manualAddress = nil
        startPathMonitorIfNeeded()
        startBrowserIfNeeded()
        // onAppear is repeated when moving between mapping and annotation.
        // A live connection or existing retry must survive those appearances.
        guard connection == nil, retryWork == nil else { return }
        if !wasRequested { backoff.reset() }
        updateStatus("Buscando ATLAS en la red local…", connected: false)
        scheduleAttempt(after: 1.5) // Give Bonjour a chance before the saved host.
    }

    private func parameters() -> NWParameters {
        let tcp = NWProtocolTCP.Options()
        tcp.connectionTimeout = 12
        tcp.enableKeepalive = true
        tcp.keepaliveIdle = 10
        tcp.keepaliveInterval = 5
        tcp.keepaliveCount = 3
        let parameters = NWParameters(tls: nil, tcp: tcp)
        // Personal Hotspot can be marked expensive/constrained. Also allow USB
        // and Ethernet paths: binding to Wi-Fi would exclude those connections.
        parameters.prohibitExpensivePaths = false
        parameters.prohibitConstrainedPaths = false
        parameters.includePeerToPeer = true
        return parameters
    }

    private func startBrowserIfNeeded() {
        guard wantsConnection, discoveryEnabled, browser == nil else { return }
        let browser = NWBrowser(for: .bonjour(type: "_atlas._tcp", domain: nil), using: parameters())
        self.browser = browser
        browser.stateUpdateHandler = { [weak self, weak browser] state in
            guard let self, let browser, self.browser === browser else { return }
            if case let .failed(error) = state {
                self.browser = nil
                browser.cancel()
                if self.connection == nil { self.scheduleRetry(message: self.networkMessage(error)) }
            } else if case let .waiting(error) = state, self.connection == nil, self.retryWork == nil {
                self.updateStatus(self.networkMessage(error), connected: false)
            }
        }
        browser.browseResultsChangedHandler = { [weak self, weak browser] results, _ in
            guard let self, let browser, self.browser === browser else { return }
            self.discoveredEndpoints = results.map(\.endpoint).sorted { $0.debugDescription < $1.debugDescription }
            guard self.wantsConnection, self.manualAddress == nil, self.connection == nil,
                  self.availableBonjourEndpoint() != nil else { return }
            self.cancelRetry()
            self.attemptConnection()
        }
        browser.start(queue: networkQueue)
    }

    private func startPathMonitorIfNeeded() {
        guard pathMonitor == nil else { return }
        let monitor = NWPathMonitor()
        pathMonitor = monitor
        monitor.pathUpdateHandler = { [weak self] path in
            guard let self else { return }
            let previouslyAvailable = self.pathAvailable
            self.pathAvailable = path.status == .satisfied
            guard self.wantsConnection else { return }
            if path.status != .satisfied {
                self.cancelRetry()
                self.disconnectConnectionOnly(clearSlamStatus: false)
                self.updateStatus("Sin red local. Reconectaremos cuando vuelva la conexión.", connected: false)
            } else if previouslyAvailable != true {
                self.backoff.reset()
                self.rejectedBonjourUntil.removeAll()
                self.startBrowserIfNeeded()
                if self.connection == nil {
                    self.cancelRetry()
                    self.attemptConnection()
                }
            }
        }
        monitor.start(queue: networkQueue)
    }

    func connect(host: String) {
        networkQueue.async { [weak self] in self?.connectOnQueue(host: host) }
    }

    private func connectOnQueue(host: String) {
        guard let address = AtlasServerAddress.parse(host) else {
            // Bad input must not tear down an already functioning stream.
            DispatchQueue.main.async { [weak self] in
                self?.statusText = "Dirección inválida. Usa IP o nombre de la Mac; puerto 47777 por defecto, sin rutas ni tokens."
            }
            return
        }
        wantsConnection = true
        discoveryEnabled = false
        hasStartedAutomatically = true
        manualAddress = address
        remember(address)
        backoff.reset()
        cancelRetry()
        let oldBrowser = browser
        browser = nil
        oldBrowser?.cancel()
        discoveredEndpoints.removeAll()
        startPathMonitorIfNeeded()
        disconnectConnectionOnly(clearSlamStatus: true)
        attemptConnection()
    }

    func disconnect() {
        networkQueue.async { [weak self] in self?.disconnectOnQueue() }
    }

    private func disconnectOnQueue() {
        wantsConnection = false
        hasStartedAutomatically = true
        discoveryEnabled = false
        manualAddress = nil
        cancelRetry()
        let oldBrowser = browser
        browser = nil
        oldBrowser?.cancel()
        discoveredEndpoints.removeAll()
        disconnectConnectionOnly(clearSlamStatus: true)
        updateStatus("Desconectado", connected: false)
    }

    private func availableBonjourEndpoint() -> NWEndpoint? {
        let now = ProcessInfo.processInfo.systemUptime
        return discoveredEndpoints.first { (rejectedBonjourUntil[$0.debugDescription] ?? 0) <= now }
    }

    private func attemptConnection() {
        guard wantsConnection, connection == nil else { return }
        guard pathAvailable != false else {
            updateStatus("Sin red local. Reconectaremos cuando vuelva la conexión.", connected: false)
            return
        }
        startBrowserIfNeeded()
        if let manualAddress {
            connect(endpoint: manualAddress.endpoint, label: manualAddress.displayAddress, bonjour: false)
        } else if let endpoint = availableBonjourEndpoint() {
            let label: String
            if case let .service(name, _, _, _) = endpoint { label = name }
            else { label = "Mac encontrada" }
            connect(endpoint: endpoint, label: label, bonjour: true)
        } else if let rememberedAddress {
            connect(endpoint: rememberedAddress.endpoint, label: rememberedAddress.displayAddress, bonjour: false)
        } else {
            scheduleRetry(message: "No encontramos ATLAS. Inicia el programa en la Mac o introduce su dirección.")
        }
    }

    private func connect(endpoint: NWEndpoint, label: String, bonjour: Bool) {
        guard wantsConnection else { return }
        cancelRetry()
        disconnectConnectionOnly(clearSlamStatus: true)
        let connection = NWConnection(to: endpoint, using: parameters())
        self.connection = connection
        activeBonjourKey = bonjour ? endpoint.debugDescription : nil
        updateStatus("Conectando a \(label)…", connected: false)

        connection.stateUpdateHandler = { [weak self, weak connection] state in
            guard let self, let connection, self.connection === connection else { return }
            switch state {
            case .ready:
                self.connectionDeadline?.cancel()
                self.connectionDeadline = nil
                let address = AtlasServerAddress.from(endpoint: connection.currentPath?.remoteEndpoint)
                    ?? AtlasServerAddress.from(endpoint: endpoint)
                if let address { self.remember(address) }
                self.updateStatus("Conectado a ATLAS", connected: true, host: address?.host)
                self.receiveNext(on: connection)
            case let .waiting(error):
                self.connectionFailed(message: self.networkMessage(error))
            case let .failed(error):
                self.connectionFailed(message: self.networkMessage(error))
            case .cancelled:
                self.connectionFailed(message: "La conexión se interrumpió.")
            default:
                break
            }
        }
        let deadline = DispatchWorkItem { [weak self, weak connection] in
            guard let self, let connection, self.connection === connection else { return }
            if case .ready = connection.state { return }
            self.connectionFailed(message: "La Mac no responde. Comprueba que ATLAS esté iniciado y la IP sea actual.")
        }
        connectionDeadline = deadline
        networkQueue.asyncAfter(deadline: .now() + 12, execute: deadline)
        connection.start(queue: networkQueue)
    }

    private func remember(_ address: AtlasServerAddress) {
        rememberedAddress = address
        preferences.set(address.displayAddress, forKey: AtlasServerAddress.preferencesKey)
        DispatchQueue.main.async { [weak self] in self?.lastHost = address.displayAddress }
    }

    private func cancelRetry() {
        retryGeneration &+= 1
        retryWork?.cancel()
        retryWork = nil
    }

    private func scheduleAttempt(after delay: TimeInterval) {
        guard wantsConnection, retryWork == nil else { return }
        let generation = retryGeneration
        let item = DispatchWorkItem { [weak self] in
            guard let self, self.wantsConnection, self.retryGeneration == generation else { return }
            self.retryWork = nil
            self.attemptConnection()
        }
        retryWork = item
        networkQueue.asyncAfter(deadline: .now() + delay, execute: item)
    }

    private func scheduleRetry(message: String) {
        guard wantsConnection, connection == nil, retryWork == nil else { return }
        guard pathAvailable != false else {
            updateStatus("Sin red local. Reconectaremos cuando vuelva la conexión.", connected: false)
            return
        }
        let delay = backoff.nextDelay()
        updateStatus("\(message) Reintento en \(Int(delay)) s.", connected: false)
        scheduleAttempt(after: delay)
    }

    private func connectionFailed(message: String) {
        if let key = activeBonjourKey {
            rejectedBonjourUntil[key] = ProcessInfo.processInfo.systemUptime + 30
        }
        disconnectConnectionOnly(clearSlamStatus: false)
        if wantsConnection { scheduleRetry(message: message) }
    }

    private func networkMessage(_ error: NWError) -> String {
        switch error {
        case .dns(-65570), .posix(.EACCES), .posix(.EPERM):
            return "Permite Red local para ATLASViewer en Ajustes del dispositivo."
        case .posix(.ECONNREFUSED):
            return "La Mac está accesible, pero ATLAS no escucha en ese puerto. Inicia startAtlas.sh."
        case .posix(.ENETDOWN), .posix(.ENETUNREACH), .posix(.EHOSTUNREACH):
            return "No hay ruta a la Mac. Comprueba la red compartida y su dirección actual."
        default:
            return "Conexión local interrumpida: \(error.localizedDescription)."
        }
    }

    private func receiveNext(on connection: NWConnection?) {
        guard let connection else { return }
        connection.receive(minimumIncompleteLength: 1, maximumLength: 1_048_576) { [weak self, weak connection] data, _, complete, error in
            guard let self, let connection, self.connection === connection else { return }
            if let data, !data.isEmpty {
                self.receiveBuffer.append(data)
                self.consumeMessages()
            }
            guard self.connection === connection else { return }
            if let error {
                self.connectionFailed(message: self.networkMessage(error))
                return
            }
            if complete {
                self.connectionFailed(message: "La Mac cerró la conexión.")
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
                // Retrying incompatible bytes would be an endless reconnect loop.
                disconnectOnQueue()
                updateStatus("Protocolo ATLAS incompatible", connected: false)
                return
            }

            let messageSize = AtlasWireProtocol.headerSize + header.payloadSize
            guard receiveBuffer.count >= messageSize else { return }
            let payload = Data(receiveBuffer[AtlasWireProtocol.headerSize..<messageSize])
            receiveBuffer.removeSubrange(0..<messageSize)
            if !hasReceivedStreamMessage {
                hasReceivedStreamMessage = true
                backoff.reset()
            }

            if header.type == .hello {
                // Missing, malformed or repeated declarations fail closed. A
                // reconnection must establish provenance again from scratch.
                let source: AtlasStreamSourceKind = hasReceivedSourceHello ? .unknown
                    : AtlasWireProtocol.decodeSourceHello(payload, header: header) ?? .unknown
                hasReceivedSourceHello = true
                DispatchQueue.main.async { [weak self] in self?.sourceKind = source }
            } else if header.type == .pointCloud,
               header.payloadSize == header.elementCount * AtlasWireProtocol.pointSize {
                let worldFrame = header.flags & AtlasWireProtocol.pointCloudWorldFrame != 0
                streamUsesWorldFrame = worldFrame
                if !worldFrame {
                    geometry.clear(statusText: "Geometría · Requiere un mapa SLAM")
                } else if header.elementCount == 0 {
                    invalidateGeometry(at: header.timestampNanoseconds)
                    geometry.clear(statusText: "Geometría · Mapa vacío")
                }
                pointCloud.update(
                    data: payload,
                    pointCount: header.elementCount,
                    sequence: header.sequence,
                    timestampNanoseconds: header.timestampNanoseconds,
                    isWorldFrame: worldFrame,
                    hasRgb: header.flags & AtlasWireProtocol.pointCloudRgb != 0
                )
            } else if header.type == .sensorWarning,
                      let warning = AtlasWireProtocol.decodeSensorWarning(payload, header: header) {
                publishWarning(warning)
            } else if let status = AtlasWireProtocol.decodeSlamStatus(payload, header: header) {
                geometryTrackingValid = status.state == .tracking && !status.fatalError
                if !geometryTrackingValid {
                    invalidateGeometry(at: status.timestampNanoseconds)
                    geometry.clear(statusText: status.state == .lost || status.fatalError
                                   ? "Geometría · Seguimiento detenido"
                                   : "Geometría · Esperando seguimiento")
                }
                DispatchQueue.main.async { [weak self] in
                    self?.slamStatus = status
                }
            } else if header.type == .sensorStatus && header.flags == AtlasWireProtocol.sensorStatusGeometry {
                guard let snapshot = AtlasWireProtocol.decodeGeometry(payload, header: header) else {
                    geometry.clear(statusText: "Geometría · Datos inválidos")
                    continue
                }
                if geometryTrackingValid && streamUsesWorldFrame != false {
                    // A completed old job must not restore surfaces after a newer
                    // empty map or a loss of tracking has already invalidated them.
                    if snapshot.surfaces.isEmpty || geometryInvalidationTimestamp.map({
                        snapshot.timestampNanoseconds > $0
                    }) ?? true {
                        geometry.update(snapshot)
                    }
                }
            }
        }
    }

    private func disconnectConnectionOnly(clearSlamStatus: Bool) {
        connectionDeadline?.cancel()
        connectionDeadline = nil
        let oldConnection = connection
        connection = nil
        oldConnection?.cancel()
        activeBonjourKey = nil
        cameraSendPending = false
        hasReceivedStreamMessage = false
        hasReceivedSourceHello = false
        geometryTrackingValid = false
        streamUsesWorldFrame = nil
        geometryInvalidationTimestamp = nil
        geometry.clear(statusText: "Geometría · Sin conexión", resetOrdering: true)
        receiveBuffer.removeAll(keepingCapacity: true)
        DispatchQueue.main.async { [weak self] in
            self?.activeWarning = nil
            self?.warningPublicationToken = nil
            self?.sourceKind = .unknown
            if clearSlamStatus { self?.slamStatus = nil }
        }
    }

    private func invalidateGeometry(at timestamp: UInt64) {
        geometryInvalidationTimestamp = max(geometryInvalidationTimestamp ?? 0, timestamp)
    }

    private func publishWarning(_ warning: AtlasSensorWarning) {
        let publicationToken = UUID()
        DispatchQueue.main.async { [weak self] in
            self?.warningPublicationToken = publicationToken
            self?.activeWarning = warning
        }
        DispatchQueue.main.asyncAfter(deadline: .now() + 3) { [weak self] in
            guard self?.warningPublicationToken == publicationToken else { return }
            self?.activeWarning = nil
            self?.warningPublicationToken = nil
        }
    }

    private func updateStatus(_ text: String, connected: Bool, host: String? = nil) {
        DispatchQueue.main.async { [weak self] in
            self?.statusText = text
            self?.isConnected = connected
            self?.connectedHost = connected ? host : nil
        }
    }
}
