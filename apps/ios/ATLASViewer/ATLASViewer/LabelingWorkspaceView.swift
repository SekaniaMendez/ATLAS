import Foundation
import Security
import SwiftUI
import UIKit
import WebKit

/// The Mac owns the captures and annotations. This view is a local-network client.
struct LabelingWorkspaceView: View {
    @StateObject private var workspace = LabelingWorkspaceModel()
    @State private var showingConfiguration = false

    var body: some View {
        VStack(spacing: 0) {
            HStack(spacing: 12) {
                VStack(alignment: .leading, spacing: 2) {
                    Text("ATLAS · Etiquetar").font(.headline)
                    Text(workspace.statusText)
                        .font(.caption)
                        .foregroundStyle(workspace.errorText == nil ? Color.secondary : .orange)
                        .lineLimit(2)
                }
                Spacer(minLength: 4)
                if workspace.isLoading { ProgressView().controlSize(.small) }
                if workspace.isConnected {
                    Button { workspace.requestReload() } label: { Image(systemName: "arrow.clockwise") }
                        .accessibilityLabel("Recargar espacio de etiquetas")
                        .disabled(workspace.isLoading)
                }
                Button {
                    workspace.prepareConfiguration()
                    showingConfiguration = true
                } label: { Image(systemName: "network") }
                    .accessibilityLabel("Configurar conexión con la Mac")
            }
            .padding(.horizontal, 16)
            .padding(.vertical, 10)
            .background(.ultraThinMaterial)

            if workspace.hasPage {
                LabelingWebView(webView: workspace.webView)
            } else {
                VStack(spacing: 18) {
                    Image(systemName: "cube.transparent")
                        .font(.system(size: 54, weight: .light))
                        .foregroundStyle(.cyan)
                    Text("Revisa tus capturas").font(.title2.bold())
                    Text("Selecciona grupos, confirma suelo, paredes y techo, y etiqueta objetos para preparar nuestros datos de entrenamiento.")
                        .multilineTextAlignment(.center)
                        .foregroundStyle(.secondary)
                    if let error = workspace.errorText {
                        Text(error).font(.callout).foregroundStyle(.orange)
                            .multilineTextAlignment(.center)
                    }
                    Button("Conectar con la Mac") {
                        workspace.prepareConfiguration()
                        showingConfiguration = true
                    }
                    .buttonStyle(.borderedProminent)
                    .disabled(workspace.isLoading)
                    Text("Las etiquetas y los archivos exportados se guardan en la Mac. Ambos dispositivos deben estar en la misma red.")
                        .font(.caption).foregroundStyle(.secondary)
                        .multilineTextAlignment(.center)
                }
                .padding(28)
                .frame(maxWidth: 520)
                .frame(maxWidth: .infinity, maxHeight: .infinity)
            }
        }
        .background(Color(uiColor: .systemBackground))
        .task { workspace.start() }
        .confirmationDialog("Hay puntos seleccionados sin guardar", isPresented: $workspace.showDiscardDraftConfirmation,
                            titleVisibility: .visible) {
            Button("Descartar selección y recargar", role: .destructive) { workspace.reload() }
            Button("Seguir seleccionando", role: .cancel) {}
        } message: {
            Text("Las etiquetas guardadas permanecen en la Mac. Sólo se descartará la selección de puntos que estás preparando.")
        }
        .sheet(isPresented: $showingConfiguration) {
            NavigationStack {
                Form {
                    Section("Enlace de la Mac") {
                        TextField("http://tu-mac.local:8770/#token=…", text: $workspace.draftURL, axis: .vertical)
                            .textInputAutocapitalization(.never)
                            .autocorrectionDisabled()
                            .keyboardType(.URL)
                            .textContentType(.URL)
                            .accessibilityIdentifier("labelingPairURL")
                        Text("Pega el enlace de emparejamiento que muestra el servidor de etiquetas en la Mac. La clave queda guardada en el llavero de este dispositivo.")
                            .font(.caption).foregroundStyle(.secondary)
                        if let error = workspace.errorText {
                            Text(error).font(.caption).foregroundStyle(.orange)
                        }
                        Button("Conectar") {
                            if workspace.connectDraft() { showingConfiguration = false }
                        }
                        .disabled(workspace.draftURL.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty)
                        .accessibilityIdentifier("labelingConnect")
                    }
                    Section("Macs en la red") {
                        if workspace.discoveredServers.isEmpty {
                            Text("Buscando el servidor de etiquetas… Puedes pegar el enlace aunque no aparezca aquí.")
                                .font(.caption).foregroundStyle(.secondary)
                        }
                        ForEach(workspace.discoveredServers) { server in
                            Button {
                                workspace.draftURL = server.baseURL.absoluteString
                                workspace.errorText = nil
                            } label: {
                                VStack(alignment: .leading, spacing: 3) {
                                    Text(server.name)
                                    Text(server.baseURL.host ?? "Mac local").font(.caption).foregroundStyle(.secondary)
                                }
                            }
                        }
                        Text("El descubrimiento encuentra la dirección. Para emparejar por primera vez también necesitas la clave del enlace.")
                            .font(.caption).foregroundStyle(.secondary)
                    }
                    if workspace.baseURL != nil {
                        Section {
                            Button("Desconectar", role: .destructive) {
                                workspace.disconnect()
                                showingConfiguration = false
                            }
                        }
                    }
                }
                .navigationTitle("Conectar etiquetas")
                .navigationBarTitleDisplayMode(.inline)
                .toolbar { ToolbarItem(placement: .cancellationAction) { Button("Cerrar") { showingConfiguration = false } } }
            }
        }
        .sheet(item: $workspace.shareFile) { file in
            LabelingShareSheet(url: file.url)
        }
    }
}

private struct LabelingWebView: UIViewRepresentable {
    let webView: WKWebView
    func makeUIView(context: Context) -> WKWebView { webView }
    func updateUIView(_ uiView: WKWebView, context: Context) {}
}

private struct LabelingShareFile: Identifiable {
    let id = UUID()
    let url: URL
}

private struct LabelingShareSheet: UIViewControllerRepresentable {
    let url: URL
    func makeUIViewController(context: Context) -> UIActivityViewController {
        UIActivityViewController(activityItems: [url], applicationActivities: nil)
    }
    func updateUIViewController(_ controller: UIActivityViewController, context: Context) {}
}

private struct LabelingServer: Identifiable {
    let id: String
    let name: String
    let baseURL: URL
}

private struct LabelingAddress {
    let baseURL: URL
    let token: String?

    var origin: String {
        Self.origin(baseURL)
    }

    static func origin(_ url: URL) -> String {
        "\(url.scheme?.lowercased() ?? "http")://\(url.host?.lowercased() ?? ""):\(url.port ?? (url.scheme == "https" ? 443 : 80))"
    }

    static func parse(_ input: String) throws -> LabelingAddress {
        var text = input.trimmingCharacters(in: .whitespacesAndNewlines)
        if !text.contains("://") { text = "http://" + text }
        guard var components = URLComponents(string: text),
              let scheme = components.scheme?.lowercased(), ["http", "https"].contains(scheme),
              let host = components.host, isLocalHost(host),
              components.user == nil, components.password == nil, components.query == nil,
              ["", "/", "/annotation.html"].contains(components.path) else {
            throw LabelingError.message("Usa el enlace local de la Mac: una dirección privada de la red o un nombre terminado en .local.")
        }
        if components.port == nil { components.port = 8770 }
        guard let port = components.port, (1...65535).contains(port) else {
            throw LabelingError.message("El puerto del servidor no es válido.")
        }
        let items = URLComponents(string: "?" + (components.fragment ?? ""))?.queryItems ?? []
        let values = items.filter { $0.name == "token" }
        let token = values.first?.value
        if values.count > 1 || (token != nil && !validToken(token!)) {
            throw LabelingError.message("La clave del enlace está incompleta. Vuelve a copiar el enlace de emparejamiento de la Mac.")
        }
        components.scheme = scheme
        components.host = host.lowercased()
        components.path = "/"
        components.fragment = nil
        guard let url = components.url else { throw LabelingError.message("La dirección del servidor no es válida.") }
        return LabelingAddress(baseURL: url, token: token)
    }

    static func validToken(_ token: String) -> Bool {
        (16...512).contains(token.utf8.count) && token.utf8.allSatisfy {
            (65...90).contains($0) || (97...122).contains($0) || (48...57).contains($0) || $0 == 45 || $0 == 95
        }
    }

    static func isLoopback(_ host: String) -> Bool {
        let text = host.lowercased().trimmingCharacters(in: CharacterSet(charactersIn: "[]"))
        if text == "::1" || text == "localhost" { return true }
        let parts = text.split(separator: ".", omittingEmptySubsequences: false)
        return parts.count == 4 && parts.first == "127" && parts.allSatisfy { part in
            guard let value = Int(part), (0...255).contains(value) else { return false }
            return String(value) == part
        }
    }

    static func isLocalHost(_ hostname: String) -> Bool {
        let host = hostname.lowercased().trimmingCharacters(in: CharacterSet(charactersIn: "[]"))
        if isLoopback(host) { return true }
        if host.hasSuffix(".local") {
            let parts = host.split(separator: ".", omittingEmptySubsequences: false)
            return parts.count >= 2 && parts.allSatisfy { part in
                !part.isEmpty && part.count <= 63 && part.utf8.allSatisfy {
                    (97...122).contains($0) || (48...57).contains($0) || $0 == 45
                }
            }
        }
        let parts = host.split(separator: ".", omittingEmptySubsequences: false)
        guard parts.count == 4 else { return false }
        let values = parts.compactMap { Int($0) }
        guard values.count == 4, values.allSatisfy({ (0...255).contains($0) }),
              zip(parts, values).allSatisfy({ String($0.1) == $0.0 }) else { return false }
        return values[0] == 10 || (values[0] == 172 && (16...31).contains(values[1])) ||
            (values[0] == 192 && values[1] == 168) || (values[0] == 169 && values[1] == 254)
    }

    func url(with token: String?) -> URL {
        guard let token else { return baseURL }
        var components = URLComponents(url: baseURL, resolvingAgainstBaseURL: false)!
        components.fragment = "token=" + token
        return components.url!
    }
}

private enum LabelingError: LocalizedError {
    case message(String)
    var errorDescription: String? {
        switch self { case .message(let text): text }
    }
}

private enum LabelingKeychain {
    private static let service = "com.atlas.viewer.labeling"

    static func read(origin: String) -> String? {
        let query: [String: Any] = [kSecClass as String: kSecClassGenericPassword,
                                  kSecAttrService as String: service, kSecAttrAccount as String: origin,
                                  kSecReturnData as String: true, kSecMatchLimit as String: kSecMatchLimitOne]
        var result: CFTypeRef?
        guard SecItemCopyMatching(query as CFDictionary, &result) == errSecSuccess,
              let data = result as? Data else { return nil }
        return String(data: data, encoding: .utf8)
    }

    static func write(_ token: String, origin: String) throws {
        let query: [String: Any] = [kSecClass as String: kSecClassGenericPassword,
                                  kSecAttrService as String: service, kSecAttrAccount as String: origin]
        let attributes: [String: Any] = [kSecValueData as String: Data(token.utf8),
                                       kSecAttrAccessible as String: kSecAttrAccessibleWhenUnlockedThisDeviceOnly]
        var status = SecItemUpdate(query as CFDictionary, attributes as CFDictionary)
        if status == errSecItemNotFound {
            status = SecItemAdd(query.merging(attributes) { _, new in new } as CFDictionary, nil)
        }
        guard status == errSecSuccess else { throw LabelingError.message("No se pudo guardar la clave en el llavero. Desbloquea el dispositivo e inténtalo de nuevo.") }
    }
}

/// Never follow a discovery/proxy redirect with a pairing credential to another origin.
private final class LabelingRedirectGuard: NSObject, URLSessionTaskDelegate {
    let origin: String
    init(origin: String) { self.origin = origin }
    func urlSession(_ session: URLSession, task: URLSessionTask,
                    willPerformHTTPRedirection response: HTTPURLResponse,
                    newRequest request: URLRequest,
                    completionHandler: @escaping (URLRequest?) -> Void) {
        guard let url = request.url, LabelingAddress.origin(url) == origin else {
            completionHandler(nil)
            return
        }
        completionHandler(request)
    }
}

@MainActor
private final class LabelingWorkspaceModel: NSObject, ObservableObject {
    @Published var statusText = "Sin conexión con el servidor de etiquetas"
    @Published var errorText: String?
    @Published var isLoading = false
    @Published var isConnected = false
    @Published var hasPage = false
    @Published var draftURL = ""
    @Published var discoveredServers: [LabelingServer] = []
    @Published var shareFile: LabelingShareFile?
    @Published var showDiscardDraftConfirmation = false
    private(set) var baseURL: URL?
    let webView: WKWebView
    private let browser = NetServiceBrowser()
    private var services: [NetService] = []
    private var started = false
    private var connectionTask: Task<Void, Never>?
    private var loadGeneration = 0
    private var currentToken: String?
    private var downloadURLs: [ObjectIdentifier: URL] = [:]
    private static let defaultsKey = "atlas.labeling.baseURL"

    override init() {
        let configuration = WKWebViewConfiguration()
        // Pairing secrets remain in Keychain; web session state does not persist to disk.
        configuration.websiteDataStore = .nonPersistent()
        webView = WKWebView(frame: .zero, configuration: configuration)
        super.init()
        webView.navigationDelegate = self
        webView.uiDelegate = self
        webView.isOpaque = false
        webView.backgroundColor = .systemBackground
        webView.scrollView.backgroundColor = .systemBackground
        // The page keeps the 3D viewport fixed and scrolls only its controls.
        webView.scrollView.bounces = false
        webView.scrollView.alwaysBounceVertical = false
        webView.allowsBackForwardNavigationGestures = false
        if let saved = UserDefaults.standard.string(forKey: Self.defaultsKey),
           let address = try? LabelingAddress.parse(saved) { baseURL = address.baseURL }
    }

    func start() {
        guard !started else { return }
        started = true
        browser.delegate = self
        browser.searchForServices(ofType: "_atlas-labels._tcp.", inDomain: "local.")
        let args = ProcessInfo.processInfo.arguments
        if let index = args.firstIndex(of: "--atlas-label-url"), args.indices.contains(index + 1) {
            draftURL = args[index + 1]
            _ = connectDraft()
            // The complete link is never retained in UserDefaults or status messages.
            draftURL = baseURL?.absoluteString ?? ""
        } else if let baseURL {
            draftURL = baseURL.absoluteString
            _ = connectDraft()
        }
    }

    func prepareConfiguration() {
        draftURL = baseURL?.absoluteString ?? ""
    }

    @discardableResult
    func connectDraft() -> Bool {
        do {
            let address = try LabelingAddress.parse(draftURL)
            let token = address.token ?? LabelingKeychain.read(origin: address.origin)
            guard LabelingAddress.isLoopback(address.baseURL.host ?? "") || token != nil else {
                throw LabelingError.message("Falta la clave de emparejamiento. Pega el enlace completo que muestra la Mac, incluida la parte #token=…")
            }
            if let token { try LabelingKeychain.write(token, origin: address.origin) }
            UserDefaults.standard.set(address.baseURL.absoluteString, forKey: Self.defaultsKey)
            baseURL = address.baseURL
            currentToken = token
            draftURL = address.baseURL.absoluteString
            connect(address, token: token)
            return true
        } catch {
            errorText = error.localizedDescription
            statusText = "Revisa el enlace de conexión"
            return false
        }
    }

    private func connect(_ address: LabelingAddress, token: String?) {
        connectionTask?.cancel()
        loadGeneration += 1
        let generation = loadGeneration
        webView.stopLoading()
        hasPage = false
        isConnected = false
        isLoading = true
        errorText = nil
        statusText = "Conectando con \(address.baseURL.host ?? "la Mac")…"
        connectionTask = Task { [weak self] in
            guard let self else { return }
            let configuration = URLSessionConfiguration.ephemeral
            configuration.timeoutIntervalForRequest = 12
            let session = URLSession(configuration: configuration,
                                     delegate: LabelingRedirectGuard(origin: address.origin),
                                     delegateQueue: nil)
            defer { session.invalidateAndCancel() }
            var request = URLRequest(url: address.baseURL.appendingPathComponent("api/datasets"))
            if let token { request.setValue("Bearer " + token, forHTTPHeaderField: "Authorization") }
            do {
                let (_, response) = try await session.data(for: request)
                guard !Task.isCancelled, generation == self.loadGeneration else { return }
                guard let http = response as? HTTPURLResponse else { throw LabelingError.message("La Mac no devolvió una respuesta válida.") }
                if http.statusCode == 401 || http.statusCode == 403 {
                    throw LabelingError.message("La clave no fue aceptada. Copia de nuevo el enlace de emparejamiento del servidor de la Mac.")
                }
                guard http.statusCode == 200 else { throw LabelingError.message("El servidor de etiquetas respondió con un error. Comprueba que está iniciado en la Mac.") }
                self.hasPage = true
                self.webView.load(URLRequest(url: address.url(with: token)))
            } catch {
                guard !Task.isCancelled, generation == self.loadGeneration else { return }
                self.isLoading = false
                self.errorText = (error as? LabelingError)?.localizedDescription ?? "No se pudo conectar. Comprueba que la Mac y este dispositivo estén en la misma red y que el servidor de etiquetas siga abierto."
                self.statusText = "Sin conexión con la Mac"
            }
        }
    }

    func requestReload() {
        guard hasPage, !isLoading else { return }
        let generation = loadGeneration
        webView.evaluateJavaScript("Boolean(window.atlasAnnotationHasDraft)") { [weak self] result, _ in
            Task { @MainActor in
                guard let self, generation == self.loadGeneration, self.hasPage else { return }
                if result as? Bool == true { self.showDiscardDraftConfirmation = true }
                else { self.reload() }
            }
        }
    }

    func reload() {
        showDiscardDraftConfirmation = false
        guard let baseURL else { return }
        connect(LabelingAddress(baseURL: baseURL, token: currentToken), token: currentToken)
    }

    func disconnect() {
        connectionTask?.cancel()
        loadGeneration += 1
        webView.stopLoading()
        webView.loadHTMLString("", baseURL: nil)
        hasPage = false
        isConnected = false
        isLoading = false
        errorText = nil
        statusText = "Desconectado · las etiquetas permanecen en la Mac"
    }

    private func permits(_ url: URL) -> Bool {
        guard let baseURL else { return false }
        if url.scheme == "blob" {
            guard let nested = URL(string: String(url.absoluteString.dropFirst(5))) else { return false }
            return LabelingAddress.origin(nested) == LabelingAddress.origin(baseURL)
        }
        return ["http", "https"].contains(url.scheme?.lowercased() ?? "") && LabelingAddress.origin(url) == LabelingAddress.origin(baseURL)
    }

    private func navigationFailed(_ error: Error) {
        if (error as NSError).code == NSURLErrorCancelled { return }
        isLoading = false
        isConnected = false
        hasPage = false
        errorText = "La conexión con la Mac se interrumpió. Puedes volver a conectar; las etiquetas guardadas permanecen allí."
        statusText = "No se pudo abrir el espacio de etiquetas"
    }
}

// NetService callbacks use the main run loop on which browsing/resolution starts.
extension LabelingWorkspaceModel: @preconcurrency NetServiceBrowserDelegate, @preconcurrency NetServiceDelegate {
    func netServiceBrowser(_ browser: NetServiceBrowser, didFind service: NetService, moreComing: Bool) {
        services.append(service)
        service.delegate = self
        service.resolve(withTimeout: 5)
    }

    func netServiceBrowser(_ browser: NetServiceBrowser, didRemove service: NetService, moreComing: Bool) {
        services.removeAll { $0 == service }
        discoveredServers.removeAll { $0.id == service.name + "|" + service.domain }
    }

    func netServiceDidResolveAddress(_ sender: NetService) {
        guard let host = sender.hostName?.trimmingCharacters(in: CharacterSet(charactersIn: ".")),
              LabelingAddress.isLocalHost(host), (1...65535).contains(sender.port) else { return }
        var components = URLComponents()
        components.scheme = "http"
        components.host = host
        components.port = sender.port
        components.path = "/"
        guard let url = components.url else { return }
        let item = LabelingServer(id: sender.name + "|" + sender.domain, name: sender.name, baseURL: url)
        discoveredServers.removeAll { $0.id == item.id }
        discoveredServers.append(item)
        discoveredServers.sort { $0.name.localizedStandardCompare($1.name) == .orderedAscending }
    }
}

extension LabelingWorkspaceModel: WKNavigationDelegate, WKUIDelegate, WKDownloadDelegate {
    func webView(_ webView: WKWebView, decidePolicyFor navigationAction: WKNavigationAction, decisionHandler: @escaping (WKNavigationActionPolicy) -> Void) {
        guard let url = navigationAction.request.url else { decisionHandler(.cancel); return }
        if url.absoluteString == "about:blank", !hasPage { decisionHandler(.allow); return }
        guard permits(url) else {
            errorText = "Este espacio solo abre el servidor de etiquetas con el que estás conectado."
            decisionHandler(.cancel)
            return
        }
        decisionHandler(navigationAction.shouldPerformDownload ? .download : .allow)
    }

    func webView(_ webView: WKWebView, decidePolicyFor navigationResponse: WKNavigationResponse, decisionHandler: @escaping (WKNavigationResponsePolicy) -> Void) {
        guard let url = navigationResponse.response.url, permits(url) else { decisionHandler(.cancel); return }
        decisionHandler(navigationResponse.canShowMIMEType ? .allow : .download)
    }

    func webView(_ webView: WKWebView, didStartProvisionalNavigation navigation: WKNavigation!) {
        if hasPage { isLoading = true }
    }

    func webView(_ webView: WKWebView, didFinish navigation: WKNavigation!) {
        guard hasPage else { return }
        isLoading = false
        isConnected = true
        errorText = nil
        statusText = "\(baseURL?.host ?? "Mac") · etiquetas guardadas en la Mac"
    }

    func webView(_ webView: WKWebView, didFail navigation: WKNavigation!, withError error: Error) { navigationFailed(error) }
    func webView(_ webView: WKWebView, didFailProvisionalNavigation navigation: WKNavigation!, withError error: Error) { navigationFailed(error) }
    func webViewWebContentProcessDidTerminate(_ webView: WKWebView) {
        isLoading = false
        isConnected = false
        hasPage = false
        errorText = "La vista se cerró por falta de recursos. Vuelve a conectar para continuar."
        statusText = "Vista de etiquetas detenida"
    }

    func webView(_ webView: WKWebView, createWebViewWith configuration: WKWebViewConfiguration, for navigationAction: WKNavigationAction, windowFeatures: WKWindowFeatures) -> WKWebView? {
        if let url = navigationAction.request.url, permits(url) { webView.load(navigationAction.request) }
        return nil
    }

    func webView(_ webView: WKWebView, requestMediaCapturePermissionFor origin: WKSecurityOrigin, initiatedByFrame frame: WKFrameInfo, type: WKMediaCaptureType, decisionHandler: @escaping (WKPermissionDecision) -> Void) {
        decisionHandler(.deny)
    }

    func webView(_ webView: WKWebView, navigationAction: WKNavigationAction, didBecome download: WKDownload) { download.delegate = self }
    func webView(_ webView: WKWebView, navigationResponse: WKNavigationResponse, didBecome download: WKDownload) { download.delegate = self }

    func download(_ download: WKDownload, decideDestinationUsing response: URLResponse, suggestedFilename: String, completionHandler: @escaping (URL?) -> Void) {
        let filename = String(URL(fileURLWithPath: suggestedFilename).lastPathComponent.prefix(180))
        guard !filename.isEmpty, filename != ".", filename != ".." else { completionHandler(nil); return }
        let directory = FileManager.default.temporaryDirectory.appendingPathComponent("ATLAS-Labels-" + UUID().uuidString, isDirectory: true)
        do {
            try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
            let url = directory.appendingPathComponent(filename)
            downloadURLs[ObjectIdentifier(download)] = url
            completionHandler(url)
        } catch {
            errorText = "El archivo sigue guardado en la Mac, pero no se pudo descargar en este dispositivo."
            completionHandler(nil)
        }
    }

    func downloadDidFinish(_ download: WKDownload) {
        guard let url = downloadURLs.removeValue(forKey: ObjectIdentifier(download)) else { return }
        shareFile = LabelingShareFile(url: url)
    }

    func download(_ download: WKDownload, didFailWithError error: Error, resumeData: Data?) {
        downloadURLs.removeValue(forKey: ObjectIdentifier(download))
        errorText = "No se pudo descargar el archivo. La exportación permanece guardada en la Mac."
    }
}
