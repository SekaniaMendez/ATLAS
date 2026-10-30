import Foundation
import SwiftUI

struct ContentView: View {
    @ObservedObject var pointCloud: PointCloudModel
    @ObservedObject var geometry: LiveGeometryModel
    @ObservedObject var connection: AtlasConnection
    @ObservedObject var calibration: CalibrationStore
    @ObservedObject var camera: CameraSessionController
    @ObservedObject var capture: CameraCaptureRecorder
    @AppStorage("atlas.viewer.manualHost") private var host = ""
    @State private var showSettings = false
    @State private var showPoints = true
    @State private var showSurfaces = true
    @State private var showAxes = false
    @State private var showCeiling = false
    @State private var topViewRequest = 0
    @State private var selectedWorkspace = Workspace.map

    private enum Workspace: Hashable {
        case map
        case labeling
    }

    var body: some View {
        TabView(selection: $selectedWorkspace) {
            mapWorkspace
                .tabItem { Label("Mapa", systemImage: "map") }
                .tag(Workspace.map)

            LabelingWorkspaceView()
                .tabItem { Label("Etiquetar", systemImage: "tag") }
                .tag(Workspace.labeling)
        }
        .tint(.cyan)
        .preferredColorScheme(.dark)
        .sheet(isPresented: $showSettings) { settings }
        .onAppear {
            configureCameraFusion()
            connection.startBrowsingIfNeeded()
            if ProcessInfo.processInfo.arguments.contains("--atlas-label-tab") {
                selectedWorkspace = .labeling
            }
        }
        .onChange(of: calibration.selectedProfileID) { _, _ in configureCameraFusion() }
    }

    private var mapWorkspace: some View {
        GeometryReader { proxy in
            ZStack {
                Color.black.ignoresSafeArea()
                MetalPointCloudView(
                    model: pointCloud,
                    geometry: geometry,
                    camera: camera,
                    calibration: calibration,
                    showPoints: isSlamView ? showPoints : true,
                    showSurfaces: showSurfaces,
                    showAxes: showAxes,
                    showCeiling: showCeiling,
                    ceilingCutHeightM: geometry.ceilingCutHeightM,
                    topViewRequest: topViewRequest
                )
                .ignoresSafeArea(edges: .top)
                .accessibilityLabel("Mapa tridimensional")

                VStack(spacing: 8) {
                    mapHeader
                    if let message = trackingWarning {
                        compactNotice(message, symbol: "exclamationmark.triangle.fill", color: .orange)
                            .accessibilityIdentifier("slamTrackingStatus")
                    }
                    if let warning = connection.activeWarning {
                        compactNotice(warningTitle(warning.code), symbol: "figure.walk.motion", color: .orange)
                            .accessibilityLabel("\(warningTitle(warning.code)). \(warningDetail(warning))")
                    }
                    Spacer(minLength: 12)
                    if camera.isRunning && (isSlamView || camera.displayMode != .calibrationOverlay) {
                        Button {
                            if isSlamView && camera.cameraPreviewRequested { camera.showMapView() }
                            else { camera.showCameraView() }
                        } label: {
                            Label(isSlamView && camera.cameraPreviewRequested ? "Volver al mapa" : "Ver cámara",
                                  systemImage: isSlamView && camera.cameraPreviewRequested ? "map" : "camera")
                                .font(.caption.weight(.semibold))
                                .padding(.horizontal, 16)
                                .padding(.vertical, 10)
                                .background(.ultraThinMaterial, in: Capsule())
                        }
                        .accessibilityIdentifier("showCalibrationCamera")
                    }
                    if capture.isRecording || capture.isFinishing {
                        recordingIndicator
                    }
                    mapFooter
                }
                .frame(maxWidth: 680)
                .padding(.horizontal, proxy.size.width < 400 ? 12 : 20)
                .padding(.top, 8)
                .padding(.bottom, 10)
                .frame(maxWidth: .infinity, maxHeight: .infinity)
            }
        }
    }

    private var mapHeader: some View {
        HStack(spacing: 12) {
            VStack(alignment: .leading, spacing: 3) {
                Text("ATLAS")
                    .font(.headline.weight(.bold))
                    .tracking(2)
                HStack(spacing: 5) {
                    Circle().fill(trackingColor).frame(width: 6, height: 6)
                    Text(trackingTitle)
                        .font(.caption)
                        .lineLimit(1)
                }
                .foregroundStyle(trackingColor)
                .accessibilityElement(children: .combine)
                .accessibilityIdentifier("slamTrackingState")
            }
            Spacer(minLength: 8)
            Button {
                topViewRequest &+= 1
            } label: {
                Image(systemName: "square.3.layers.3d.top.filled")
                    .frame(width: 44, height: 44)
                    .background(.white.opacity(0.07), in: Circle())
            }
            .accessibilityLabel("Vista superior")
            .accessibilityIdentifier("mapTopView")
            .disabled(!isSlamView)
            Menu {
                Toggle("Puntos", isOn: $showPoints)
                    .accessibilityIdentifier("showMapPoints")
                Toggle("Superficies", isOn: $showSurfaces)
                    .accessibilityIdentifier("showLiveSurfaces")
                Toggle("Ejes de referencia", isOn: $showAxes)
                    .accessibilityIdentifier("showReferenceAxes")
                Toggle("Mostrar techo", isOn: $showCeiling)
                    .accessibilityIdentifier("showMapCeiling")
                Text("Ocultarlo aplica un corte superior estimado; no cambia las mediciones.")
            } label: {
                Image(systemName: "square.3.layers.3d")
                    .frame(width: 44, height: 44)
                    .background(.white.opacity(0.07), in: Circle())
            }
            .accessibilityLabel("Capas del mapa")
            .disabled(!isSlamView)
            Button {
                showSettings = true
            } label: {
                Image(systemName: "slider.horizontal.3")
                    .frame(width: 44, height: 44)
                    .background(.white.opacity(0.07), in: Circle())
            }
            .accessibilityLabel("Ajustes y conexión")
            .accessibilityIdentifier("openViewerSettings")
        }
        .foregroundStyle(.white)
        .padding(.leading, 14)
        .padding(.trailing, 6)
        .padding(.vertical, 6)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 20))
    }

    private var mapFooter: some View {
        VStack(spacing: 4) {
            HStack(spacing: 6) {
                Text("\(pointCloud.pointCount.formatted()) puntos")
                if isSlamView {
                    Text("·")
                    Text("\(geometry.surfaceCount) superficies")
                }
                if geometry.truncated {
                    Image(systemName: "ellipsis.circle")
                        .foregroundStyle(.orange)
                        .accessibilityLabel("Vista de superficies limitada")
                }
            }
            .font(.caption.monospacedDigit())
            .accessibilityIdentifier("liveGeometryStatus")
            if isSlamView && !showCeiling {
                Text(geometry.ceilingCutHeightM == nil ? "Buscando altura del techo…" : "Corte superior estimado · techo oculto")
                    .font(.caption2)
                    .foregroundStyle(.secondary)
                    .accessibilityIdentifier("ceilingCutStatus")
            }
            Text(camera.isRunning && isSlamView && camera.cameraPreviewRequested ? "Vista de cámara para encuadrar"
                 : camera.isRunning && !isSlamView ? (camera.displayMode == .calibrationOverlay
                 ? (calibration.selectedProfile.verified ? "Cámara · superposición LiDAR"
                    : "Cámara · superposición pendiente de calibrar") : "Nube RGB · usa Ver cámara para encuadrar")
                 : isSlamView || connection.isConnected ? "Arrastra para girar · pellizca para acercar"
                 : "Vista de prueba · conecta la Mac en Ajustes")
                .font(.caption2)
                .foregroundStyle(.secondary)
                .multilineTextAlignment(.center)
        }
        .padding(.horizontal, 14)
        .padding(.vertical, 8)
        .background(.ultraThinMaterial, in: Capsule())
        .allowsHitTesting(false)
    }

    private var recordingIndicator: some View {
        HStack(spacing: 8) {
            Circle().fill(.red).frame(width: 7, height: 7)
            Text(capture.isFinishing ? "Guardando captura…" : "Grabando cámara")
                .font(.caption.weight(.semibold))
            if capture.isRecording {
                Button("Detener", role: .destructive) { capture.stop() }
                    .font(.caption.weight(.semibold))
                    .padding(.vertical, 8)
            }
        }
        .padding(.horizontal, 14)
        .padding(.vertical, 4)
        .background(.ultraThinMaterial, in: Capsule())
    }

    private func compactNotice(_ text: String, symbol: String, color: Color) -> some View {
        HStack(spacing: 8) {
            Image(systemName: symbol)
            Text(text)
                .font(.caption.weight(.medium))
                .fixedSize(horizontal: false, vertical: true)
            Spacer(minLength: 0)
        }
        .foregroundStyle(color)
        .padding(.horizontal, 12)
        .padding(.vertical, 9)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))
        .accessibilityElement(children: .combine)
    }

    private var isSlamView: Bool {
        pointCloud.isWorldFrame || connection.slamStatus != nil || geometry.surfaceCount > 0
    }

    private var trackingTitle: String {
        guard connection.isConnected else {
            return isSlamView ? "Sin conexión" : connection.statusText
        }
        guard let status = connection.slamStatus else {
            return isSlamView ? "Esperando estado SLAM" : "Sensor conectado"
        }
        if status.fatalError { return "SLAM detenido" }
        if status.state != .lost && status.message.hasPrefix("OFFLINE GEOMETRY PREVIEW:") {
            return "Reproducción de captura"
        }
        switch status.state {
        case .tracking: return "Seguimiento activo"
        case .lost: return "Seguimiento perdido"
        case .initializing: return "Inicializando SLAM"
        case .uninitialized: return "Esperando LiDAR"
        }
    }

    private var trackingColor: Color {
        guard connection.isConnected else { return isSlamView ? .orange : .secondary }
        guard let status = connection.slamStatus else { return isSlamView ? .orange : .cyan }
        if status.fatalError || status.state == .lost { return .red }
        if status.message.hasPrefix("OFFLINE GEOMETRY PREVIEW:") { return .cyan }
        return status.state == .tracking ? .green : .orange
    }

    private var trackingWarning: String? {
        if isSlamView && !connection.isConnected {
            return "Sin conexión · se muestra el último mapa recibido."
        }
        if let status = connection.slamStatus, status.fatalError || status.state == .lost {
            return "El mapa no se está actualizando · se conserva el último estado válido."
        }
        if isSlamView && connection.isConnected && connection.slamStatus == nil {
            return "Seguimiento sin confirmar · esperando el estado de la Mac."
        }
        return nil
    }

    private var settings: some View {
        NavigationStack {
            Form {
                connectionSettings
                cameraSettings
                calibrationSettings
                telemetrySettings
                Section {
                    Button("Abrir mapa de demostración") {
                        connection.disconnect()
                        geometry.clear(statusText: "Geometría · Modo demo", resetOrdering: true)
                        pointCloud.loadDemoCloud()
                        showPoints = true
                        showSettings = false
                    }
                }
            }
            .navigationTitle("Ajustes")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .confirmationAction) {
                    Button("Listo") { showSettings = false }
                }
            }
        }
        .presentationDetents([.large])
        .presentationDragIndicator(.visible)
    }

    private var connectionSettings: some View {
        Section {
            Label(connection.statusText, systemImage: connection.isConnected ? "network" : "wifi")
                .font(.subheadline)
                .foregroundStyle(connection.isConnected ? Color.green : Color.secondary)
            TextField("IP de la Mac", text: $host)
                .textInputAutocapitalization(.never)
                .autocorrectionDisabled()
                .keyboardType(.numbersAndPunctuation)
                .submitLabel(.go)
                .onSubmit { connectToHost() }
                .accessibilityIdentifier("macHostField")
            Button("Conectar a la Mac") { connectToHost() }
                .disabled(host.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty)
            Button("Buscar Mac en la red") { connection.startBrowsing() }
            Button("Desconectar", role: .destructive) { connection.disconnect() }
        } header: {
            Text("Conexión")
        } footer: {
            Text("La Mac ejecuta el SLAM. Este dispositivo recibe el mapa por la red local.")
        }
    }

    private var cameraSettings: some View {
        Section {
            Toggle("Activar cámara", isOn: Binding(
                get: { camera.isRunning },
                set: { enabled in enabled ? camera.start() : camera.pause() }
            ))
            .disabled(capture.isRecording || capture.isFinishing)
            Text(camera.statusText)
                .font(.caption)
                .foregroundStyle(.secondary)
            Toggle("Enviar color al filtro", isOn: $camera.fusionEnabled)
            if camera.fusionEnabled && !calibration.selectedProfile.verified {
                Label("Calibración RGB pendiente de verificar", systemImage: "exclamationmark.triangle")
                    .font(.caption)
                    .foregroundStyle(.orange)
            }
            Picker("Lente", selection: Binding(
                get: { camera.lensPreference }, set: { camera.selectLens($0) }
            )) {
                ForEach(CameraLensPreference.allCases) { lens in
                    Text(lens.title).tag(lens)
                }
            }
            .disabled(capture.isRecording || capture.isFinishing)
            Button {
                if isSlamView {
                    if camera.cameraPreviewRequested { camera.showMapView() }
                    else { camera.showCameraView() }
                } else { camera.toggleDisplayMode() }
            } label: {
                LabeledContent("Visualización", value: isSlamView
                    ? (camera.cameraPreviewRequested ? "Cámara en vivo" : "Mapa SLAM")
                    : camera.displayMode.title)
            }
            .disabled(!camera.isRunning)
            Button {
                if capture.isRecording { capture.stop() }
                else { capture.start(profile: calibration.selectedProfile, lens: camera.activeLensIdentifier) }
            } label: {
                Label(capture.isRecording ? "Detener grabación" : "Grabar cámara",
                      systemImage: capture.isRecording ? "stop.circle.fill" : "record.circle")
            }
            .tint(capture.isRecording ? .red : .cyan)
            .disabled(!camera.isRunning || capture.isFinishing)
            Text(capture.statusText)
                .font(.caption)
                .foregroundStyle(capture.isRecording ? Color.red : Color.secondary)
        } header: {
            Text("Cámara")
        } footer: {
            Text("La cámara se activa solo al solicitarlo. Puedes ver el mapa y etiquetar capturas con la cámara apagada.")
        }
    }

    private var calibrationSettings: some View {
        Section {
            Picker("Perfil del dispositivo", selection: $calibration.selectedProfileID) {
                ForEach(calibration.profiles) { profile in
                    Text(profile.displayName).tag(profile.id)
                }
            }
            .disabled(capture.isRecording || capture.isFinishing)
            Label(calibration.selectedProfile.verified ? "Calibración verificada" : "Calibración preliminar",
                  systemImage: calibration.selectedProfile.verified ? "checkmark.seal" : "exclamationmark.triangle")
                .foregroundStyle(calibration.selectedProfile.verified ? Color.green : Color.orange)
            DisclosureGroup("Detalles del perfil") {
                Text(calibration.selectedProfile.notes)
                    .font(.caption)
                    .foregroundStyle(.secondary)
                    .fixedSize(horizontal: false, vertical: true)
            }
        } header: {
            Text("Montaje y calibración")
        }
    }

    private var telemetrySettings: some View {
        Section {
            DisclosureGroup("Estado del escaneo") {
                if let status = connection.slamStatus {
                    Text(status.fatalError ? "SLAM detenido por error" : status.state.title)
                        .foregroundStyle(trackingColor)
                    Text(status.message)
                        .font(.caption)
                        .fixedSize(horizontal: false, vertical: true)
                    LabeledContent("Escaneos aceptados", value: status.acceptedScans.formatted())
                    LabeledContent("Escaneos rechazados", value: status.rejectedScans.formatted())
                } else {
                    Text("La Mac todavía no ha enviado el estado del SLAM.")
                        .font(.caption)
                }
                LabeledContent("Puntos", value: pointCloud.pointCount.formatted())
                LabeledContent("Fotograma", value: String(pointCloud.sequence))
                if let warning = connection.activeWarning {
                    Text("\(warningTitle(warning.code)): \(warningDetail(warning))")
                        .font(.caption)
                        .foregroundStyle(.orange)
                }
            }
            DisclosureGroup("Superficies y referencias") {
                Text(geometry.statusText).font(.caption)
                LabeledContent("Superficies", value: String(geometry.surfaceCount))
                LabeledContent("Triángulos", value: geometry.triangleCount.formatted())
                TimelineView(.periodic(from: .now, by: 1)) { context in
                    if let age = geometry.age(at: context.date) {
                        LabeledContent("Última actualización", value: String(format: "hace %.1f s", max(0, age)))
                    }
                }
                if geometry.truncated {
                    Text("La vista limita superficies para mantener la actualización en vivo.")
                        .font(.caption)
                        .foregroundStyle(.orange)
                }
                Text("Vista provisional · ejes X rojo, Y verde y Z azul.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        } header: {
            Text("Información avanzada")
        }
        .font(.subheadline.monospacedDigit())
    }

    private func connectToHost() {
        let address = host.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !address.isEmpty else { return }
        host = address
        connection.connect(host: address)
        showSettings = false
    }

    private func configureCameraFusion() {
        camera.fusionEncoder.configure(profile: calibration.selectedProfile) { [weak connection] data, done in
            guard let connection else { done(); return }
            connection.sendCameraImage(data, completion: done)
        }
    }

    private func warningTitle(_ code: AtlasSensorWarningCode) -> String {
        switch code {
        case .abruptAcceleration: "Movimiento brusco detectado"
        case .abruptRotation: "Rotación brusca detectada"
        case .excessiveSpeed: "Velocidad elevada"
        }
    }

    private func warningDetail(_ warning: AtlasSensorWarning) -> String {
        switch warning.code {
        case .abruptAcceleration:
            String(format: "Cambio %.1f m/s³ · límite %.1f m/s³", warning.measuredValue, warning.threshold)
        case .abruptRotation:
            String(format: "Giro %.2f rad/s · límite %.2f rad/s", warning.measuredValue, warning.threshold)
        case .excessiveSpeed:
            String(format: "%.2f m/s · límite %.2f m/s", warning.measuredValue, warning.threshold)
        }
    }
}
