import Foundation
import SwiftUI

struct ContentView: View {
    @ObservedObject var pointCloud: PointCloudModel
    @ObservedObject var connection: AtlasConnection
    @ObservedObject var calibration: CalibrationStore
    @ObservedObject var camera: CameraSessionController
    @ObservedObject var capture: CameraCaptureRecorder
    @State private var host = ""

    var body: some View {
        ZStack {
            Color.black.ignoresSafeArea()
            MetalPointCloudView(
                model: pointCloud,
                camera: camera,
                calibration: calibration
            )
                .ignoresSafeArea()

            VStack(spacing: 12) {
                header
                if let warning = connection.activeWarning {
                    warningBanner(warning)
                }
                Spacer()
                controls
            }
            .padding()
        }
        .preferredColorScheme(.dark)
        .onAppear {
            connection.startBrowsing()
            camera.start()
        }
    }

    private func warningBanner(_ warning: AtlasSensorWarning) -> some View {
        HStack(spacing: 10) {
            Image(systemName: "exclamationmark.triangle.fill")
                .font(.title3)
            VStack(alignment: .leading, spacing: 2) {
                Text(warningTitle(warning.code))
                    .font(.subheadline.bold())
                Text(warningDetail(warning))
                    .font(.caption.monospacedDigit())
            }
            Spacer()
        }
        .foregroundStyle(.black)
        .padding(12)
        .background(.orange, in: RoundedRectangle(cornerRadius: 14))
        .accessibilityElement(children: .combine)
    }

    private func warningTitle(_ code: AtlasSensorWarningCode) -> String {
        switch code {
        case .abruptAcceleration:
            "Movimiento brusco detectado"
        case .abruptRotation:
            "Rotación brusca detectada"
        case .excessiveSpeed:
            "Velocidad elevada"
        }
    }

    private func warningDetail(_ warning: AtlasSensorWarning) -> String {
        switch warning.code {
        case .abruptAcceleration:
            String(
                format: "Cambio %.1f m/s³ · límite %.1f m/s³",
                warning.measuredValue,
                warning.threshold
            )
        case .abruptRotation:
            String(
                format: "Giro %.2f rad/s · límite %.2f rad/s",
                warning.measuredValue,
                warning.threshold
            )
        case .excessiveSpeed:
            String(
                format: "%.2f m/s · límite %.2f m/s",
                warning.measuredValue,
                warning.threshold
            )
        }
    }

    private var header: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                VStack(alignment: .leading, spacing: 3) {
                    Text(pointCloud.isWorldFrame || connection.slamStatus != nil ? "ATLAS · Mapa SLAM" : "ATLAS")
                        .font(.title2.bold())
                    Text(connection.statusText)
                        .font(.caption)
                        .foregroundStyle(connection.isConnected ? .green : .secondary)
                    Text(camera.statusText)
                        .font(.caption2)
                        .foregroundStyle(camera.isRunning ? .cyan : .secondary)
                    Text(capture.statusText)
                        .font(.caption2)
                        .foregroundStyle(capture.isRecording ? .red : .secondary)
                }
                Spacer()
                VStack(alignment: .trailing, spacing: 3) {
                    Text("\(pointCloud.pointCount.formatted()) puntos")
                    Text("frame \(pointCloud.sequence)")
                        .foregroundStyle(.secondary)
                }
                .font(.caption.monospacedDigit())
            }
            if pointCloud.isWorldFrame || connection.slamStatus != nil {
                slamTrackingIndicator
            }
        }
        .padding(12)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 16))
    }

    private var slamTrackingIndicator: some View {
        VStack(alignment: .leading, spacing: 3) {
            if !connection.isConnected {
                Text("SLAM · Sin conexión")
                    .foregroundStyle(.red)
                Text("Se muestra el último mapa recibido")
                if let status = connection.slamStatus {
                    DisclosureGroup("Último estado SLAM") {
                        VStack(alignment: .leading, spacing: 4) {
                            Text(status.fatalError ? "SLAM · Detenido por error" : status.state.title)
                            Text(status.message)
                                .fixedSize(horizontal: false, vertical: true)
                        }
                        .frame(maxWidth: .infinity, alignment: .leading)
                    }
                }
            } else if let status = connection.slamStatus {
                Text(status.fatalError ? "SLAM · Detenido por error" : status.state.title)
                    .foregroundStyle(status.fatalError || status.state == .lost
                                     ? .red : status.state == .tracking ? .green : .orange)
                    .font(.caption.bold())
                    .accessibilityIdentifier("slamTrackingState")
                if status.fatalError || status.state == .lost {
                    Text("Último mapa válido · el mapa no se está actualizando")
                        .foregroundStyle(.orange)
                }
                DisclosureGroup("Detalle del seguimiento") {
                    VStack(alignment: .leading, spacing: 4) {
                        Text(status.message)
                            .fixedSize(horizontal: false, vertical: true)
                        Text("Escaneos: \(status.acceptedScans) aceptados · \(status.rejectedScans) rechazados")
                    }
                    .frame(maxWidth: .infinity, alignment: .leading)
                }
            } else {
                Text("SLAM · Estado no disponible")
                    .foregroundStyle(.orange)
                Text("La conexión no confirma el seguimiento")
            }
        }
        .font(.caption2)
        .accessibilityElement(children: .contain)
        .accessibilityIdentifier("slamTrackingStatus")
    }

    private var controls: some View {
        VStack(spacing: 10) {
            HStack {
                TextField("IP de la Mac", text: $host)
                    .textInputAutocapitalization(.never)
                    .keyboardType(.numbersAndPunctuation)
                    .textFieldStyle(.roundedBorder)

                Button("Conectar") {
                    connection.connect(host: host)
                }
                .buttonStyle(.borderedProminent)
                .disabled(host.trimmingCharacters(in: .whitespaces).isEmpty)
            }

            ScrollView(.horizontal, showsIndicators: false) {
                HStack {
                    Button {
                        if capture.isRecording { capture.stop() }
                        else { capture.start(profile: calibration.selectedProfile, lens: camera.activeLensName) }
                    } label: {
                        Label(capture.isRecording ? "Detener captura" : "Grabar cámara",
                              systemImage: capture.isRecording ? "stop.circle.fill" : "record.circle")
                    }
                    .disabled(!camera.isRunning || capture.isFinishing)
                    Button {
                        camera.isRunning ? camera.pause() : camera.start()
                    } label: {
                        Label(
                            camera.isRunning ? "Ocultar cámara" : "Cámara",
                            systemImage: camera.isRunning ? "camera.fill" : "camera"
                        )
                    }
                    Button {
                        camera.toggleDisplayMode()
                    } label: {
                        Label(
                            camera.displayMode.title,
                            systemImage: camera.displayMode == .rgbCloud
                                ? "circle.hexagongrid.fill" : "scope"
                        )
                    }
                    .disabled(pointCloud.isWorldFrame)
                    Menu {
                        ForEach(CameraLensPreference.allCases) { lens in
                            Button {
                                camera.selectLens(lens)
                            } label: {
                                if camera.lensPreference == lens {
                                    Label(lens.title, systemImage: "checkmark")
                                } else {
                                    Text(lens.title)
                                }
                            }
                        }
                    } label: {
                        Label(camera.activeLensName, systemImage: "camera.aperture")
                    }
                    .disabled(capture.isRecording || capture.isFinishing)
                    Button("Buscar Mac") { connection.startBrowsing() }
                    Button("Demo") {
                        connection.disconnect()
                        pointCloud.loadDemoCloud()
                    }
                    Button("Desconectar", role: .destructive) { connection.disconnect() }
                        .disabled(!connection.isConnected)
                    Menu {
                        Picker("Perfil del dispositivo", selection: $calibration.selectedProfileID) {
                            ForEach(calibration.profiles) { profile in
                                Text(profile.displayName).tag(profile.id)
                            }
                        }
                    } label: {
                        Label(calibration.selectedProfile.displayName, systemImage: "iphone")
                    }
                    .disabled(capture.isRecording || capture.isFinishing)
                }
                .fixedSize(horizontal: true, vertical: false)
            }
            .buttonStyle(.bordered)

            HStack {
                Text(pointCloud.isWorldFrame
                     ? "Mapa SLAM local · arrastra para rotar · pellizca para zoom"
                     : camera.isRunning
                        ? "\(camera.displayMode.title) · apunta a bordes y superficies"
                        : "Arrastra para rotar · pellizca para zoom")
                Spacer()
                if !pointCloud.isWorldFrame && !calibration.selectedProfile.verified {
                    Text("Calibración preliminar")
                        .foregroundStyle(.orange)
                }
            }
            .font(.caption2)
            .foregroundStyle(.secondary)
        }
        .padding(12)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 16))
    }
}
