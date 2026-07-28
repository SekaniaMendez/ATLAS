import SwiftUI

struct ContentView: View {
    @ObservedObject var pointCloud: PointCloudModel
    @ObservedObject var connection: AtlasConnection
    @ObservedObject var calibration: CalibrationStore
    @ObservedObject var camera: CameraSessionController
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

    private var header: some View {
        HStack {
            VStack(alignment: .leading, spacing: 3) {
                Text("ATLAS")
                    .font(.title2.bold())
                Text(connection.statusText)
                    .font(.caption)
                    .foregroundStyle(connection.isConnected ? .green : .secondary)
                Text(camera.statusText)
                    .font(.caption2)
                    .foregroundStyle(camera.isRunning ? .cyan : .secondary)
            }
            Spacer()
            VStack(alignment: .trailing, spacing: 3) {
                Text("\(pointCloud.pointCount.formatted()) puntos")
                Text("frame \(pointCloud.sequence)")
                    .foregroundStyle(.secondary)
            }
            .font(.caption.monospacedDigit())
        }
        .padding(12)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 16))
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
                }
                .fixedSize(horizontal: true, vertical: false)
            }
            .buttonStyle(.bordered)

            HStack {
                Text(camera.isRunning
                     ? "\(camera.displayMode.title) · apunta a bordes y superficies"
                     : "Arrastra para rotar · pellizca para zoom")
                Spacer()
                if !calibration.selectedProfile.verified {
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
