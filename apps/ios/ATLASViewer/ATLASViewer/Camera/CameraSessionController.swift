import ARKit
import AVFoundation
import Foundation

enum CameraDisplayMode: String {
    case rgbCloud
    case calibrationOverlay

    var title: String {
        switch self {
        case .rgbCloud: "Nube RGB"
        case .calibrationOverlay: "Calibración"
        }
    }
}

enum CameraLensPreference: String, CaseIterable, Identifiable {
    case ultraWide
    case wide

    var id: String { rawValue }

    var title: String {
        switch self {
        case .ultraWide: "0.5× Ultra gran angular"
        case .wide: "1× Gran angular"
        }
    }

    var captureDeviceType: AVCaptureDevice.DeviceType {
        switch self {
        case .ultraWide: .builtInUltraWideCamera
        case .wide: .builtInWideAngleCamera
        }
    }
}

final class CameraSessionController: NSObject, ObservableObject, ARSessionDelegate {
    @Published private(set) var isRunning = false
    @Published private(set) var statusText = "Cámara detenida"
    // Start with the actual camera image visible. A provisional rigid transform
    // can project every LiDAR point outside the image, making rgbCloud black.
    @Published private(set) var displayMode: CameraDisplayMode = .calibrationOverlay
    @Published private(set) var cameraPreviewRequested = false
    @Published private(set) var lensPreference: CameraLensPreference = .wide
    @Published private(set) var activeLensName = "1×"

    let session = ARSession()
    let captureRecorder = CameraCaptureRecorder()
    let fusionEncoder = CameraFusionEncoder()
    @Published var fusionEnabled = true { didSet { updateFusionState() } }
    private var activeLensKey = "wide"
    var activeLensIdentifier: String { activeLensKey }

    private func updateFusionState() {
        fusionEncoder.setActive(enabled: fusionEnabled && shouldRenderCamera, lens: activeLensKey)
    }

    private let stateLock = NSLock()
    private var requestedRunning = false
    private var ultraWideFallback = false

    override init() {
        super.init()
        session.delegate = self
    }

    var shouldRenderCamera: Bool {
        stateLock.lock()
        defer { stateLock.unlock() }
        return requestedRunning
    }

    func start() {
        captureRecorder.stop()
        guard ARWorldTrackingConfiguration.isSupported else {
            setRequestedRunning(false)
            publish(running: false, status: "ARKit no disponible en este dispositivo")
            return
        }

        let configuration = ARWorldTrackingConfiguration()
        configuration.worldAlignment = .gravity
        configuration.planeDetection = []
        configuration.environmentTexturing = .none

        let formats = ARWorldTrackingConfiguration.supportedVideoFormats
        let requestedFormat = formats.first {
            $0.captureDevicePosition == .back
                && $0.captureDeviceType == lensPreference.captureDeviceType
        }
        let wideFallback = formats.first {
            $0.captureDevicePosition == .back
                && $0.captureDeviceType == AVCaptureDevice.DeviceType.builtInWideAngleCamera
        }
        if let format = requestedFormat ?? wideFallback {
            configuration.videoFormat = format
        }

        let selectedType = configuration.videoFormat.captureDeviceType
        activeLensKey = selectedType == .builtInUltraWideCamera ? "ultraWide" : "wide"
        activeLensName = selectedType == .builtInUltraWideCamera ? "0.5×" : "1×"
        ultraWideFallback = lensPreference == .ultraWide
            && selectedType != .builtInUltraWideCamera

        setRequestedRunning(true)
        publish(running: true, status: "Iniciando cámara \(activeLensName)…")
        session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
    }

    func pause() {
        captureRecorder.stop()
        setRequestedRunning(false)
        session.pause()
        publish(running: false, status: "Cámara detenida")
    }

    func toggleDisplayMode() {
        displayMode = displayMode == .rgbCloud ? .calibrationOverlay : .rgbCloud
    }

    /// Changes presentation only; never restarts ARKit or interrupts recording.
    func showCameraView() {
        cameraPreviewRequested = true
        displayMode = .calibrationOverlay
    }

    func showMapView() {
        cameraPreviewRequested = false
    }

    func selectLens(_ lens: CameraLensPreference) {
        guard lens != lensPreference else { return }
        lensPreference = lens
        if shouldRenderCamera {
            start()
        }
    }

    func session(_ session: ARSession, cameraDidChangeTrackingState camera: ARCamera) {
        let status: String
        switch camera.trackingState {
        case .normal:
            status = ultraWideFallback
                ? "Cámara RGB 1× activa · 0.5× no disponible"
                : "Cámara RGB \(activeLensName) activa"
        case .notAvailable:
            status = "Tracking ARKit no disponible"
        case let .limited(reason):
            switch reason {
            case .initializing:
                status = "Inicializando ARKit…"
            case .excessiveMotion:
                status = "Mueve el equipo más despacio"
            case .insufficientFeatures:
                status = "La cámara necesita más textura o luz"
            case .relocalizing:
                status = "Relocalizando ARKit…"
            @unknown default:
                status = "Tracking ARKit limitado"
            }
        }
        publish(running: shouldRenderCamera, status: status)
    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        DispatchQueue.main.async { [weak self] in self?.captureRecorder.stop() }
        setRequestedRunning(false)
        publish(running: false, status: "Error de cámara: \(error.localizedDescription)")
    }

    func sessionWasInterrupted(_ session: ARSession) {
        DispatchQueue.main.async { [weak self] in self?.captureRecorder.stop() }
        fusionEncoder.setActive(enabled: false, lens: activeLensKey)
        publish(running: shouldRenderCamera, status: "Cámara interrumpida")
    }

    func sessionInterruptionEnded(_ session: ARSession) {
        DispatchQueue.main.async { [weak self] in
            guard let self, self.shouldRenderCamera else { return }
            self.start()
        }
    }

    private func setRequestedRunning(_ value: Bool) {
        stateLock.lock()
        requestedRunning = value
        stateLock.unlock()
        updateFusionState()
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard shouldRenderCamera else { return }
        captureRecorder.submit(frame)
        fusionEncoder.submit(frame)
    }

    private func publish(running: Bool, status: String) {
        DispatchQueue.main.async { [weak self] in
            self?.isRunning = running
            self?.statusText = status
        }
    }
}
