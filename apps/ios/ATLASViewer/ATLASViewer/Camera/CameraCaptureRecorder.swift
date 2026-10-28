import ARKit
import CoreImage
import Foundation

/// Native, unrotated JPEG keyframes plus calibration/clock metadata. Deliberately
/// bounded to one image in flight and at most 5 fps; it is not a full video log.
final class CameraCaptureRecorder: ObservableObject {
    @Published private(set) var isRecording = false
    @Published private(set) var isFinishing = false
    @Published private(set) var statusText = "Captura de cámara detenida"

    private let lock = NSLock()
    private let writer = DispatchQueue(label: "atlas.camera.capture", qos: .utility)
    private let context = CIContext()
    private var directory: URL?
    private var accepting = false
    private var inFlight = false
    private var lastTimestamp = -Double.infinity
    private var recorded = 0
    private var skippedBusy = 0
    private var failed = false

    func start(profile: DeviceCalibrationProfile, lens: String) {
        guard !isRecording && !isFinishing else { return }
        do {
            let root = try FileManager.default.url(for: .documentDirectory, in: .userDomainMask,
                                                   appropriateFor: nil, create: true)
                .appendingPathComponent("ATLASCaptures", isDirectory: true)
            let target = root.appendingPathComponent(UUID().uuidString, isDirectory: true)
            try FileManager.default.createDirectory(at: target, withIntermediateDirectories: true)
            let profileData = try JSONEncoder().encode(profile)
            try profileData.write(to: target.appendingPathComponent("calibration.json"), options: .atomic)
            let manifest: [String: Any] = [
                "schema_version": 1, "created_unix_s": Date().timeIntervalSince1970,
                "clock_domain": "ARFrame.timestamp: iPhone monotonic seconds; NOT synchronized to Mac",
                "image_coordinates": "native unrotated camera image",
                "pose_frame": "ARKit local world; not LiDAR/project frame",
                "lens": lens, "calibration_verified": profile.verified,
                "max_keyframe_fps": 5, "max_in_flight_images": 1
            ]
            try JSONSerialization.data(withJSONObject: manifest, options: [.sortedKeys, .prettyPrinted])
                .write(to: target.appendingPathComponent("session.json"), options: .atomic)
            lock.lock()
            directory = target; accepting = true; inFlight = false
            recorded = 0; skippedBusy = 0; failed = false; lastTimestamp = -Double.infinity
            lock.unlock()
            isRecording = true
            statusText = "Grabando cámara · \(target.lastPathComponent.prefix(8))"
        } catch {
            statusText = "No se pudo grabar: \(error.localizedDescription)"
        }
    }

    func submit(_ frame: ARFrame) {
        lock.lock()
        guard accepting, let target = directory, frame.timestamp.isFinite,
              frame.timestamp - lastTimestamp >= 0.2 else { lock.unlock(); return }
        guard !inFlight else { skippedBusy += 1; lock.unlock(); return }
        inFlight = true; lastTimestamp = frame.timestamp
        let sequence = recorded
        // Retain at most one ARFrame while encoding; never build a frame queue.
        writer.async { [self] in
            do {
                try autoreleasepool {
                    let image = CIImage(cvPixelBuffer: frame.capturedImage)
                    guard let colorSpace = CGColorSpace(name: CGColorSpace.sRGB),
                          let jpeg = context.jpegRepresentation(of: image, colorSpace: colorSpace,
                                                               options: [:]) else {
                        throw NSError(domain: "ATLASCapture", code: 1,
                                      userInfo: [NSLocalizedDescriptionKey: "No se pudo codificar JPEG"])
                    }
                    let name = String(format: "%08d", sequence)
                    try jpeg.write(to: target.appendingPathComponent(name + ".jpg"), options: .atomic)
                    let intrinsics = frame.camera.intrinsics
                    let pose = frame.camera.transform
                    let metadata: [String: Any] = [
                        "sequence": sequence, "timestamp_s": frame.timestamp,
                        "encoded_unix_s": Date().timeIntervalSince1970,
                        "image": name + ".jpg", "width": CVPixelBufferGetWidth(frame.capturedImage),
                        "height": CVPixelBufferGetHeight(frame.capturedImage),
                        "intrinsics_row_major": (0..<3).flatMap { r in (0..<3).map { c in intrinsics[c][r] } },
                        "world_from_camera_row_major": (0..<4).flatMap { r in (0..<4).map { c in pose[c][r] } },
                        "tracking_state": String(describing: frame.camera.trackingState)
                    ]
                    try JSONSerialization.data(withJSONObject: metadata, options: [.sortedKeys])
                        .write(to: target.appendingPathComponent(name + ".json"), options: .atomic)
                }
                lock.lock(); recorded += 1; inFlight = false; let count = recorded; lock.unlock()
                DispatchQueue.main.async { [self] in
                    if isRecording { statusText = "Cámara: \(count) imágenes guardadas" }
                }
            } catch {
                lock.lock(); failed = true; accepting = false; inFlight = false; lock.unlock()
                DispatchQueue.main.async { [self] in
                    isRecording = false; statusText = "Captura incompleta: \(error.localizedDescription)"
                }
            }
        }
        lock.unlock() // enqueue under the lock so stop() cannot overtake this frame
    }

    func stop() {
        lock.lock()
        guard accepting || inFlight else { lock.unlock(); return }
        accepting = false
        let target = directory
        lock.unlock()
        isRecording = false; isFinishing = true; statusText = "Finalizando captura…"
        writer.async { [self] in
            lock.lock(); let count = recorded; let skipped = skippedBusy; let incomplete = failed; lock.unlock()
            do {
                if !incomplete, let target {
                    let summary: [String: Any] = ["complete": true, "keyframes": count,
                                                  "skipped_writer_busy": skipped]
                    try JSONSerialization.data(withJSONObject: summary, options: [.sortedKeys])
                        .write(to: target.appendingPathComponent("complete.json"), options: .atomic)
                }
                DispatchQueue.main.async { [self] in
                    isFinishing = false
                    statusText = incomplete ? "Captura incompleta" : "\(count) imágenes · Archivos → ATLAS Viewer"
                }
            } catch {
                DispatchQueue.main.async { [self] in
                    isFinishing = false; statusText = "Captura incompleta: \(error.localizedDescription)"
                }
            }
        }
    }
}
