import ARKit
import CoreImage
import Foundation

/// Native JPEG keyframes (at most 5 fps) and an independent, bounded full-rate
/// ARKit pose log. Only the JPEG worker retains an ARFrame, at most one at a time.
final class CameraCaptureRecorder: ObservableObject {
    @Published private(set) var isRecording = false
    @Published private(set) var isFinishing = false
    @Published private(set) var statusText = "Captura de cámara detenida"

    private let lock = NSLock()
    private let writer = DispatchQueue(label: "atlas.camera.capture", qos: .utility)
    private let poseWriter = DispatchQueue(label: "atlas.camera.poses", qos: .utility)
    private let context = CIContext()
    private static let maximumPendingPoses = 512
    private var poseFile: FileHandle?
    private var poseAccepted = 0
    private var poseRecorded = 0
    private var pendingPoses = 0
    private var lastPoseTimestamp: UInt64?
    private var finishing = false
    private var failureMessage: String?
    private var directory: URL?
    private var accepting = false
    private var inFlight = false
    private var lastTimestamp = -Double.infinity
    private var recorded = 0
    private var skippedBusy = 0
    private var failed = false

    private struct PoseSample: Encodable {
        let schema_version = 1
        let sequence: Int
        let timestamp_ns: UInt64
        let tracking_normal: Bool
        let world_from_camera_row_major: [Float]
    }

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
                "max_keyframe_fps": 5, "max_in_flight_images": 1,
                "pose_file": "poses.jsonl", "pose_schema_version": 1,
                "pose_sampling": "every delivered ARFrame, independently of JPEG",
                "pose_timestamp": "ARFrame.timestamp * 1e9; capture time, not arrival",
                "max_pending_poses": Self.maximumPendingPoses
            ]
            try JSONSerialization.data(withJSONObject: manifest, options: [.sortedKeys, .prettyPrinted])
                .write(to: target.appendingPathComponent("session.json"), options: .atomic)
            let poseURL = target.appendingPathComponent("poses.jsonl")
            guard FileManager.default.createFile(atPath: poseURL.path, contents: nil) else {
                throw NSError(domain: "ATLASCapture", code: 2,
                              userInfo: [NSLocalizedDescriptionKey: "No se pudo crear el registro de poses"])
            }
            let handle = try FileHandle(forWritingTo: poseURL)
            lock.lock()
            directory = target; accepting = true; inFlight = false
            recorded = 0; skippedBusy = 0; failed = false; lastTimestamp = -Double.infinity
            poseFile = handle; poseAccepted = 0; poseRecorded = 0; pendingPoses = 0
            lastPoseTimestamp = nil; finishing = false; failureMessage = nil
            lock.unlock()
            isRecording = true
            statusText = "Grabando cámara · \(target.lastPathComponent.prefix(8))"
        } catch {
            statusText = "No se pudo grabar: \(error.localizedDescription)"
        }
    }

    func submit(_ frame: ARFrame) {
        lock.lock()
        guard accepting, let target = directory, let poseFile else { lock.unlock(); return }
        guard frame.timestamp.isFinite, frame.timestamp > 0,
              frame.timestamp < Double(UInt64.max) / 1e9 else {
            lock.unlock(); fail("Timestamp de cámara inválido"); return
        }
        let timestamp = UInt64(frame.timestamp * 1e9)
        let pose = frame.camera.transform
        let matrix = (0..<4).flatMap { r in (0..<4).map { c in pose[c][r] } }
        guard matrix.allSatisfy({ $0.isFinite }),
              lastPoseTimestamp.map({ timestamp > $0 }) ?? true else {
            lock.unlock(); fail("La secuencia de poses cambió de reloj o contiene datos inválidos"); return
        }
        guard pendingPoses < Self.maximumPendingPoses else {
            lock.unlock(); fail("El registro de poses no pudo seguir el ritmo; captura incompleta"); return
        }
        let trackingNormal: Bool
        if case .normal = frame.camera.trackingState { trackingNormal = true }
        else { trackingNormal = false }
        let sample = PoseSample(sequence: poseAccepted, timestamp_ns: timestamp,
                                tracking_normal: trackingNormal, world_from_camera_row_major: matrix)
        poseAccepted += 1; pendingPoses += 1; lastPoseTimestamp = timestamp
        // Copy small values only: a slow JPEG must neither retain extra ARFrames
        // nor reduce the temporal calibration signal to the 5 fps image rate.
        poseWriter.async { [self] in
            do {
                var data = try JSONEncoder().encode(sample)
                data.append(0x0A)
                try poseFile.write(contentsOf: data)
                lock.lock(); poseRecorded += 1; pendingPoses -= 1; lock.unlock()
            } catch {
                lock.lock(); pendingPoses -= 1; lock.unlock()
                fail(error.localizedDescription)
            }
        }
        guard frame.timestamp - lastTimestamp >= 0.2 else { lock.unlock(); return }
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
                lock.lock(); inFlight = false; lock.unlock()
                fail(error.localizedDescription)
            }
        }
        lock.unlock() // enqueue under the lock so stop() cannot overtake this frame
    }

    func stop() {
        lock.lock()
        guard directory != nil, !finishing else { lock.unlock(); return }
        accepting = false; finishing = true
        let target = directory
        let handle = poseFile
        lock.unlock()
        isRecording = false; isFinishing = true; statusText = "Finalizando captura…"
        // Both barriers are enqueued after all accepted samples. The final
        // marker is created only after images AND poses are flushed and closed.
        let completion = DispatchGroup()
        completion.enter()
        poseWriter.async { [self] in
            defer { completion.leave() }
            do {
                try handle?.synchronize()
                try handle?.close()
            } catch {
                try? handle?.close()
                fail(error.localizedDescription)
            }
        }
        completion.enter()
        writer.async { completion.leave() }
        completion.notify(queue: writer) { [self] in
            lock.lock()
            let count = recorded, skipped = skippedBusy, poses = poseRecorded
            let incomplete = failed || poses != poseAccepted || pendingPoses != 0
            let errorText = failureMessage
            directory = nil; poseFile = nil
            lock.unlock()
            do {
                if !incomplete, let target {
                    let summary: [String: Any] = ["complete": true, "keyframes": count,
                                                  "skipped_writer_busy": skipped,
                                                  "pose_samples": poses, "pose_samples_dropped": 0,
                                                  "pose_log_complete": true]
                    try JSONSerialization.data(withJSONObject: summary, options: [.sortedKeys])
                        .write(to: target.appendingPathComponent("complete.json"), options: .atomic)
                }
                DispatchQueue.main.async { [self] in
                    isFinishing = false
                    statusText = incomplete ? "Captura incompleta: \(errorText ?? "faltan poses")"
                        : "\(count) imágenes · \(poses) poses · Archivos → ATLAS Viewer"
                }
            } catch {
                DispatchQueue.main.async { [self] in
                    isFinishing = false; statusText = "Captura incompleta: \(error.localizedDescription)"
                }
            }
        }
    }

    private func fail(_ message: String) {
        lock.lock()
        failed = true; accepting = false
        if failureMessage == nil { failureMessage = message }
        lock.unlock()
        // Finalize even when a worker failed after accepting was cleared.
        DispatchQueue.main.async { [weak self] in self?.stop() }
    }
}
