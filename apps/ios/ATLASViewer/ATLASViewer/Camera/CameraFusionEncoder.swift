import ARKit
import CoreImage
import Foundation

/// Native sensor orientation, top row first. No portrait/display transforms.
/// One frame in flight, including network delivery; slow Wi-Fi drops frames.
final class CameraFusionEncoder {
    typealias Sender = (Data, @escaping () -> Void) -> Void
    private let queue = DispatchQueue(label: "atlas.camera.fusion", qos: .utility)
    private let context = CIContext(options: [.cacheIntermediates: false])
    private let lock = NSLock()
    private var inFlight = false
    private var lastTimestamp = -Double.infinity
    private var profile: DeviceCalibrationProfile?
    private var actualLens = "wide"
    private var enabled = true
    private var sender: Sender?

    func configure(profile: DeviceCalibrationProfile, sender: @escaping Sender) {
        lock.lock(); defer { lock.unlock() }
        self.profile = profile
        self.sender = sender
    }

    func setActive(enabled: Bool, lens: String) {
        lock.lock(); defer { lock.unlock() }
        self.enabled = enabled
        actualLens = lens
    }

    func submit(_ frame: ARFrame) {
        lock.lock()
        guard enabled, !inFlight, frame.timestamp - lastTimestamp >= 0.2,
              let profile, let sender else { lock.unlock(); return }
        let lens = actualLens
        inFlight = true
        lastTimestamp = frame.timestamp
        lock.unlock()
        queue.async { [weak self] in
            guard let self else { return }
            guard let data = self.encode(frame, profile: profile, lens: lens) else {
                self.complete(); return
            }
            sender(data) { [weak self] in self?.complete() }
        }
    }

    private func complete() {
        lock.lock(); inFlight = false; lock.unlock()
    }

    private func encode(_ frame: ARFrame, profile: DeviceCalibrationProfile, lens: String) -> Data? {
        let source = frame.capturedImage
        let sourceWidth = CVPixelBufferGetWidth(source), sourceHeight = CVPixelBufferGetHeight(source)
        guard sourceWidth > 0, sourceHeight > 0, profile.lidarToCamera.isValid,
              frame.camera.imageResolution.width == CGFloat(sourceWidth),
              frame.camera.imageResolution.height == CGFloat(sourceHeight),
              frame.timestamp.isFinite, frame.timestamp > 0,
              frame.timestamp < Double(UInt64.max) / 1e9 else { return nil }
        let width = min(320, sourceWidth)
        let height = max(1, Int(Double(sourceHeight) * Double(width) / Double(sourceWidth)))
        let sx = Double(width) / Double(sourceWidth), sy = Double(height) / Double(sourceHeight)
        var target: CVPixelBuffer?
        guard CVPixelBufferCreate(kCFAllocatorDefault, width, height, kCVPixelFormatType_32BGRA,
                                  [kCVPixelBufferIOSurfacePropertiesKey: [:]] as CFDictionary,
                                  &target) == kCVReturnSuccess, let target else { return nil }
        let image = CIImage(cvPixelBuffer: source).transformed(by: CGAffineTransform(scaleX: sx, y: sy))
        context.render(image, to: target, bounds: CGRect(x: 0, y: 0, width: width, height: height),
                       colorSpace: CGColorSpace(name: CGColorSpace.sRGB)!)
        guard CVPixelBufferLockBaseAddress(target, .readOnly) == kCVReturnSuccess else { return nil }
        defer { CVPixelBufferUnlockBaseAddress(target, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddress(target) else { return nil }
        let rowBytes = CVPixelBufferGetBytesPerRow(target)
        var data = Data(capacity: 160 + width * height * 3)
        func append<T: FixedWidthInteger>(_ value: T) {
            var little = value.littleEndian
            withUnsafeBytes(of: &little) { data.append(contentsOf: $0) }
        }
        append(UInt32(0x52435441)); append(UInt32(1)); append(UInt32(148 + width * height * 3))
        append(UInt32(width)); append(UInt32(height))
        // Older profiles lack a lens declaration: they remain preview-only.
        let verified = profile.verified && profile.cameraLens == lens
        var flags: UInt32 = verified ? 1 : 0
        if case .normal = frame.camera.trackingState { flags |= 2 }
        append(flags); append(UInt64(frame.timestamp * 1e9))
        let k = frame.camera.intrinsics
        let intrinsics = [Double(k[0, 0]) * sx, Double(k[1, 1]) * sy,
                          Double(k[2, 0]) * sx, Double(k[2, 1]) * sy]
        for value in intrinsics { append(value.bitPattern) }
        for value in profile.lidarToCamera.rotationMatrixRowMajor { append(Double(value).bitPattern) }
        for value in profile.lidarToCamera.translationM { append(Double(value).bitPattern) }
        for y in 0..<height {
            let row = base.advanced(by: y * rowBytes).assumingMemoryBound(to: UInt8.self)
            for x in 0..<width {
                data.append(row[x * 4 + 2]); data.append(row[x * 4 + 1]); data.append(row[x * 4])
            }
        }
        return data
    }
}
