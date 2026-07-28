import MetalKit
import SwiftUI

struct MetalPointCloudView: UIViewRepresentable {
    let model: PointCloudModel
    let camera: CameraSessionController
    let calibration: CalibrationStore

    func makeCoordinator() -> Coordinator {
        Coordinator(model: model, camera: camera, calibration: calibration)
    }

    func makeUIView(context: Context) -> MTKView {
        let view = MTKView(frame: .zero, device: MTLCreateSystemDefaultDevice())
        view.colorPixelFormat = .bgra8Unorm
        view.depthStencilPixelFormat = .depth32Float
        view.clearColor = MTLClearColorMake(0.005, 0.008, 0.015, 1)
        view.preferredFramesPerSecond = 60
        view.enableSetNeedsDisplay = false
        view.isPaused = false

        context.coordinator.attach(to: view)
        let pan = UIPanGestureRecognizer(target: context.coordinator, action: #selector(Coordinator.pan(_:)))
        let pinch = UIPinchGestureRecognizer(target: context.coordinator, action: #selector(Coordinator.pinch(_:)))
        view.addGestureRecognizer(pan)
        view.addGestureRecognizer(pinch)
        return view
    }

    func updateUIView(_ uiView: MTKView, context: Context) {}

    final class Coordinator: NSObject {
        private let model: PointCloudModel
        private let camera: CameraSessionController
        private let calibration: CalibrationStore
        private var renderer: PointCloudRenderer?

        init(
            model: PointCloudModel,
            camera: CameraSessionController,
            calibration: CalibrationStore
        ) {
            self.model = model
            self.camera = camera
            self.calibration = calibration
        }

        func attach(to view: MTKView) {
            renderer = PointCloudRenderer(
                view: view,
                model: model,
                camera: camera,
                calibration: calibration
            )
            view.delegate = renderer
        }

        @objc func pan(_ recognizer: UIPanGestureRecognizer) {
            let delta = recognizer.translation(in: recognizer.view)
            recognizer.setTranslation(.zero, in: recognizer.view)
            renderer?.orbit(deltaX: Float(delta.x), deltaY: Float(delta.y))
        }

        @objc func pinch(_ recognizer: UIPinchGestureRecognizer) {
            renderer?.zoom(scale: Float(recognizer.scale))
            recognizer.scale = 1
        }
    }
}
