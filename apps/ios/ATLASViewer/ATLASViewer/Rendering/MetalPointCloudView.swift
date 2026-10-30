import MetalKit
import SwiftUI

struct MetalPointCloudView: UIViewRepresentable {
    let model: PointCloudModel
    let geometry: LiveGeometryModel
    let camera: CameraSessionController
    let calibration: CalibrationStore
    let showPoints: Bool
    let showSurfaces: Bool
    let showAxes: Bool
    let showCeiling: Bool
    let ceilingCutHeightM: Float?
    let topViewRequest: Int

    func makeCoordinator() -> Coordinator {
        Coordinator(model: model, geometry: geometry, camera: camera, calibration: calibration)
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
        updateRenderer(context.coordinator)
        let pan = UIPanGestureRecognizer(target: context.coordinator, action: #selector(Coordinator.pan(_:)))
        let pinch = UIPinchGestureRecognizer(target: context.coordinator, action: #selector(Coordinator.pinch(_:)))
        view.addGestureRecognizer(pan)
        view.addGestureRecognizer(pinch)
        return view
    }

    func updateUIView(_ uiView: MTKView, context: Context) {
        updateRenderer(context.coordinator)
    }

    private func updateRenderer(_ coordinator: Coordinator) {
        coordinator.setVisibility(points: showPoints, surfaces: showSurfaces, axes: showAxes,
                                  ceiling: showCeiling, ceilingCutHeightM: ceilingCutHeightM)
        coordinator.consumeTopViewRequest(topViewRequest)
    }

    final class Coordinator: NSObject {
        private let model: PointCloudModel
        private let geometry: LiveGeometryModel
        private let camera: CameraSessionController
        private let calibration: CalibrationStore
        private var renderer: PointCloudRenderer?
        private var lastTopViewRequest = 0

        init(
            model: PointCloudModel,
            geometry: LiveGeometryModel,
            camera: CameraSessionController,
            calibration: CalibrationStore
        ) {
            self.model = model
            self.geometry = geometry
            self.camera = camera
            self.calibration = calibration
        }

        func attach(to view: MTKView) {
            renderer = PointCloudRenderer(
                view: view,
                model: model,
                geometry: geometry,
                camera: camera,
                calibration: calibration
            )
            view.delegate = renderer
        }

        func setVisibility(points: Bool, surfaces: Bool, axes: Bool,
                           ceiling: Bool, ceilingCutHeightM: Float?) {
            renderer?.setVisibility(points: points, surfaces: surfaces, axes: axes,
                                    ceiling: ceiling, ceilingCutHeightM: ceilingCutHeightM)
        }

        func consumeTopViewRequest(_ value: Int) {
            guard value != lastTopViewRequest else { return }
            lastTopViewRequest = value
            renderer?.showTopView()
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
