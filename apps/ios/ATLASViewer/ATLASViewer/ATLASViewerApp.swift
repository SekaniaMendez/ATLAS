import SwiftUI

@main
struct ATLASViewerApp: App {
    @StateObject private var pointCloud: PointCloudModel
    @StateObject private var connection: AtlasConnection
    @StateObject private var calibration: CalibrationStore
    @StateObject private var camera: CameraSessionController
    @StateObject private var capture: CameraCaptureRecorder

    init() {
        let model = PointCloudModel()
        model.loadDemoCloud()
        _pointCloud = StateObject(wrappedValue: model)
        _connection = StateObject(wrappedValue: AtlasConnection(pointCloud: model))
        _calibration = StateObject(wrappedValue: CalibrationStore())
        let cameraSession = CameraSessionController()
        _camera = StateObject(wrappedValue: cameraSession)
        _capture = StateObject(wrappedValue: cameraSession.captureRecorder)
    }

    var body: some Scene {
        WindowGroup {
            ContentView(
                pointCloud: pointCloud,
                connection: connection,
                calibration: calibration,
                camera: camera,
                capture: capture
            )
        }
    }
}
