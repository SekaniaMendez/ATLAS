import SwiftUI

@main
struct ATLASViewerApp: App {
    @StateObject private var pointCloud: PointCloudModel
    @StateObject private var connection: AtlasConnection
    @StateObject private var calibration: CalibrationStore
    @StateObject private var camera: CameraSessionController

    init() {
        let model = PointCloudModel()
        model.loadDemoCloud()
        _pointCloud = StateObject(wrappedValue: model)
        _connection = StateObject(wrappedValue: AtlasConnection(pointCloud: model))
        _calibration = StateObject(wrappedValue: CalibrationStore())
        _camera = StateObject(wrappedValue: CameraSessionController())
    }

    var body: some Scene {
        WindowGroup {
            ContentView(
                pointCloud: pointCloud,
                connection: connection,
                calibration: calibration,
                camera: camera
            )
        }
    }
}
