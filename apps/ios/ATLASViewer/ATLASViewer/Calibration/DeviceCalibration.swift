import Foundation

struct DeviceDimensions: Codable {
    let height: Float
    let width: Float
    let thickness: Float
}

struct LidarToCameraTransform: Codable {
    let translationM: [Float]
    let rotationMatrixRowMajor: [Float]

    var isValid: Bool {
        translationM.count == 3 && rotationMatrixRowMajor.count == 9
    }
}

struct DeviceCalibrationProfile: Codable, Identifiable {
    let id: String
    let displayName: String
    let deviceIdentifiers: [String]
    let dimensionsMm: DeviceDimensions
    let lidarToCamera: LidarToCameraTransform
    let verified: Bool
    let notes: String
}

private struct DeviceCalibrationCatalog: Codable {
    let schemaVersion: Int
    let profiles: [DeviceCalibrationProfile]
}

final class CalibrationStore: ObservableObject {
    @Published var selectedProfileID: String {
        didSet {
            guard profiles.contains(where: { $0.id == selectedProfileID }) else { return }
            UserDefaults.standard.set(selectedProfileID, forKey: Self.selectionKey)
        }
    }

    let profiles: [DeviceCalibrationProfile]
    let detectedDeviceIdentifier: String

    var selectedProfile: DeviceCalibrationProfile {
        profiles.first(where: { $0.id == selectedProfileID }) ?? profiles[0]
    }

    init(bundle: Bundle = .main) {
        let identifier = Self.hardwareIdentifier()
        let loadedProfiles = Self.loadProfiles(bundle: bundle)
        let availableProfiles = loadedProfiles.isEmpty ? [Self.fallbackProfile] : loadedProfiles

        let savedID = UserDefaults.standard.string(forKey: Self.selectionKey)
        let detectedID = availableProfiles.first {
            $0.deviceIdentifiers.contains(identifier)
        }?.id
        let initialID = savedID.flatMap { saved in
            availableProfiles.contains(where: { $0.id == saved }) ? saved : nil
        } ?? detectedID ?? availableProfiles[0].id

        detectedDeviceIdentifier = identifier
        profiles = availableProfiles
        selectedProfileID = initialID
    }

    private static let selectionKey = "atlas.cameraCalibration.profile"

    private static func loadProfiles(bundle: Bundle) -> [DeviceCalibrationProfile] {
        guard let url = bundle.url(
            forResource: "iphone-mid360-calibrations",
            withExtension: "json"
        ), let data = try? Data(contentsOf: url) else { return [] }

        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        guard let catalog = try? decoder.decode(DeviceCalibrationCatalog.self, from: data),
              catalog.schemaVersion == 1
        else { return [] }
        return catalog.profiles.filter { $0.lidarToCamera.isValid }
    }

    private static func hardwareIdentifier() -> String {
#if targetEnvironment(simulator)
        if let simulated = ProcessInfo.processInfo.environment["SIMULATOR_MODEL_IDENTIFIER"] {
            return simulated
        }
#endif
        var info = utsname()
        uname(&info)
        return withUnsafePointer(to: &info.machine) { pointer in
            pointer.withMemoryRebound(to: CChar.self, capacity: 1) {
                String(cString: $0)
            }
        }
    }

    private static let fallbackProfile = DeviceCalibrationProfile(
        id: "generic-mid360-upright-v0",
        displayName: "iPhone genérico",
        deviceIdentifiers: [],
        dimensionsMm: DeviceDimensions(height: 0, width: 0, thickness: 0),
        lidarToCamera: LidarToCameraTransform(
            translationM: [0.075, 0, -0.031],
            rotationMatrixRowMajor: [
                0, 0, -1,
                0, -1, 0,
                -1, 0, 0,
            ]
        ),
        verified: false,
        notes: "Perfil de emergencia; requiere calibración física."
    )
}
