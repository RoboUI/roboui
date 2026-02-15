// RoboUI — RoboMessage
// Protocol for type-safe ROS2 message parsing.

import Foundation

/// A ROS2 message that can be parsed from a rosbridge JSON dictionary.
///
/// Conform your custom types to this protocol for use with `TopicSubscriber<T>`.
///
/// ```swift
/// struct BatteryState: RoboMessage {
///     let voltage: Float
///     let percentage: Float
///
///     static func from(dict: [String: Any]) -> BatteryState? {
///         guard let v = dict["voltage"] as? Double,
///               let p = dict["percentage"] as? Double else { return nil }
///         return BatteryState(voltage: Float(v), percentage: Float(p))
///     }
/// }
/// ```
public protocol RoboMessage: Sendable {
    /// Parse a message from a rosbridge JSON dictionary.
    /// Returns `nil` if required fields are missing.
    static func from(dict: [String: Any]) -> Self?
}

// MARK: - Built-in conformances

extension LaserScan: RoboMessage {}
extension CompressedImage: RoboMessage {}

extension Twist: RoboMessage {
    public static func from(dict: [String: Any]) -> Twist? {
        let linear: Vector3
        let angular: Vector3
        
        if let lin = dict["linear"] as? [String: Any] {
            linear = Vector3(
                x: (lin["x"] as? Double) ?? 0,
                y: (lin["y"] as? Double) ?? 0,
                z: (lin["z"] as? Double) ?? 0
            )
        } else {
            linear = Vector3()
        }
        
        if let ang = dict["angular"] as? [String: Any] {
            angular = Vector3(
                x: (ang["x"] as? Double) ?? 0,
                y: (ang["y"] as? Double) ?? 0,
                z: (ang["z"] as? Double) ?? 0
            )
        } else {
            angular = Vector3()
        }
        
        return Twist(linear: linear, angular: angular)
    }
}
