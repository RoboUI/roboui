// RoboUI — TopicSubscriber<T>
// Generic type-safe ROS2 topic subscriber.

import Foundation
import Combine

/// A type-safe subscriber for a ROS2 topic.
///
/// Usage with built-in types:
/// ```swift
/// let lidar = TopicSubscriber<LaserScan>(
///     connection: robot,
///     topic: "/scan"
/// )
///
/// lidar.$latest.sink { scan in
///     print("Got \(scan?.points.count ?? 0) points")
/// }
/// ```
///
/// Custom Codable message:
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
///
/// let battery = TopicSubscriber<BatteryState>(
///     connection: robot,
///     topic: "/battery_state",
///     type: "sensor_msgs/msg/BatteryState"
/// )
/// ```
@MainActor
public final class TopicSubscriber<T: RoboMessage>: ObservableObject {
    
    /// Latest received message (nil until first message arrives).
    @Published public private(set) var latest: T?
    
    /// Number of messages received.
    @Published public private(set) var messageCount: Int = 0
    
    /// Whether currently subscribed.
    @Published public private(set) var isSubscribed: Bool = false
    
    private let connection: RosbridgeConnection
    private let topic: String
    private let type: String?
    private let throttleRate: Int?
    private var subscriptionId: String?
    
    /// Create a subscriber.
    /// - Parameters:
    ///   - connection: The rosbridge connection.
    ///   - topic: ROS2 topic name (e.g. "/scan").
    ///   - type: Optional ROS2 message type (e.g. "sensor_msgs/msg/LaserScan").
    ///     Rosbridge can infer the type, but specifying it avoids a round-trip.
    ///   - throttleRate: Max rate in ms between messages (rosbridge-side throttle).
    ///   - autoSubscribe: Subscribe immediately on init. Default: `true`.
    public init(
        connection: RosbridgeConnection,
        topic: String,
        type: String? = nil,
        throttleRate: Int? = nil,
        autoSubscribe: Bool = true
    ) {
        self.connection = connection
        self.topic = topic
        self.type = type
        self.throttleRate = throttleRate
        
        if autoSubscribe {
            subscribe()
        }
    }
    
    /// Start receiving messages.
    public func subscribe() {
        guard !isSubscribed else { return }
        
        subscriptionId = connection.subscribe(
            topic: topic,
            type: type,
            throttleRate: throttleRate
        ) { [weak self] dict in
            Task { @MainActor in
                guard let self else { return }
                if let parsed = T.from(dict: dict) {
                    self.latest = parsed
                    self.messageCount += 1
                }
            }
        }
        isSubscribed = true
    }
    
    /// Stop receiving messages.
    public func unsubscribe() {
        guard isSubscribed else { return }
        connection.unsubscribe(topic: topic, id: subscriptionId)
        subscriptionId = nil
        isSubscribed = false
    }
    
    deinit {
        // Can't call unsubscribe() from deinit (MainActor isolation),
        // but the connection will clean up on disconnect anyway.
    }
}
