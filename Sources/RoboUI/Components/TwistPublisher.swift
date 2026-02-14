import Foundation
import Combine

/// Publishes geometry_msgs/Twist to a topic (typically /cmd_vel).
/// Handles advertise/unadvertise lifecycle automatically.
@MainActor
public final class TwistPublisher: ObservableObject {
    
    private let connection: RosbridgeConnection
    private let topic: String
    private var isAdvertised = false
    
    @Published public var lastTwist: Twist = .init()
    
    public init(connection: RosbridgeConnection, topic: String = "/cmd_vel") {
        self.connection = connection
        self.topic = topic
    }
    
    /// Start advertising the topic.
    public func start() {
        guard !isAdvertised else { return }
        connection.advertise(topic: topic, type: "geometry_msgs/msg/Twist")
        isAdvertised = true
    }
    
    /// Stop advertising.
    public func stop() {
        // Send zero velocity before stopping
        publish(.init())
        
        guard isAdvertised else { return }
        connection.unadvertise(topic: topic)
        isAdvertised = false
    }
    
    /// Publish a Twist message.
    public func publish(_ twist: Twist) {
        if !isAdvertised { start() }
        lastTwist = twist
        connection.publish(topic: topic, message: twist.dict)
    }
    
    /// Convenience: publish linear.x and angular.z (typical for differential drive).
    public func drive(linear: Double, angular: Double) {
        publish(Twist(
            linear: Vector3(x: linear),
            angular: Vector3(z: angular)
        ))
    }
    
    /// Emergency stop — send zero velocity.
    public func emergencyStop() {
        publish(.init())
    }
}
