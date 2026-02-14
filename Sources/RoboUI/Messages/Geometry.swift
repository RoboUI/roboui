import Foundation

/// geometry_msgs/msg/Vector3
public struct Vector3: Sendable {
    public var x: Double
    public var y: Double
    public var z: Double
    
    public init(x: Double = 0, y: Double = 0, z: Double = 0) {
        self.x = x
        self.y = y
        self.z = z
    }
    
    public var dict: [String: Any] {
        ["x": x, "y": y, "z": z]
    }
}

/// geometry_msgs/msg/Twist
public struct Twist: Sendable {
    public var linear: Vector3
    public var angular: Vector3
    
    public init(linear: Vector3 = .init(), angular: Vector3 = .init()) {
        self.linear = linear
        self.angular = angular
    }
    
    public var dict: [String: Any] {
        ["linear": linear.dict, "angular": angular.dict]
    }
}
