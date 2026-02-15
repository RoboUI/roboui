// RoboUI — Native Mobile SDK for ROS2 Robots
//
// https://roboui.dev
// https://github.com/RoboUI/roboui-ios

// Transport
@_exported import struct Foundation.URL
public typealias Connection = RosbridgeConnection

// Messages
// - Vector3, Twist (Geometry.swift)
// - LaserScan (LaserScan.swift)
// - CompressedImage (CompressedImage.swift)
// - OccupancyGrid (OccupancyGrid.swift)
// - RoboMessage protocol (RoboMessage.swift)

// Transport
// - RosbridgeConnection: WebSocket client with auto-reconnect
// - TopicSubscriber<T>: generic type-safe topic subscriber

// Components
// - TwistPublisher: differential drive (/cmd_vel)
// - MecanumPublisher: omnidirectional drive (/cmd_vel)

// Transform
// - TransformRegistry: client-side TF2 buffer with SLERP interpolation
// - StampedTransform: timestamped rigid-body transform
// - Quaternion4: quaternion math (SLERP, multiply, inverse, rotate)
// - FrameBuffer: per-frame-pair time-indexed transform storage

// Views
// - JoystickView: single virtual joystick
// - DualJoystickView: move + rotate (for mecanum)
// - LaserScanView: LiDAR point cloud visualization
// - CameraView: compressed image / video feed
// - CameraHUD: recording indicator + format overlay
// - MapView: OccupancyGrid 2D map visualization
// - EmergencyStopButton: one-tap safety stop with haptics
// - TelemetryView: battery, speed, custom sensors panel
// - ConnectionIndicator: state + latency quality display
