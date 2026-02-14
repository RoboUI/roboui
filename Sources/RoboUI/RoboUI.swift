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

// Components
// - TwistPublisher: differential drive (/cmd_vel)
// - MecanumPublisher: omnidirectional drive (/cmd_vel)

// Views
// - JoystickView: single virtual joystick
// - DualJoystickView: move + rotate (for mecanum)
// - LaserScanView: LiDAR point cloud visualization
// - CameraView: compressed image / video feed
// - CameraHUD: recording indicator + format overlay
// - EmergencyStopButton: one-tap safety stop with haptics
// - TelemetryView: battery, speed, custom sensors panel
