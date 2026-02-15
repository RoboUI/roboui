// RoboUI — OccupancyGrid
// nav_msgs/msg/OccupancyGrid

import Foundation

/// ROS2 `nav_msgs/msg/OccupancyGrid` message.
///
/// Represents a 2D grid map where each cell holds an occupancy probability:
/// - `-1`: unknown
/// - `0`: free
/// - `100`: occupied
public struct OccupancyGrid: RoboMessage, Sendable {
    /// Map resolution in meters/cell.
    public let resolution: Float
    
    /// Grid width in cells.
    public let width: Int
    
    /// Grid height in cells.
    public let height: Int
    
    /// Origin position (x, y) in world frame (meters).
    public let originX: Double
    public let originY: Double
    
    /// Origin orientation (yaw) in radians.
    public let originYaw: Double
    
    /// Frame ID from header.
    public let frameId: String
    
    /// Raw occupancy data: row-major, values in [-1, 100].
    /// Index = y * width + x.
    public let data: [Int8]
    
    public init(
        resolution: Float,
        width: Int,
        height: Int,
        originX: Double = 0,
        originY: Double = 0,
        originYaw: Double = 0,
        frameId: String = "map",
        data: [Int8]
    ) {
        self.resolution = resolution
        self.width = width
        self.height = height
        self.originX = originX
        self.originY = originY
        self.originYaw = originYaw
        self.frameId = frameId
        self.data = data
    }
    
    /// Parse from rosbridge JSON dictionary.
    public static func from(dict: [String: Any]) -> OccupancyGrid? {
        guard let info = dict["info"] as? [String: Any],
              let w = info["width"] as? Int,
              let h = info["height"] as? Int,
              let rawData = dict["data"] as? [Any] else {
            return nil
        }
        
        let resolution = Float((info["resolution"] as? Double) ?? 0.05)
        
        var originX: Double = 0
        var originY: Double = 0
        var originYaw: Double = 0
        
        if let origin = info["origin"] as? [String: Any] {
            if let pos = origin["position"] as? [String: Any] {
                originX = (pos["x"] as? Double) ?? 0
                originY = (pos["y"] as? Double) ?? 0
            }
            if let orient = origin["orientation"] as? [String: Any] {
                // Extract yaw from quaternion (z, w)
                let qz = (orient["z"] as? Double) ?? 0
                let qw = (orient["w"] as? Double) ?? 1
                originYaw = 2 * atan2(qz, qw)
            }
        }
        
        var frameId = ""
        if let header = dict["header"] as? [String: Any] {
            frameId = (header["frame_id"] as? String) ?? ""
        }
        
        let data: [Int8] = rawData.map { val in
            if let i = val as? Int { return Int8(clamping: i) }
            if let d = val as? Double { return Int8(clamping: Int(d)) }
            return -1
        }
        
        guard data.count == w * h else { return nil }
        
        return OccupancyGrid(
            resolution: resolution,
            width: w,
            height: h,
            originX: originX,
            originY: originY,
            originYaw: originYaw,
            frameId: frameId,
            data: data
        )
    }
    
    /// Get occupancy at grid coordinates. Returns nil if out of bounds.
    public func occupancy(atX x: Int, y: Int) -> Int8? {
        guard x >= 0, x < width, y >= 0, y < height else { return nil }
        return data[y * width + x]
    }
}
