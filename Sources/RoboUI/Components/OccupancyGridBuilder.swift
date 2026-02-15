// RoboUI — OccupancyGridBuilder
// Client-side occupancy grid from LiDAR scans.

import Foundation

/// Builds an `OccupancyGrid` incrementally from LiDAR scans.
///
/// Simple hit-count approach: each scan endpoint increments a cell counter.
/// Cells with enough hits become walls. No ray-casting, no free-space clearing,
/// so walls persist reliably even during rotation.
///
/// Usage:
/// ```swift
/// let builder = OccupancyGridBuilder(resolution: 0.05, size: 20)
/// builder.integrate(scan: scan, robotX: x, robotY: y, robotYaw: yaw)
/// let grid = builder.buildGrid()
/// ```
public final class OccupancyGridBuilder: @unchecked Sendable {
    
    /// Grid resolution in meters per cell.
    public let resolution: Float
    
    /// Grid dimensions in cells.
    public let width: Int
    public let height: Int
    
    /// World-space origin (bottom-left corner of grid).
    public let originX: Double
    public let originY: Double
    
    /// Hit counts per cell.
    private var hits: [UInt16]
    
    /// Minimum hits to consider a cell as wall.
    private let hitThreshold: UInt16 = 3
    
    /// Create an occupancy grid builder.
    /// - Parameters:
    ///   - resolution: Meters per cell (default 0.05 = 5cm).
    ///   - size: World size in meters (grid will be size×size). Default 20m.
    ///   - centerX: World X center. Default 0.
    ///   - centerY: World Y center. Default 0.
    public init(resolution: Float = 0.05, size: Float = 20, centerX: Double = 0, centerY: Double = 0) {
        self.resolution = resolution
        let cells = Int(size / resolution)
        self.width = cells
        self.height = cells
        self.originX = centerX - Double(size) / 2
        self.originY = centerY - Double(size) / 2
        self.hits = [UInt16](repeating: 0, count: cells * cells)
    }
    
    /// Integrate a LiDAR scan into the grid.
    public func integrate(scan: LaserScan, robotX: Double, robotY: Double, robotYaw: Double) {
        let res = Double(resolution)
        
        for (i, range) in scan.ranges.enumerated() {
            let r = Double(range)
            
            // Skip invalid / max ranges (max range = no wall detected)
            guard r >= Double(scan.rangeMin),
                  r < Double(scan.rangeMax) - 0.1 else { continue }
            
            // Beam angle in world frame
            let angle = robotYaw + Double(scan.angleMin) + Double(i) * Double(scan.angleIncrement)
            
            // Hit point in world frame
            let worldX = robotX + r * cos(angle)
            let worldY = robotY + r * sin(angle)
            
            // World → grid cell
            let gx = Int((worldX - originX) / res)
            let gy = Int((worldY - originY) / res)
            
            guard gx >= 0, gx < width, gy >= 0, gy < height else { continue }
            
            let idx = gy * width + gx
            if hits[idx] < UInt16.max {
                hits[idx] += 1
            }
        }
    }
    
    /// Build an OccupancyGrid from current state.
    public func buildGrid() -> OccupancyGrid {
        let data: [Int8] = hits.map { count in
            if count >= hitThreshold { return 100 }  // wall
            return -1                                  // unknown
        }
        
        return OccupancyGrid(
            resolution: resolution,
            width: width,
            height: height,
            originX: originX,
            originY: originY,
            originYaw: 0,
            frameId: "map",
            data: data
        )
    }
    
    /// Clear the grid.
    public func reset() {
        hits = [UInt16](repeating: 0, count: width * height)
    }
}
