// RoboUI — MapView
// Renders a nav_msgs/OccupancyGrid as a 2D map.

import SwiftUI

/// A SwiftUI view that renders a ROS2 OccupancyGrid as a 2D map.
///
/// Usage:
/// ```swift
/// let mapSub = TopicSubscriber<OccupancyGrid>(
///     connection: robot,
///     topic: "/map",
///     type: "nav_msgs/msg/OccupancyGrid"
/// )
///
/// MapView(grid: mapSub.latest)
/// ```
///
/// With robot position overlay:
/// ```swift
/// MapView(grid: mapSub.latest, robotX: odomX, robotY: odomY, robotYaw: yaw)
/// ```
public struct MapView: View {
    let grid: OccupancyGrid?
    let robotX: Double?
    let robotY: Double?
    let robotYaw: Double?
    let style: MapStyle
    
    /// Create a map view.
    /// - Parameters:
    ///   - grid: OccupancyGrid to render. `nil` shows placeholder.
    ///   - robotX: Robot X position in world frame (meters).
    ///   - robotY: Robot Y position in world frame (meters).
    ///   - robotYaw: Robot heading in radians.
    ///   - style: Visual style. Default: `.dark`.
    public init(
        grid: OccupancyGrid?,
        robotX: Double? = nil,
        robotY: Double? = nil,
        robotYaw: Double? = nil,
        style: MapStyle = .dark
    ) {
        self.grid = grid
        self.robotX = robotX
        self.robotY = robotY
        self.robotYaw = robotYaw
        self.style = style
    }
    
    public var body: some View {
        ZStack {
            style.background
            
            if let grid {
                Canvas { context, size in
                    drawGrid(grid, context: context, size: size)
                    drawRobot(grid, context: context, size: size)
                }
            } else {
                placeholder
            }
            
            // Frame info
            if let grid {
                VStack {
                    Spacer()
                    HStack {
                        Text("\(grid.width)×\(grid.height) • \(String(format: "%.2f", grid.resolution))m/px")
                            .font(.system(size: 9, design: .monospaced))
                            .foregroundStyle(.white.opacity(0.4))
                            .padding(4)
                        Spacer()
                    }
                }
            }
        }
        .clipShape(RoundedRectangle(cornerRadius: 12))
        .overlay(
            RoundedRectangle(cornerRadius: 12)
                .stroke(.white.opacity(0.08), lineWidth: 1)
        )
    }
    
    // MARK: - Grid Rendering
    
    private func drawGrid(_ grid: OccupancyGrid, context: GraphicsContext, size: CGSize) {
        let cellW = size.width / CGFloat(grid.width)
        let cellH = size.height / CGFloat(grid.height)
        let cellSize = min(cellW, cellH)
        
        let totalW = cellSize * CGFloat(grid.width)
        let totalH = cellSize * CGFloat(grid.height)
        let offsetX = (size.width - totalW) / 2
        let offsetY = (size.height - totalH) / 2
        
        for y in 0..<grid.height {
            for x in 0..<grid.width {
                let value = grid.data[y * grid.width + x]
                guard let color = cellColor(value) else { continue }
                
                // ROS grid is bottom-up, screen is top-down
                let screenY = grid.height - 1 - y
                let rect = CGRect(
                    x: offsetX + CGFloat(x) * cellSize,
                    y: offsetY + CGFloat(screenY) * cellSize,
                    width: cellSize + 0.5, // slight overlap to avoid gaps
                    height: cellSize + 0.5
                )
                context.fill(Path(rect), with: .color(color))
            }
        }
    }
    
    private func cellColor(_ value: Int8) -> Color? {
        switch value {
        case -1:
            // Unknown — skip (background shows through)
            return nil
        case 0:
            return style.freeColor
        case 100:
            return style.wallColor
        default:
            // Gradient: 0 (free) → 100 (occupied)
            let t = Double(value) / 100.0
            return style.freeColor.opacity(1 - t * 0.7)
        }
    }
    
    // MARK: - Robot
    
    private func drawRobot(_ grid: OccupancyGrid, context: GraphicsContext, size: CGSize) {
        guard let rx = robotX, let ry = robotY else { return }
        
        let cellW = size.width / CGFloat(grid.width)
        let cellH = size.height / CGFloat(grid.height)
        let cellSize = min(cellW, cellH)
        
        let totalW = cellSize * CGFloat(grid.width)
        let totalH = cellSize * CGFloat(grid.height)
        let offsetX = (size.width - totalW) / 2
        let offsetY = (size.height - totalH) / 2
        
        // World → grid → screen
        let gridX = (rx - grid.originX) / Double(grid.resolution)
        let gridY = (ry - grid.originY) / Double(grid.resolution)
        let screenX = offsetX + CGFloat(gridX) * cellSize
        let screenY = offsetY + CGFloat(Double(grid.height) - gridY) * cellSize
        
        let heading = CGFloat(robotYaw ?? 0)
        let s: CGFloat = max(cellSize * 3, 8)
        
        // Triangle
        var tri = Path()
        tri.move(to: CGPoint(
            x: screenX + CoreGraphics.cos(heading) * s,
            y: screenY - CoreGraphics.sin(heading) * s
        ))
        tri.addLine(to: CGPoint(
            x: screenX + CoreGraphics.cos(heading + 2.4) * s * 0.7,
            y: screenY - CoreGraphics.sin(heading + 2.4) * s * 0.7
        ))
        tri.addLine(to: CGPoint(
            x: screenX + CoreGraphics.cos(heading - 2.4) * s * 0.7,
            y: screenY - CoreGraphics.sin(heading - 2.4) * s * 0.7
        ))
        tri.closeSubpath()
        context.fill(tri, with: .color(.teal))
        
        // Heading line
        var line = Path()
        line.move(to: CGPoint(x: screenX, y: screenY))
        line.addLine(to: CGPoint(
            x: screenX + CoreGraphics.cos(heading) * s * 1.8,
            y: screenY - CoreGraphics.sin(heading) * s * 1.8
        ))
        context.stroke(line, with: .color(.teal.opacity(0.4)), lineWidth: 1)
    }
    
    // MARK: - Placeholder
    
    private var placeholder: some View {
        VStack(spacing: 12) {
            Image(systemName: "map")
                .font(.system(size: 32))
                .foregroundStyle(.secondary)
            Text("No Map Data")
                .font(.caption)
                .foregroundStyle(.tertiary)
        }
    }
}

// MARK: - MapStyle

/// Visual style for MapView.
public struct MapStyle: Sendable {
    public let background: Color
    public let freeColor: Color
    public let wallColor: Color
    
    public init(background: Color, freeColor: Color, wallColor: Color) {
        self.background = background
        self.freeColor = freeColor
        self.wallColor = wallColor
    }
    
    /// Dark theme (default).
    public static let dark = MapStyle(
        background: Color(white: 0.06),
        freeColor: Color(white: 0.15),
        wallColor: .white
    )
    
    /// Light theme.
    public static let light = MapStyle(
        background: Color(white: 0.95),
        freeColor: Color(white: 0.85),
        wallColor: Color(white: 0.1)
    )
    
    /// RViz-style (grey free, black walls, dark grey unknown).
    public static let rviz = MapStyle(
        background: Color(white: 0.3),
        freeColor: Color(white: 0.8),
        wallColor: .black
    )
}
