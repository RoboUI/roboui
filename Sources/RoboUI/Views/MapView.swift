// RoboUI — MapView
// Renders a nav_msgs/OccupancyGrid as a 2D map using CGImage (fast).
// Supports auto-fit to occupied area, pinch-to-zoom, and pan gestures.

import SwiftUI
import CoreGraphics

/// A SwiftUI view that renders a ROS2 OccupancyGrid as a 2D map.
public struct MapView: View {
    let grid: OccupancyGrid?
    let robotX: Double?
    let robotY: Double?
    let robotYaw: Double?
    let style: MapStyle
    
    // Zoom/pan state
    @State private var zoomScale: CGFloat = 1.0
    @State private var panOffset: CGSize = .zero
    @State private var lastPanOffset: CGSize = .zero
    @State private var lastZoomScale: CGFloat = 1.0
    
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
                if let cgImage = renderGridImage(grid) {
                    GeometryReader { geo in
                        let bounds = occupiedBounds(grid)
                        let autoFit = autoFitTransform(bounds: bounds, grid: grid, viewSize: geo.size)
                        
                        Canvas { context, size in
                            // Combined transform: auto-fit + user zoom/pan
                            let scale = autoFit.scale * zoomScale
                            let cx = size.width / 2 + (autoFit.offsetX + panOffset.width) * zoomScale
                            let cy = size.height / 2 + (autoFit.offsetY + panOffset.height) * zoomScale
                            
                            let totalW = scale * CGFloat(grid.width)
                            let totalH = scale * CGFloat(grid.height)
                            let ox = cx - totalW / 2
                            let oy = cy - totalH / 2
                            
                            // Draw map image
                            let imgRect = CGRect(x: ox, y: oy, width: totalW, height: totalH)
                            context.draw(Image(decorative: cgImage, scale: 1), in: imgRect)
                            
                            // Draw robot
                            drawRobot(grid, context: context, size: size,
                                      cellSize: scale, offsetX: ox, offsetY: oy)
                        }
                        .gesture(pinchAndPanGesture())
                    }
                } else {
                    placeholder
                }
                
                // Zoom controls + info
                VStack {
                    Spacer()
                    HStack {
                        Spacer()
                        VStack(spacing: 4) {
                            zoomButton(icon: "plus", action: { withAnimation(.easeOut(duration: 0.2)) { zoomScale = min(zoomScale * 1.4, 8) } })
                            zoomButton(icon: "minus", action: { withAnimation(.easeOut(duration: 0.2)) { zoomScale = max(zoomScale / 1.4, 0.3) } })
                            zoomButton(icon: "arrow.counterclockwise", action: {
                                withAnimation(.easeOut(duration: 0.25)) {
                                    zoomScale = 1.0; panOffset = .zero; lastPanOffset = .zero; lastZoomScale = 1.0
                                }
                            })
                        }
                        .padding(6)
                    }
                    HStack {
                        let z = Int(zoomScale * 100)
                        Text("\(grid.width)×\(grid.height) • \(String(format: "%.2f", grid.resolution))m/px • \(z)%")
                            .font(.system(size: 9, design: .monospaced))
                            .foregroundStyle(.white.opacity(0.4))
                            .padding(4)
                        Spacer()
                    }
                }
            } else {
                placeholder
            }
        }
        .clipShape(RoundedRectangle(cornerRadius: 12))
        .overlay(
            RoundedRectangle(cornerRadius: 12)
                .stroke(.white.opacity(0.08), lineWidth: 1)
        )
    }
    
    // MARK: - Auto-fit
    
    /// Find bounding box of non-unknown cells (walls + free space).
    private func occupiedBounds(_ grid: OccupancyGrid) -> (minX: Int, minY: Int, maxX: Int, maxY: Int)? {
        var minX = grid.width, minY = grid.height, maxX = 0, maxY = 0
        var found = false
        for y in 0 ..< grid.height {
            for x in 0 ..< grid.width {
                if grid.data[y * grid.width + x] != -1 {
                    if x < minX { minX = x }
                    if x > maxX { maxX = x }
                    if y < minY { minY = y }
                    if y > maxY { maxY = y }
                    found = true
                }
            }
        }
        return found ? (minX, minY, maxX, maxY) : nil
    }
    
    /// Compute scale and offset to fit the occupied region with padding.
    private func autoFitTransform(bounds: (minX: Int, minY: Int, maxX: Int, maxY: Int)?,
                                   grid: OccupancyGrid, viewSize: CGSize)
        -> (scale: CGFloat, offsetX: CGFloat, offsetY: CGFloat)
    {
        guard let b = bounds else {
            // No data — fit whole grid
            let s = min(viewSize.width / CGFloat(grid.width), viewSize.height / CGFloat(grid.height))
            return (s, 0, 0)
        }
        
        let pad = 15 // pixel padding around occupied area
        let bw = CGFloat(b.maxX - b.minX + 1 + pad * 2)
        let bh = CGFloat(b.maxY - b.minY + 1 + pad * 2)
        let scale = min(viewSize.width / bw, viewSize.height / bh)
        
        // Center of occupied region in grid coords
        let centerX = CGFloat(b.minX + b.maxX) / 2.0
        let centerY = CGFloat(b.minY + b.maxY) / 2.0
        // Center of full grid
        let gridCenterX = CGFloat(grid.width) / 2.0
        let gridCenterY = CGFloat(grid.height) / 2.0
        // Offset to shift occupied center to view center
        let offsetX = (gridCenterX - centerX) * scale
        let offsetY = (gridCenterY - centerY) * scale
        
        return (scale, offsetX, offsetY)
    }
    
    // MARK: - Gestures
    
    private func pinchAndPanGesture() -> some Gesture {
        SimultaneousGesture(
            MagnificationGesture()
                .onChanged { value in
                    zoomScale = max(0.3, min(8, lastZoomScale * value))
                }
                .onEnded { value in
                    lastZoomScale = zoomScale
                },
            DragGesture()
                .onChanged { value in
                    panOffset = CGSize(
                        width: lastPanOffset.width + value.translation.width / zoomScale,
                        height: lastPanOffset.height + value.translation.height / zoomScale
                    )
                }
                .onEnded { _ in
                    lastPanOffset = panOffset
                }
        )
    }
    
    // MARK: - Zoom Buttons
    
    private func zoomButton(icon: String, action: @escaping () -> Void) -> some View {
        Button(action: action) {
            Image(systemName: icon)
                .font(.system(size: 12, weight: .bold))
                .foregroundStyle(.white.opacity(0.7))
                .frame(width: 28, height: 28)
                .background(.ultraThinMaterial.opacity(0.6))
                .clipShape(Circle())
        }
    }
    
    // MARK: - CGImage Rendering (fast)
    
    /// Render the occupancy grid to a CGImage. One pixel per cell.
    /// The image is then scaled by Canvas to fit the view.
    private func renderGridImage(_ grid: OccupancyGrid) -> CGImage? {
        let w = grid.width
        let h = grid.height
        let count = w * h
        guard count > 0, grid.data.count == count else { return nil }
        
        // RGBA pixel buffer (4 bytes per pixel)
        var pixels = [UInt8](repeating: 0, count: count * 4)
        
        let (bgR, bgG, bgB) = style.backgroundRGB
        let (freeR, freeG, freeB) = style.freeRGB
        let (wallR, wallG, wallB) = style.wallRGB
        
        for y in 0 ..< h {
            // ROS grid is bottom-up, image is top-down
            let screenY = h - 1 - y
            for x in 0 ..< w {
                let value = grid.data[y * w + x]
                let pixIdx = (screenY * w + x) * 4
                
                let r: UInt8, g: UInt8, b: UInt8, a: UInt8
                
                switch value {
                case -1:
                    // Unknown — background
                    r = bgR; g = bgG; b = bgB; a = 255
                case 0:
                    // Free
                    r = freeR; g = freeG; b = freeB; a = 255
                case 100:
                    // Wall
                    r = wallR; g = wallG; b = wallB; a = 255
                default:
                    // Gradient: lerp between free and wall
                    let t = Float(max(0, min(100, value))) / 100.0
                    r = UInt8(Float(freeR) + (Float(wallR) - Float(freeR)) * t)
                    g = UInt8(Float(freeG) + (Float(wallG) - Float(freeG)) * t)
                    b = UInt8(Float(freeB) + (Float(wallB) - Float(freeB)) * t)
                    a = 255
                }
                
                pixels[pixIdx] = r
                pixels[pixIdx + 1] = g
                pixels[pixIdx + 2] = b
                pixels[pixIdx + 3] = a
            }
        }
        
        let colorSpace = CGColorSpaceCreateDeviceRGB()
        guard let context = CGContext(
            data: &pixels,
            width: w,
            height: h,
            bitsPerComponent: 8,
            bytesPerRow: w * 4,
            space: colorSpace,
            bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue
        ) else { return nil }
        
        // Nearest-neighbor scaling to keep crisp pixels
        context.interpolationQuality = .none
        
        return context.makeImage()
    }
    
    // MARK: - Robot
    
    private func drawRobot(_ grid: OccupancyGrid, context: GraphicsContext, size: CGSize,
                           cellSize: CGFloat, offsetX: CGFloat, offsetY: CGFloat) {
        guard let rx = robotX, let ry = robotY else { return }
        
        let gridX = (rx - grid.originX) / Double(grid.resolution)
        let gridY = (ry - grid.originY) / Double(grid.resolution)
        let screenX = offsetX + CGFloat(gridX) * cellSize
        let screenY = offsetY + CGFloat(Double(grid.height) - gridY) * cellSize
        
        let heading = CGFloat(robotYaw ?? 0)
        let s: CGFloat = max(cellSize * 3, 8)
        
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

public struct MapStyle: Sendable {
    public let background: Color
    public let freeColor: Color
    public let wallColor: Color
    
    // Pre-computed RGB for fast pixel rendering
    let backgroundRGB: (UInt8, UInt8, UInt8)
    let freeRGB: (UInt8, UInt8, UInt8)
    let wallRGB: (UInt8, UInt8, UInt8)
    
    public init(background: Color, freeColor: Color, wallColor: Color) {
        self.background = background
        self.freeColor = freeColor
        self.wallColor = wallColor
        self.backgroundRGB = Self.colorToRGB(background)
        self.freeRGB = Self.colorToRGB(freeColor)
        self.wallRGB = Self.colorToRGB(wallColor)
    }
    
    private static func colorToRGB(_ color: Color) -> (UInt8, UInt8, UInt8) {
        // Approximate — works for grayscale and simple colors
        let resolved = UIColor(color)
        var r: CGFloat = 0, g: CGFloat = 0, b: CGFloat = 0, a: CGFloat = 0
        resolved.getRed(&r, green: &g, blue: &b, alpha: &a)
        return (UInt8(r * 255), UInt8(g * 255), UInt8(b * 255))
    }
    
    public static let dark = MapStyle(
        background: Color(white: 0.06),
        freeColor: Color(white: 0.15),
        wallColor: .white
    )
    
    public static let light = MapStyle(
        background: Color(white: 0.95),
        freeColor: Color(white: 0.85),
        wallColor: Color(white: 0.1)
    )
    
    public static let rviz = MapStyle(
        background: Color(white: 0.3),
        freeColor: Color(white: 0.8),
        wallColor: .black
    )
}
