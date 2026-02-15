import SwiftUI

/// Visualizes LiDAR data as a radial point cloud.
///
/// Usage:
/// ```swift
/// LaserScanView(scan: viewModel.latestScan, robotHeading: heading)
/// ```
public struct LaserScanView: View {
    
    let scan: LaserScan?
    let robotHeading: Double
    let maxRange: CGFloat
    let dotSize: CGFloat
    let dotColor: Color
    let showGrid: Bool
    
    public init(
        scan: LaserScan?,
        robotHeading: Double = 0,
        maxRange: CGFloat = 8,
        dotSize: CGFloat = 2.5,
        dotColor: Color = .green,
        showGrid: Bool = true
    ) {
        self.scan = scan
        self.robotHeading = robotHeading
        self.maxRange = maxRange
        self.dotSize = dotSize
        self.dotColor = dotColor
        self.showGrid = showGrid
    }
    
    public var body: some View {
        Canvas { context, size in
            let center = CGPoint(x: size.width / 2, y: size.height / 2)
            let viewRadius = min(size.width, size.height) / 2 - 10
            let scale = viewRadius / maxRange
            
            drawGrid(context: context, center: center, viewRadius: viewRadius, scale: scale)
            drawRobot(context: context, center: center)
            drawScanPoints(context: context, center: center, scale: scale)
        }
    }
    
    private func drawGrid(context: GraphicsContext, center: CGPoint, viewRadius: CGFloat, scale: CGFloat) {
        guard showGrid else { return }
        
        for ring in 1...Int(maxRange / 2) {
            let r = CGFloat(ring) * 2 * scale
            var ringPath = Path()
            ringPath.addEllipse(in: CGRect(
                x: center.x - r, y: center.y - r,
                width: r * 2, height: r * 2
            ))
            context.stroke(ringPath, with: .color(.white.opacity(0.06)), lineWidth: 0.5)
        }
        
        var cross = Path()
        cross.move(to: CGPoint(x: center.x, y: center.y - viewRadius))
        cross.addLine(to: CGPoint(x: center.x, y: center.y + viewRadius))
        cross.move(to: CGPoint(x: center.x - viewRadius, y: center.y))
        cross.addLine(to: CGPoint(x: center.x + viewRadius, y: center.y))
        context.stroke(cross, with: .color(.white.opacity(0.05)), lineWidth: 0.5)
    }
    
    private func drawRobot(context: GraphicsContext, center: CGPoint) {
        let triSize: CGFloat = 6
        let h = CGFloat(robotHeading)
        var tri = Path()
        tri.move(to: CGPoint(
            x: center.x + cos(h) * triSize,
            y: center.y - sin(h) * triSize
        ))
        tri.addLine(to: CGPoint(
            x: center.x + cos(h + 2.4) * triSize * 0.6,
            y: center.y - sin(h + 2.4) * triSize * 0.6
        ))
        tri.addLine(to: CGPoint(
            x: center.x + cos(h - 2.4) * triSize * 0.6,
            y: center.y - sin(h - 2.4) * triSize * 0.6
        ))
        tri.closeSubpath()
        context.fill(tri, with: .color(.white.opacity(0.6)))
    }
    
    private func drawScanPoints(context: GraphicsContext, center: CGPoint, scale: CGFloat) {
        guard let scan else { return }
        let points = scan.points
        
        // Fade-out zone: points beyond fadeStart fade to 0 opacity at fadeEnd.
        // Close walls are crisp, far walls (which distort) smoothly disappear.
        let fadeStart: CGFloat = maxRange * 0.5   // start fading at 50% of max range
        let fadeEnd: CGFloat = maxRange * 0.85    // fully hidden at 85% of max range
        let fadeRange = fadeEnd - fadeStart
        
        for point in points {
            let heading = CGFloat(robotHeading)
            let px = CGFloat(point.x)
            let py = CGFloat(point.y)
            let rx = px * cos(heading) - py * sin(heading)
            let ry = px * sin(heading) + py * cos(heading)
            
            let screenX = center.x + rx * scale
            let screenY = center.y - ry * scale
            
            let dist = hypot(px, py)
            
            // Distance-based fade: close=bright, far=hidden
            let fadeAlpha: CGFloat
            if dist < fadeStart {
                fadeAlpha = 1.0
            } else if dist > fadeEnd {
                continue  // skip — fully faded, don't draw
            } else {
                fadeAlpha = 1.0 - (dist - fadeStart) / fadeRange
            }
            
            let intensity = (1.0 - min(dist / maxRange, 1.0) * 0.6) * fadeAlpha
            
            var dot = Path()
            dot.addEllipse(in: CGRect(
                x: screenX - dotSize / 2,
                y: screenY - dotSize / 2,
                width: dotSize,
                height: dotSize
            ))
            context.fill(dot, with: .color(dotColor.opacity(intensity)))
        }
    }
}
