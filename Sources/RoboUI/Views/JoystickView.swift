import SwiftUI

/// A virtual joystick for robot teleoperation.
///
/// Usage:
/// ```swift
/// JoystickView { x, y in
///     publisher.drive(linear: y * 0.5, angular: -x * 1.0)
/// }
/// ```
public struct JoystickView: View {
    
    public typealias JoystickHandler = (_ x: Double, _ y: Double) -> Void
    
    private let size: CGFloat
    private let knobRatio: CGFloat
    private let onChange: JoystickHandler
    private let onRelease: (() -> Void)?
    
    @State private var knobOffset: CGSize = .zero
    @State private var isDragging = false
    
    public init(
        size: CGFloat = 150,
        knobRatio: CGFloat = 0.4,
        onRelease: (() -> Void)? = nil,
        onChange: @escaping JoystickHandler
    ) {
        self.size = size
        self.knobRatio = knobRatio
        self.onChange = onChange
        self.onRelease = onRelease
    }
    
    private var maxRadius: CGFloat { size / 2 }
    private var knobSize: CGFloat { size * knobRatio }
    
    public var body: some View {
        ZStack {
            // Base circle
            Circle()
                .fill(.ultraThinMaterial)
                .frame(width: size, height: size)
                .overlay(
                    Circle()
                        .stroke(.secondary.opacity(0.3), lineWidth: 2)
                )
            
            // Crosshair guides
            Path { path in
                path.move(to: CGPoint(x: size / 2, y: 10))
                path.addLine(to: CGPoint(x: size / 2, y: size - 10))
                path.move(to: CGPoint(x: 10, y: size / 2))
                path.addLine(to: CGPoint(x: size - 10, y: size / 2))
            }
            .stroke(.secondary.opacity(0.15), lineWidth: 1)
            .frame(width: size, height: size)
            
            // Knob
            Circle()
                .fill(isDragging ? Color.accentColor : Color.accentColor.opacity(0.7))
                .frame(width: knobSize, height: knobSize)
                .shadow(color: .black.opacity(0.2), radius: isDragging ? 8 : 4)
                .offset(knobOffset)
        }
        .frame(width: size, height: size)
        .gesture(
            DragGesture()
                .onChanged { value in
                    isDragging = true
                    let translation = value.translation
                    let distance = hypot(translation.width, translation.height)
                    
                    if distance <= maxRadius {
                        knobOffset = translation
                    } else {
                        // Clamp to circle boundary
                        let angle = atan2(translation.height, translation.width)
                        knobOffset = CGSize(
                            width: cos(angle) * maxRadius,
                            height: sin(angle) * maxRadius
                        )
                    }
                    
                    // Normalize to -1...1
                    let normalizedX = knobOffset.width / maxRadius
                    let normalizedY = -knobOffset.height / maxRadius // Invert Y
                    onChange(normalizedX, normalizedY)
                }
                .onEnded { _ in
                    isDragging = false
                    withAnimation(.spring(response: 0.3, dampingFraction: 0.6)) {
                        knobOffset = .zero
                    }
                    onChange(0, 0)
                    onRelease?()
                }
        )
    }
}

#Preview {
    JoystickView { x, y in
        print("x: \(x), y: \(y)")
    }
    .padding()
}
