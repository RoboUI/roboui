// RoboUI — EmergencyStopButton
// One-tap safety stop with haptic feedback.

import SwiftUI

/// A prominent emergency stop button that publishes zero-velocity Twist to stop the robot.
///
/// Usage:
/// ```swift
/// EmergencyStopButton(publisher: twistPublisher)
/// ```
///
/// Or with custom action:
/// ```swift
/// EmergencyStopButton {
///     myPublisher.emergencyStop()
///     // additional cleanup...
/// }
/// ```
public struct EmergencyStopButton: View {
    private let action: () -> Void
    private let size: EmergencyStopSize
    
    /// Create with a TwistPublisher (calls `emergencyStop()` on it).
    public init(publisher: TwistPublisher, size: EmergencyStopSize = .medium) {
        self.size = size
        self.action = { publisher.emergencyStop() }
    }
    
    /// Create with a custom action.
    public init(size: EmergencyStopSize = .medium, action: @escaping () -> Void) {
        self.size = size
        self.action = action
    }
    
    public var body: some View {
        Button {
            action()
            #if canImport(UIKit)
            let impact = UIImpactFeedbackGenerator(style: .heavy)
            impact.impactOccurred()
            // Double haptic for urgency
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.1) {
                let notify = UINotificationFeedbackGenerator()
                notify.notificationOccurred(.error)
            }
            #endif
        } label: {
            HStack(spacing: 6) {
                Image(systemName: "exclamationmark.octagon.fill")
                    .font(size.iconFont)
                Text("E-STOP")
                    .font(size.textFont)
            }
            .foregroundStyle(.white)
            .frame(maxWidth: size.maxWidth)
            .frame(height: size.height)
            .background(.red, in: RoundedRectangle(cornerRadius: size.cornerRadius))
        }
        .buttonStyle(.plain)
        #if canImport(UIKit)
        .accessibilityLabel("Emergency Stop")
        .accessibilityHint("Immediately stops all robot movement")
        #endif
    }
}

/// Size presets for EmergencyStopButton.
public enum EmergencyStopSize {
    case compact
    case medium
    case large
    
    var maxWidth: CGFloat {
        switch self {
        case .compact: 100
        case .medium: 160
        case .large: 240
        }
    }
    
    var height: CGFloat {
        switch self {
        case .compact: 36
        case .medium: 44
        case .large: 56
        }
    }
    
    var cornerRadius: CGFloat {
        switch self {
        case .compact: 8
        case .medium: 12
        case .large: 16
        }
    }
    
    var iconFont: Font {
        switch self {
        case .compact: .caption
        case .medium: .subheadline
        case .large: .title3
        }
    }
    
    var textFont: Font {
        switch self {
        case .compact: .system(.caption, design: .rounded, weight: .black)
        case .medium: .system(.subheadline, design: .rounded, weight: .black)
        case .large: .system(.title3, design: .rounded, weight: .black)
        }
    }
}
