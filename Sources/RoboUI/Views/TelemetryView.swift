// RoboUI — TelemetryView
// Displays robot telemetry data (battery, speed, custom sensors).

import SwiftUI

/// A single telemetry reading.
public struct TelemetryItem: Identifiable {
    public let id: String
    public let label: String
    public let value: String
    public let icon: String?
    public let color: Color
    public let progress: Double? // 0.0-1.0 for bar display, nil for text-only
    
    public init(
        id: String? = nil,
        label: String,
        value: String,
        icon: String? = nil,
        color: Color = .teal,
        progress: Double? = nil
    ) {
        self.id = id ?? label
        self.label = label
        self.value = value
        self.icon = icon
        self.color = color
        self.progress = progress
    }
}

/// A compact telemetry panel showing robot sensor data.
///
/// Usage:
/// ```swift
/// TelemetryView(items: [
///     TelemetryItem(label: "Battery", value: "87%", icon: "battery.75", color: .green, progress: 0.87),
///     TelemetryItem(label: "Speed", value: "0.8 m/s", icon: "speedometer", color: .teal),
///     TelemetryItem(label: "IMU", value: "2.1°", icon: "gyroscope", color: .orange),
/// ])
/// ```
public struct TelemetryView: View {
    let items: [TelemetryItem]
    let style: TelemetryStyle
    
    public init(items: [TelemetryItem], style: TelemetryStyle = .compact) {
        self.items = items
        self.style = style
    }
    
    public var body: some View {
        switch style {
        case .compact:
            compactLayout
        case .grid:
            gridLayout
        case .list:
            listLayout
        }
    }
    
    // MARK: - Compact (horizontal pills)
    
    private var compactLayout: some View {
        HStack(spacing: 8) {
            ForEach(items) { item in
                compactItem(item)
            }
        }
    }
    
    private func compactItem(_ item: TelemetryItem) -> some View {
        HStack(spacing: 4) {
            if let icon = item.icon {
                Image(systemName: icon)
                    .font(.caption2)
                    .foregroundStyle(item.color)
            }
            Text(item.value)
                .font(.system(.caption, design: .monospaced))
                .foregroundStyle(.primary)
        }
        .padding(.horizontal, 8)
        .padding(.vertical, 4)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 6))
    }
    
    // MARK: - Grid (cards)
    
    private var gridLayout: some View {
        LazyVGrid(columns: [
            GridItem(.flexible()),
            GridItem(.flexible()),
            GridItem(.flexible())
        ], spacing: 8) {
            ForEach(items) { item in
                gridCard(item)
            }
        }
    }
    
    private func gridCard(_ item: TelemetryItem) -> some View {
        VStack(spacing: 4) {
            if let icon = item.icon {
                Image(systemName: icon)
                    .font(.caption)
                    .foregroundStyle(item.color)
            }
            Text(item.label)
                .font(.system(size: 9))
                .foregroundStyle(.secondary)
            Text(item.value)
                .font(.system(.callout, design: .monospaced, weight: .semibold))
                .foregroundStyle(.primary)
            
            if let progress = item.progress {
                GeometryReader { geo in
                    ZStack(alignment: .leading) {
                        RoundedRectangle(cornerRadius: 2)
                            .fill(.white.opacity(0.1))
                        RoundedRectangle(cornerRadius: 2)
                            .fill(item.color)
                            .frame(width: geo.size.width * max(0, min(progress, 1)))
                    }
                }
                .frame(height: 3)
            }
        }
        .padding(8)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 10))
    }
    
    // MARK: - List (rows)
    
    private var listLayout: some View {
        VStack(spacing: 4) {
            ForEach(items) { item in
                HStack {
                    if let icon = item.icon {
                        Image(systemName: icon)
                            .font(.caption)
                            .foregroundStyle(item.color)
                            .frame(width: 20)
                    }
                    Text(item.label)
                        .font(.caption)
                        .foregroundStyle(.secondary)
                    Spacer()
                    Text(item.value)
                        .font(.system(.caption, design: .monospaced))
                        .foregroundStyle(.primary)
                }
                .padding(.horizontal, 10)
                .padding(.vertical, 4)
            }
        }
        .padding(.vertical, 6)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 10))
    }
}

/// Display style for TelemetryView.
public enum TelemetryStyle {
    /// Horizontal pills (minimal space).
    case compact
    /// Grid of cards with optional progress bars.
    case grid
    /// Vertical list with labels and values.
    case list
}
