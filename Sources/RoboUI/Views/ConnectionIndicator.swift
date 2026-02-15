// RoboUI — ConnectionIndicator
// Visual indicator for connection state and latency.

import SwiftUI

/// Displays connection state with latency measurement.
///
/// Usage:
/// ```swift
/// ConnectionIndicator(connection: robot)
/// ```
///
/// Compact mode (just the dot + latency):
/// ```swift
/// ConnectionIndicator(connection: robot, style: .compact)
/// ```
public struct ConnectionIndicator: View {
    @ObservedObject var connection: RosbridgeConnection
    @ObservedObject var monitor: ConnectionMonitor
    let style: IndicatorStyle
    
    /// Create an indicator.
    /// - Parameters:
    ///   - connection: The rosbridge connection to monitor.
    ///   - style: Visual style. Default: `.badge`.
    ///   - pingInterval: How often to measure latency. Default: `5.0` seconds.
    public init(
        connection: RosbridgeConnection,
        style: IndicatorStyle = .badge,
        pingInterval: TimeInterval = 5.0
    ) {
        self.connection = connection
        self.style = style
        self.monitor = ConnectionMonitor(
            connection: connection,
            pingInterval: pingInterval
        )
    }
    
    public var body: some View {
        switch style {
        case .compact:
            compactView
        case .badge:
            badgeView
        case .detailed:
            detailedView
        }
    }
    
    // MARK: - Compact (dot + ms)
    
    private var compactView: some View {
        HStack(spacing: 4) {
            Circle()
                .fill(stateColor)
                .frame(width: 8, height: 8)
            
            if connection.state.isConnected, let latency = monitor.latencyMs {
                Text("\(Int(latency))ms")
                    .font(.system(size: 10, design: .monospaced))
                    .foregroundStyle(latencyColor)
            }
        }
    }
    
    // MARK: - Badge (pill with state + latency)
    
    private var badgeView: some View {
        HStack(spacing: 6) {
            Circle()
                .fill(stateColor)
                .frame(width: 8, height: 8)
            
            Text(stateLabel)
                .font(.caption2)
                .foregroundStyle(.secondary)
            
            if connection.state.isConnected, let latency = monitor.latencyMs {
                Text("•")
                    .foregroundStyle(.tertiary)
                Text("\(Int(latency))ms")
                    .font(.system(.caption2, design: .monospaced))
                    .foregroundStyle(latencyColor)
                
                qualityBars
            }
        }
        .padding(.horizontal, 10)
        .padding(.vertical, 5)
        .background(.ultraThinMaterial, in: Capsule())
    }
    
    // MARK: - Detailed (card with history)
    
    private var detailedView: some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack {
                Circle()
                    .fill(stateColor)
                    .frame(width: 10, height: 10)
                Text(stateLabel)
                    .font(.caption)
                    .fontWeight(.medium)
                Spacer()
                if let latency = monitor.latencyMs {
                    Text("\(Int(latency))ms")
                        .font(.system(.caption, design: .monospaced))
                        .foregroundStyle(latencyColor)
                }
            }
            
            if !monitor.latencyHistory.isEmpty {
                // Mini sparkline
                Canvas { context, size in
                    let history = monitor.latencyHistory.suffix(20)
                    guard history.count > 1 else { return }
                    let maxVal = max(history.max() ?? 1, 1)
                    
                    var path = Path()
                    for (i, val) in history.enumerated() {
                        let x = CGFloat(i) / CGFloat(history.count - 1) * size.width
                        let y = size.height - (val / maxVal) * size.height
                        if i == 0 { path.move(to: CGPoint(x: x, y: y)) }
                        else { path.addLine(to: CGPoint(x: x, y: y)) }
                    }
                    context.stroke(path, with: .color(.teal.opacity(0.7)), style: StrokeStyle(lineWidth: 1.5, lineCap: .round))
                }
                .frame(height: 24)
                
                HStack {
                    if let avg = monitor.averageLatencyMs {
                        Text("avg: \(Int(avg))ms")
                    }
                    Spacer()
                    Text(monitor.quality.label)
                        .foregroundStyle(monitor.quality.color)
                }
                .font(.system(size: 9, design: .monospaced))
                .foregroundStyle(.secondary)
            }
        }
        .padding(10)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 10))
    }
    
    // MARK: - Helpers
    
    private var stateColor: Color {
        switch connection.state {
        case .disconnected: .gray
        case .connecting: .orange
        case .connected: monitor.quality.color
        case .reconnecting: .yellow
        case .error: .red
        }
    }
    
    private var stateLabel: String {
        switch connection.state {
        case .disconnected: "Disconnected"
        case .connecting: "Connecting…"
        case .connected: "Connected"
        case .reconnecting(let n): "Reconnecting (\(n))…"
        case .error(let msg): "Error: \(msg)"
        }
    }
    
    private var latencyColor: Color {
        monitor.quality.color
    }
    
    private var qualityBars: some View {
        HStack(spacing: 1.5) {
            ForEach(0..<3) { i in
                RoundedRectangle(cornerRadius: 1)
                    .fill(i < monitor.quality.bars ? monitor.quality.color : .white.opacity(0.15))
                    .frame(width: 3, height: CGFloat(4 + i * 3))
            }
        }
    }
}

// MARK: - ConnectionState helpers

extension ConnectionState {
    /// Whether the connection is established.
    public var isConnected: Bool {
        if case .connected = self { return true }
        return false
    }
}

/// Visual style for ConnectionIndicator.
public enum IndicatorStyle {
    /// Just a dot and latency number.
    case compact
    /// Pill with state label, latency, and quality bars.
    case badge
    /// Card with sparkline history.
    case detailed
}

// MARK: - ConnectionMonitor

/// Measures connection latency via WebSocket ping/pong.
@MainActor
public final class ConnectionMonitor: ObservableObject {
    @Published public private(set) var latencyMs: Double?
    @Published public private(set) var averageLatencyMs: Double?
    @Published public private(set) var latencyHistory: [Double] = []
    @Published public private(set) var quality: ConnectionQuality = .unknown
    
    private weak var connection: RosbridgeConnection?
    private var pingTask: Task<Void, Never>?
    private let pingInterval: TimeInterval
    private let historySize = 30
    
    init(connection: RosbridgeConnection, pingInterval: TimeInterval) {
        self.connection = connection
        self.pingInterval = pingInterval
        startMonitoring()
    }
    
    private func startMonitoring() {
        pingTask = Task { [weak self] in
            while !Task.isCancelled {
                guard let self, let conn = self.connection else { break }
                
                if conn.state.isConnected {
                    await self.measureLatency(conn)
                } else {
                    self.quality = .unknown
                    self.latencyMs = nil
                }
                
                try? await Task.sleep(nanoseconds: UInt64(self.pingInterval * 1_000_000_000))
            }
        }
    }
    
    private func measureLatency(_ connection: RosbridgeConnection) async {
        let start = CFAbsoluteTimeGetCurrent()
        
        // Use rosbridge service call to /rosapi/get_time as a ping
        // This measures actual round-trip through rosbridge
        await withCheckedContinuation { (continuation: CheckedContinuation<Void, Never>) in
            connection.callService(service: "/rosapi/get_time") { _ in
                continuation.resume()
            }
            
            // Timeout after 5s
            Task {
                try? await Task.sleep(nanoseconds: 5_000_000_000)
                continuation.resume() // no-op if already resumed
            }
        }
        
        let elapsed = (CFAbsoluteTimeGetCurrent() - start) * 1000
        
        latencyMs = elapsed
        latencyHistory.append(elapsed)
        if latencyHistory.count > historySize {
            latencyHistory.removeFirst(latencyHistory.count - historySize)
        }
        averageLatencyMs = latencyHistory.reduce(0, +) / Double(latencyHistory.count)
        quality = ConnectionQuality.from(latencyMs: elapsed)
    }
    
    deinit {
        pingTask?.cancel()
    }
}

// MARK: - ConnectionQuality

/// Connection quality rating based on latency.
public enum ConnectionQuality: Sendable {
    case excellent  // < 50ms
    case good       // < 150ms
    case fair       // < 300ms
    case poor       // >= 300ms
    case unknown
    
    public var label: String {
        switch self {
        case .excellent: "Excellent"
        case .good: "Good"
        case .fair: "Fair"
        case .poor: "Poor"
        case .unknown: "—"
        }
    }
    
    public var color: Color {
        switch self {
        case .excellent: .green
        case .good: .green
        case .fair: .yellow
        case .poor: .red
        case .unknown: .gray
        }
    }
    
    public var bars: Int {
        switch self {
        case .excellent: 3
        case .good: 3
        case .fair: 2
        case .poor: 1
        case .unknown: 0
        }
    }
    
    public static func from(latencyMs: Double) -> ConnectionQuality {
        switch latencyMs {
        case ..<50: .excellent
        case ..<150: .good
        case ..<300: .fair
        default: .poor
        }
    }
}
