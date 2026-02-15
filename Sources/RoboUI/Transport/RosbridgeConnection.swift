import Foundation
import Combine

/// Connection state for a rosbridge WebSocket
public enum ConnectionState: Sendable {
    case disconnected
    case connecting
    case connected
    case reconnecting(attempt: Int)
    case error(String)
}

/// Configuration for auto-reconnect behavior.
public struct ReconnectConfig: Sendable {
    /// Enable automatic reconnection. Default: `true`.
    public var enabled: Bool
    /// Initial delay before first reconnect attempt. Default: `1.0` seconds.
    public var initialDelay: TimeInterval
    /// Maximum delay between reconnect attempts. Default: `30.0` seconds.
    public var maxDelay: TimeInterval
    /// Multiplier for exponential backoff. Default: `2.0`.
    public var multiplier: Double
    /// Maximum number of reconnect attempts. `nil` = unlimited. Default: `nil`.
    public var maxAttempts: Int?
    
    public static let `default` = ReconnectConfig(
        enabled: true,
        initialDelay: 1.0,
        maxDelay: 30.0,
        multiplier: 2.0,
        maxAttempts: nil
    )
    
    public static let disabled = ReconnectConfig(
        enabled: false,
        initialDelay: 1.0,
        maxDelay: 30.0,
        multiplier: 2.0,
        maxAttempts: nil
    )
    
    public init(
        enabled: Bool = true,
        initialDelay: TimeInterval = 1.0,
        maxDelay: TimeInterval = 30.0,
        multiplier: Double = 2.0,
        maxAttempts: Int? = nil
    ) {
        self.enabled = enabled
        self.initialDelay = initialDelay
        self.maxDelay = maxDelay
        self.multiplier = multiplier
        self.maxAttempts = maxAttempts
    }
}

/// A rosbridge v2.0 WebSocket connection to a ROS2 robot.
///
/// Supports automatic reconnection with exponential backoff.
///
/// Usage:
/// ```swift
/// let robot = RosbridgeConnection(url: "ws://robot.local:9090")
/// robot.connect()
/// // Auto-reconnects on disconnect. Call disconnect() to stop.
/// ```
@MainActor
public final class RosbridgeConnection: ObservableObject {
    
    @Published public private(set) var state: ConnectionState = .disconnected
    
    /// Reconnect configuration. Change before calling `connect()`.
    public var reconnectConfig: ReconnectConfig
    
    private var url: URL
    private var webSocket: URLSessionWebSocketTask?
    private var session: URLSession?
    private var messageHandlers: [String: (Data) -> Void] = [:]
    private var pendingSubscriptions: [(topic: String, type: String?, throttleRate: Int?, handler: ([String: Any]) -> Void)] = []
    private var pendingAdvertisements: [(topic: String, type: String)] = []
    private var idCounter: Int = 0
    private var receiveTask: Task<Void, Never>?
    private var reconnectTask: Task<Void, Never>?
    private var reconnectAttempt: Int = 0
    private var intentionalDisconnect = false
    
    public init(url: String, reconnect: ReconnectConfig = .default) {
        guard let parsed = URL(string: url) else {
            fatalError("Invalid rosbridge URL: \(url)")
        }
        self.url = parsed
        self.reconnectConfig = reconnect
    }
    
    // MARK: - Connection
    
    public func connect() {
        intentionalDisconnect = false
        reconnectAttempt = 0
        cancelReconnect()
        performConnect()
    }
    
    /// Connect to a new URL. Disconnects from current if connected.
    public func connect(to newURL: String) {
        guard let parsed = URL(string: newURL) else {
            state = .error("Invalid URL: \(newURL)")
            return
        }
        disconnect()
        url = parsed
        connect()
    }
    
    public func disconnect() {
        intentionalDisconnect = true
        cancelReconnect()
        teardownSocket()
        state = .disconnected
    }
    
    // MARK: - Publishing
    
    public func advertise(topic: String, type: String, latch: Bool = false) {
        // Track for re-advertise on reconnect
        if !pendingAdvertisements.contains(where: { $0.topic == topic }) {
            pendingAdvertisements.append((topic: topic, type: type))
        }
        
        let msg: [String: Any] = [
            "op": "advertise",
            "topic": topic,
            "type": type,
            "latch": latch  // false = VOLATILE QoS (compatible with most ROS2 subscribers)
        ]
        send(msg)
    }
    
    public func publish(topic: String, message: [String: Any]) {
        let msg: [String: Any] = [
            "op": "publish",
            "topic": topic,
            "msg": message
        ]
        send(msg)
    }
    
    public func unadvertise(topic: String) {
        pendingAdvertisements.removeAll { $0.topic == topic }
        let msg: [String: Any] = [
            "op": "unadvertise",
            "topic": topic
        ]
        send(msg)
    }
    
    // MARK: - Subscribing
    
    @discardableResult
    public func subscribe(
        topic: String,
        type: String? = nil,
        throttleRate: Int? = nil,
        handler: @escaping ([String: Any]) -> Void
    ) -> String {
        let id = nextID()
        
        // Track for re-subscribe on reconnect
        pendingSubscriptions.removeAll { $0.topic == topic }
        pendingSubscriptions.append((topic: topic, type: type, throttleRate: throttleRate, handler: handler))
        
        messageHandlers[topic] = { data in
            guard let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
                  let innerMsg = json["msg"] as? [String: Any] else { return }
            handler(innerMsg)
        }
        
        sendSubscribe(id: id, topic: topic, type: type, throttleRate: throttleRate)
        return id
    }
    
    public func unsubscribe(topic: String, id: String? = nil) {
        pendingSubscriptions.removeAll { $0.topic == topic }
        messageHandlers.removeValue(forKey: topic)
        
        var msg: [String: Any] = [
            "op": "unsubscribe",
            "topic": topic
        ]
        if let id { msg["id"] = id }
        send(msg)
    }
    
    // MARK: - Service Calls
    
    public func callService(
        service: String,
        args: [String: Any]? = nil,
        handler: @escaping ([String: Any]) -> Void
    ) {
        let id = nextID()
        
        var msg: [String: Any] = [
            "op": "call_service",
            "id": id,
            "service": service
        ]
        if let args { msg["args"] = args }
        
        messageHandlers["service:\(id)"] = { data in
            guard let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
                  let values = json["values"] as? [String: Any] else { return }
            handler(values)
        }
        
        send(msg)
    }
    
    // MARK: - Private: Connection Management
    
    private func performConnect() {
        state = reconnectAttempt > 0 ? .reconnecting(attempt: reconnectAttempt) : .connecting
        
        session = URLSession(configuration: .default)
        webSocket = session?.webSocketTask(with: url)
        webSocket?.resume()
        
        // Send a ping to verify connection is actually alive
        webSocket?.sendPing { [weak self] error in
            Task { @MainActor in
                if let error {
                    self?.handleConnectionFailure(error)
                } else {
                    self?.handleConnectionSuccess()
                }
            }
        }
    }
    
    private func handleConnectionSuccess() {
        state = .connected
        reconnectAttempt = 0
        
        // Re-subscribe all topics
        resubscribeAll()
        
        // Re-advertise all topics
        readvertiseAll()
        
        startReceiving()
    }
    
    private func handleConnectionFailure(_ error: Error) {
        teardownSocket()
        
        if intentionalDisconnect { return }
        
        state = .error(error.localizedDescription)
        scheduleReconnect()
    }
    
    private func handleDisconnect(error: Error?) {
        teardownSocket()
        
        if intentionalDisconnect { return }
        
        if let error {
            state = .error(error.localizedDescription)
        }
        scheduleReconnect()
    }
    
    private func teardownSocket() {
        receiveTask?.cancel()
        receiveTask = nil
        webSocket?.cancel(with: .normalClosure, reason: nil)
        webSocket = nil
        session?.invalidateAndCancel()
        session = nil
    }
    
    // MARK: - Private: Reconnect
    
    private func scheduleReconnect() {
        guard reconnectConfig.enabled, !intentionalDisconnect else { return }
        
        if let max = reconnectConfig.maxAttempts, reconnectAttempt >= max {
            state = .error("Max reconnect attempts (\(max)) reached")
            return
        }
        
        reconnectAttempt += 1
        let delay = min(
            reconnectConfig.initialDelay * pow(reconnectConfig.multiplier, Double(reconnectAttempt - 1)),
            reconnectConfig.maxDelay
        )
        
        state = .reconnecting(attempt: reconnectAttempt)
        
        reconnectTask = Task { [weak self] in
            try? await Task.sleep(nanoseconds: UInt64(delay * 1_000_000_000))
            guard !Task.isCancelled else { return }
            await self?.performConnect()
        }
    }
    
    private func cancelReconnect() {
        reconnectTask?.cancel()
        reconnectTask = nil
    }
    
    // MARK: - Private: Re-subscribe/Re-advertise
    
    private func resubscribeAll() {
        for sub in pendingSubscriptions {
            let id = nextID()
            
            messageHandlers[sub.topic] = { data in
                guard let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
                      let innerMsg = json["msg"] as? [String: Any] else { return }
                sub.handler(innerMsg)
            }
            
            sendSubscribe(id: id, topic: sub.topic, type: sub.type, throttleRate: sub.throttleRate)
        }
    }
    
    private func readvertiseAll() {
        for adv in pendingAdvertisements {
            let msg: [String: Any] = [
                "op": "advertise",
                "topic": adv.topic,
                "type": adv.type
            ]
            send(msg)
        }
    }
    
    private func sendSubscribe(id: String, topic: String, type: String?, throttleRate: Int?) {
        var msg: [String: Any] = [
            "op": "subscribe",
            "id": id,
            "topic": topic
        ]
        if let type { msg["type"] = type }
        if let throttleRate { msg["throttle_rate"] = throttleRate }
        send(msg)
    }
    
    // MARK: - Private: Messaging
    
    private func nextID() -> String {
        idCounter += 1
        return "roboui_\(idCounter)"
    }
    
    private func send(_ dict: [String: Any]) {
        guard let data = try? JSONSerialization.data(withJSONObject: dict),
              let text = String(data: data, encoding: .utf8) else { return }
        
        webSocket?.send(.string(text)) { error in
            if let error {
                print("[RoboUI] Send error: \(error.localizedDescription)")
            }
        }
    }
    
    private func startReceiving() {
        receiveTask = Task { [weak self] in
            while !Task.isCancelled {
                guard let ws = self?.webSocket else { break }
                do {
                    let message = try await ws.receive()
                    await self?.handleMessage(message)
                } catch {
                    await MainActor.run {
                        self?.handleDisconnect(error: error)
                    }
                    break
                }
            }
        }
    }
    
    private func handleMessage(_ message: URLSessionWebSocketTask.Message) {
        let data: Data
        switch message {
        case .string(let text):
            data = Data(text.utf8)
        case .data(let d):
            data = d
        @unknown default:
            return
        }
        
        guard let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else {
            return
        }
        
        if let topic = json["topic"] as? String {
            if let handler = messageHandlers[topic] {
                handler(data)
            }
        } else if let op = json["op"] as? String, op == "service_response",
                  let id = json["id"] as? String, let handler = messageHandlers["service:\(id)"] {
            handler(data)
            messageHandlers.removeValue(forKey: "service:\(id)")
        }
    }
}
