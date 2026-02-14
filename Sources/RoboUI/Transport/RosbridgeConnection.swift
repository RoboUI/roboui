import Foundation
import Combine

/// Connection state for a rosbridge WebSocket
public enum ConnectionState: Sendable {
    case disconnected
    case connecting
    case connected
    case error(String)
}

/// A rosbridge v2.0 WebSocket connection to a ROS2 robot.
///
/// Usage:
/// ```swift
/// let robot = RosbridgeConnection(url: "ws://robot.local:9090")
/// await robot.connect()
/// ```
@MainActor
public final class RosbridgeConnection: ObservableObject {
    
    @Published public private(set) var state: ConnectionState = .disconnected
    
    private let url: URL
    private var webSocket: URLSessionWebSocketTask?
    private var session: URLSession?
    private var messageHandlers: [String: (Data) -> Void] = [:]
    private var idCounter: Int = 0
    private var receiveTask: Task<Void, Never>?
    
    public init(url: String) {
        guard let parsed = URL(string: url) else {
            fatalError("Invalid rosbridge URL: \(url)")
        }
        self.url = parsed
    }
    
    // MARK: - Connection
    
    public func connect() {
        guard case .disconnected = state else { return }
        state = .connecting
        
        session = URLSession(configuration: .default)
        webSocket = session?.webSocketTask(with: url)
        webSocket?.resume()
        
        state = .connected
        startReceiving()
    }
    
    public func disconnect() {
        receiveTask?.cancel()
        receiveTask = nil
        webSocket?.cancel(with: .normalClosure, reason: nil)
        webSocket = nil
        session?.invalidateAndCancel()
        session = nil
        state = .disconnected
        messageHandlers.removeAll()
    }
    
    // MARK: - Publishing
    
    /// Advertise a topic for publishing.
    public func advertise(topic: String, type: String) {
        let msg: [String: Any] = [
            "op": "advertise",
            "topic": topic,
            "type": type
        ]
        send(msg)
    }
    
    /// Publish a message to a topic.
    public func publish(topic: String, message: [String: Any]) {
        let msg: [String: Any] = [
            "op": "publish",
            "topic": topic,
            "msg": message
        ]
        send(msg)
    }
    
    /// Unadvertise a topic.
    public func unadvertise(topic: String) {
        let msg: [String: Any] = [
            "op": "unadvertise",
            "topic": topic
        ]
        send(msg)
    }
    
    // MARK: - Subscribing
    
    /// Subscribe to a topic and receive messages via callback.
    /// Returns an ID that can be used to unsubscribe.
    @discardableResult
    public func subscribe(
        topic: String,
        type: String? = nil,
        throttleRate: Int? = nil,
        handler: @escaping ([String: Any]) -> Void
    ) -> String {
        let id = nextID()
        
        var msg: [String: Any] = [
            "op": "subscribe",
            "id": id,
            "topic": topic
        ]
        if let type { msg["type"] = type }
        if let throttleRate { msg["throttle_rate"] = throttleRate }
        
        messageHandlers[topic] = { data in
            guard let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
                  let innerMsg = json["msg"] as? [String: Any] else { return }
            handler(innerMsg)
        }
        
        send(msg)
        return id
    }
    
    /// Unsubscribe from a topic.
    public func unsubscribe(topic: String, id: String? = nil) {
        var msg: [String: Any] = [
            "op": "unsubscribe",
            "topic": topic
        ]
        if let id { msg["id"] = id }
        
        messageHandlers.removeValue(forKey: topic)
        send(msg)
    }
    
    // MARK: - Service Calls
    
    /// Call a ROS2 service.
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
    
    // MARK: - Private
    
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
                        self?.state = .error(error.localizedDescription)
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
        
        guard let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else { return }
        
        // Route to appropriate handler
        if let topic = json["topic"] as? String, let handler = messageHandlers[topic] {
            handler(data)
        } else if let op = json["op"] as? String, op == "service_response",
                  let id = json["id"] as? String, let handler = messageHandlers["service:\(id)"] {
            handler(data)
            messageHandlers.removeValue(forKey: "service:\(id)")
        }
    }
}
