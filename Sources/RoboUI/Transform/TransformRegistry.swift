import Foundation

/// Client-side TF2 transform registry for RoboUI.
///
/// Subscribes to `/tf` and `/tf_static` via rosbridge, maintains a time-buffered
/// frame tree, and provides `lookupTransform()` with SLERP interpolation.
///
/// Inspired by ROS2 tf2 and the Rust `transforms` crate.
///
/// Usage:
/// ```swift
/// let registry = TransformRegistry(connection: robot)
/// registry.start()
///
/// // Later, when rendering a sensor message:
/// if let tf = registry.lookupTransform(parent: "odom", child: "base_link", time: scan.timestamp) {
///     // Use tf.translation and tf.rotation to place scan points in odom frame
/// }
/// ```
@MainActor
public final class TransformRegistry: ObservableObject {
    
    /// All known frame IDs.
    @Published public private(set) var knownFrames: Set<String> = []
    
    /// Whether the registry is receiving data.
    @Published public private(set) var isActive: Bool = false
    
    private let connection: RosbridgeConnection
    private let bufferDuration: Double  // seconds
    
    // Frame pair → buffer: key is "parent→child"
    private var buffers: [String: FrameBuffer] = [:]
    
    // Static transforms (never expire)
    private var staticBuffers: [String: FrameBuffer] = [:]
    
    // Adjacency list for frame tree traversal
    private var adjacency: [String: Set<String>] = [:]
    
    private var tfThrottleRate: Int
    
    public init(
        connection: RosbridgeConnection,
        bufferDuration: Double = 30.0,
        tfThrottleRate: Int = 0  // 0 = no throttle (get every TF)
    ) {
        self.connection = connection
        self.bufferDuration = bufferDuration
        self.tfThrottleRate = tfThrottleRate
    }
    
    /// Start listening to `/tf` and `/tf_static`.
    public func start() {
        connection.subscribe(
            topic: "/tf",
            type: "tf2_msgs/msg/TFMessage",
            throttleRate: tfThrottleRate
        ) { [weak self] msg in
            Task { @MainActor in
                self?.handleTFMessage(msg, isStatic: false)
            }
        }
        
        connection.subscribe(
            topic: "/tf_static",
            type: "tf2_msgs/msg/TFMessage",
            throttleRate: 0
        ) { [weak self] msg in
            Task { @MainActor in
                self?.handleTFMessage(msg, isStatic: true)
            }
        }
        
        isActive = true
    }
    
    /// Stop listening.
    public func stop() {
        connection.unsubscribe(topic: "/tf")
        connection.unsubscribe(topic: "/tf_static")
        isActive = false
    }
    
    // MARK: - Lookup
    
    /// Look up the transform from `child` frame to `parent` frame at the given time.
    ///
    /// For direct connections, interpolates between buffered transforms.
    /// For indirect connections, traverses the frame tree and chains transforms.
    ///
    /// - Parameters:
    ///   - parent: Target frame (e.g., "odom", "map")
    ///   - child: Source frame (e.g., "base_link", "laser")
    ///   - time: ROS timestamp in seconds. Pass 0 for latest.
    /// - Returns: Interpolated transform, or nil if not available.
    public func lookupTransform(parent: String, child: String, time: Double = 0) -> StampedTransform? {
        // Direct lookup
        if let tf = directLookup(parent: parent, child: child, time: time) {
            return tf
        }
        
        // Try inverse
        if let tf = directLookup(parent: child, child: parent, time: time) {
            return invertTransform(tf)
        }
        
        // Tree traversal for indirect connections
        guard let path = findPath(from: child, to: parent) else { return nil }
        return chainTransforms(path: path, time: time)
    }
    
    /// Convenience: get the latest position + heading for a frame relative to a reference.
    public func latestPose(frame: String, relativeTo reference: String = "odom") -> (x: Double, y: Double, heading: Double)? {
        guard let tf = lookupTransform(parent: reference, child: frame, time: 0) else { return nil }
        return (x: tf.translation.x, y: tf.translation.y, heading: tf.rotation.yaw)
    }
    
    // MARK: - Internal
    
    private func handleTFMessage(_ msg: [String: Any], isStatic: Bool) {
        guard let transforms = msg["transforms"] as? [[String: Any]] else { return }
        
        for tfMsg in transforms {
            guard let tf = parseTransformStamped(tfMsg) else { continue }
            
            let key = "\(tf.parent)→\(tf.child)"
            let storage = isStatic ? staticBuffers : buffers
            
            if storage[key] == nil {
                let buf = FrameBuffer(maxAge: isStatic ? 0 : bufferDuration)
                if isStatic {
                    staticBuffers[key] = buf
                } else {
                    buffers[key] = buf
                }
            }
            
            if isStatic {
                staticBuffers[key]?.insert(tf)
            } else {
                buffers[key]?.insert(tf)
            }
            
            // Update adjacency and known frames
            knownFrames.insert(tf.parent)
            knownFrames.insert(tf.child)
            
            if adjacency[tf.parent] == nil { adjacency[tf.parent] = [] }
            if adjacency[tf.child] == nil { adjacency[tf.child] = [] }
            adjacency[tf.parent]?.insert(tf.child)
            adjacency[tf.child]?.insert(tf.parent)
        }
    }
    
    private func parseTransformStamped(_ msg: [String: Any]) -> StampedTransform? {
        guard let header = msg["header"] as? [String: Any],
              let stamp = header["stamp"] as? [String: Any],
              let childFrameId = msg["child_frame_id"] as? String,
              let frameId = (header["frame_id"] as? String),
              let transform = msg["transform"] as? [String: Any],
              let translation = transform["translation"] as? [String: Any],
              let rotation = transform["rotation"] as? [String: Any] else {
            return nil
        }
        
        let sec = (stamp["sec"] as? Double) ?? Double(stamp["sec"] as? Int ?? 0)
        let nsec = (stamp["nanosec"] as? Double) ?? Double(stamp["nanosec"] as? Int ?? 0)
        let timestamp = sec + nsec / 1e9
        
        let tx = (translation["x"] as? Double) ?? 0
        let ty = (translation["y"] as? Double) ?? 0
        let tz = (translation["z"] as? Double) ?? 0
        
        let rx = (rotation["x"] as? Double) ?? 0
        let ry = (rotation["y"] as? Double) ?? 0
        let rz = (rotation["z"] as? Double) ?? 0
        let rw = (rotation["w"] as? Double) ?? 1
        
        return StampedTransform(
            parent: frameId,
            child: childFrameId,
            timestamp: timestamp,
            translation: SIMD3(tx, ty, tz),
            rotation: Quaternion4(x: rx, y: ry, z: rz, w: rw)
        )
    }
    
    private func directLookup(parent: String, child: String, time: Double) -> StampedTransform? {
        let key = "\(parent)→\(child)"
        
        // Check static first
        if let buf = staticBuffers[key], let tf = try? buf.lookup(at: 0) {
            return tf
        }
        
        // Then dynamic
        if let buf = buffers[key], let tf = try? buf.lookup(at: time) {
            return tf
        }
        
        return nil
    }
    
    private func invertTransform(_ tf: StampedTransform) -> StampedTransform {
        let invRotation = tf.rotation.inverse
        let invTranslation = invRotation.rotate(-tf.translation)
        
        return StampedTransform(
            parent: tf.child,
            child: tf.parent,
            timestamp: tf.timestamp,
            translation: invTranslation,
            rotation: invRotation
        )
    }
    
    /// BFS to find shortest path in the frame tree.
    private func findPath(from source: String, to target: String) -> [String]? {
        guard source != target else { return [source] }
        
        var visited: Set<String> = [source]
        var queue: [(node: String, path: [String])] = [(source, [source])]
        
        while !queue.isEmpty {
            let (node, path) = queue.removeFirst()
            
            guard let neighbors = adjacency[node] else { continue }
            for neighbor in neighbors {
                if neighbor == target {
                    return path + [neighbor]
                }
                if !visited.contains(neighbor) {
                    visited.insert(neighbor)
                    queue.append((neighbor, path + [neighbor]))
                }
            }
        }
        
        return nil
    }
    
    /// Chain transforms along a path.
    private func chainTransforms(path: [String], time: Double) -> StampedTransform? {
        guard path.count >= 2 else { return nil }
        
        var result = StampedTransform(
            parent: path[0],
            child: path[0],
            timestamp: time,
            translation: .zero,
            rotation: .identity
        )
        
        for i in 0..<(path.count - 1) {
            let from = path[i]
            let to = path[i + 1]
            
            let tf: StampedTransform
            if let direct = directLookup(parent: to, child: from, time: time) {
                tf = direct
            } else if let inverse = directLookup(parent: from, child: to, time: time) {
                tf = invertTransform(inverse)
            } else {
                return nil
            }
            
            // Compose: result = tf * result
            result = StampedTransform(
                parent: tf.parent,
                child: result.child,
                timestamp: time,
                translation: tf.rotation.rotate(result.translation) + tf.translation,
                rotation: tf.rotation.multiply(result.rotation)
            )
        }
        
        return result
    }
}
