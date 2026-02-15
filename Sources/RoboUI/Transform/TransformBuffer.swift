import Foundation

/// Errors from transform lookups.
public enum TransformError: Error, CustomStringConvertible {
    case noData(parent: String, child: String)
    case extrapolation(parent: String, child: String, time: Double, available: ClosedRange<Double>)
    case noPath(from: String, to: String)
    
    public var description: String {
        switch self {
        case .noData(let p, let c):
            return "No transform data between '\(p)' and '\(c)'"
        case .extrapolation(let p, let c, let t, let range):
            return "Transform \(p)→\(c) at t=\(String(format: "%.3f", t)) requires extrapolation (available: \(String(format: "%.3f", range.lowerBound))...\(String(format: "%.3f", range.upperBound)))"
        case .noPath(let from, let to):
            return "No transform path from '\(from)' to '\(to)'"
        }
    }
}

/// A time-indexed buffer of transforms between two specific frames.
///
/// Stores a sorted list of `StampedTransform` values and supports
/// lookup with SLERP/lerp interpolation at arbitrary timestamps.
final class FrameBuffer {
    private var transforms: [StampedTransform] = []
    private let maxAge: Double  // seconds
    
    init(maxAge: Double = 30.0) {
        self.maxAge = maxAge
    }
    
    /// Insert a transform, maintaining sorted order by timestamp.
    func insert(_ tf: StampedTransform) {
        // Fast path: append if newer than last
        if let last = transforms.last, tf.timestamp >= last.timestamp {
            transforms.append(tf)
        } else {
            // Binary search insert
            let idx = transforms.partitioningIndex { $0.timestamp >= tf.timestamp }
            transforms.insert(tf, at: idx)
        }
        
        // Evict old entries
        if maxAge > 0, let newest = transforms.last {
            let cutoff = newest.timestamp - maxAge
            transforms.removeAll { $0.timestamp < cutoff }
        }
    }
    
    /// Lookup transform at a given timestamp with interpolation.
    ///
    /// - If exact match exists, returns it.
    /// - If between two entries, interpolates (lerp + SLERP).
    /// - If timestamp is 0 (static), returns latest.
    func lookup(at timestamp: Double) throws -> StampedTransform {
        guard !transforms.isEmpty else {
            throw TransformError.noData(parent: "", child: "")
        }
        
        // Static transform (t=0): return latest
        if timestamp == 0 {
            return transforms.last!
        }
        
        // Find bracketing entries
        let idx = transforms.partitioningIndex { $0.timestamp >= timestamp }
        
        // Exact or beyond range
        if idx < transforms.count && abs(transforms[idx].timestamp - timestamp) < 1e-6 {
            return transforms[idx]
        }
        
        // Before all data
        if idx == 0 {
            // Allow small extrapolation (50ms) for network jitter
            let first = transforms[0]
            if timestamp >= first.timestamp - 0.05 {
                return first
            }
            throw TransformError.extrapolation(
                parent: transforms[0].parent,
                child: transforms[0].child,
                time: timestamp,
                available: transforms.first!.timestamp...transforms.last!.timestamp
            )
        }
        
        // After all data
        if idx == transforms.count {
            let last = transforms.last!
            // Allow small extrapolation (50ms) for network jitter
            if timestamp <= last.timestamp + 0.05 {
                return last
            }
            throw TransformError.extrapolation(
                parent: last.parent,
                child: last.child,
                time: timestamp,
                available: transforms.first!.timestamp...last.timestamp
            )
        }
        
        // Interpolate between idx-1 and idx
        let before = transforms[idx - 1]
        let after = transforms[idx]
        let t = (timestamp - before.timestamp) / (after.timestamp - before.timestamp)
        
        return StampedTransform(
            parent: before.parent,
            child: before.child,
            timestamp: timestamp,
            translation: lerp(before.translation, after.translation, t: t),
            rotation: before.rotation.slerp(to: after.rotation, t: t)
        )
    }
    
    /// Latest timestamp in buffer, or nil.
    var latestTimestamp: Double? {
        transforms.last?.timestamp
    }
    
    /// Number of entries in buffer.
    var count: Int { transforms.count }
}

// MARK: - Array binary search helper

private extension Array {
    /// Returns the index of the first element matching the predicate (like C++ lower_bound).
    func partitioningIndex(where predicate: (Element) -> Bool) -> Int {
        var lo = startIndex
        var hi = endIndex
        while lo < hi {
            let mid = lo + (hi - lo) / 2
            if predicate(self[mid]) {
                hi = mid
            } else {
                lo = mid + 1
            }
        }
        return lo
    }
}
