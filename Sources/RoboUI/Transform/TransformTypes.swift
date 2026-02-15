import Foundation

/// A stamped 3D rigid-body transform between two coordinate frames.
///
/// Follows ROS2 tf2 conventions: transform goes from `child` frame to `parent` frame.
/// Inspired by the Rust `transforms` crate API.
public struct StampedTransform {
    public let parent: String
    public let child: String
    public let timestamp: Double  // seconds (ROS time)
    public let translation: SIMD3<Double>
    public let rotation: Quaternion4  // (x, y, z, w)
    
    public init(
        parent: String,
        child: String,
        timestamp: Double,
        translation: SIMD3<Double> = .zero,
        rotation: Quaternion4 = .identity
    ) {
        self.parent = parent
        self.child = child
        self.timestamp = timestamp
        self.translation = translation
        self.rotation = rotation
    }
}

/// Quaternion for 3D rotations (x, y, z, w convention).
public struct Quaternion4: Equatable {
    public let x: Double
    public let y: Double
    public let z: Double
    public let w: Double
    
    public static let identity = Quaternion4(x: 0, y: 0, z: 0, w: 1)
    
    public init(x: Double, y: Double, z: Double, w: Double) {
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    }
    
    /// Yaw angle (rotation around Z axis) in radians, range [-π, π].
    public var yaw: Double {
        atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    }
    
    /// Spherical Linear Interpolation (SLERP) between two quaternions.
    ///
    /// Standard algorithm from the tf2 paper — ensures constant angular velocity interpolation.
    public func slerp(to other: Quaternion4, t: Double) -> Quaternion4 {
        // Compute dot product
        var dot = x * other.x + y * other.y + z * other.z + w * other.w
        
        // If dot is negative, negate one quaternion to take shorter path
        var target = other
        if dot < 0 {
            target = Quaternion4(x: -other.x, y: -other.y, z: -other.z, w: -other.w)
            dot = -dot
        }
        
        // If quaternions are very close, use linear interpolation to avoid division by zero
        if dot > 0.9995 {
            return Quaternion4(
                x: x + t * (target.x - x),
                y: y + t * (target.y - y),
                z: z + t * (target.z - z),
                w: w + t * (target.w - w)
            ).normalized
        }
        
        let theta0 = acos(dot)
        let theta = theta0 * t
        let sinTheta = sin(theta)
        let sinTheta0 = sin(theta0)
        
        let s0 = cos(theta) - dot * sinTheta / sinTheta0
        let s1 = sinTheta / sinTheta0
        
        return Quaternion4(
            x: s0 * x + s1 * target.x,
            y: s0 * y + s1 * target.y,
            z: s0 * z + s1 * target.z,
            w: s0 * w + s1 * target.w
        ).normalized
    }
    
    /// Normalize quaternion to unit length.
    public var normalized: Quaternion4 {
        let len = sqrt(x * x + y * y + z * z + w * w)
        guard len > 1e-10 else { return .identity }
        return Quaternion4(x: x / len, y: y / len, z: z / len, w: w / len)
    }
    
    /// Multiply two quaternions (compose rotations).
    public func multiply(_ other: Quaternion4) -> Quaternion4 {
        Quaternion4(
            x: w * other.x + x * other.w + y * other.z - z * other.y,
            y: w * other.y - x * other.z + y * other.w + z * other.x,
            z: w * other.z + x * other.y - y * other.x + z * other.w,
            w: w * other.w - x * other.x - y * other.y - z * other.z
        )
    }
    
    /// Inverse of this quaternion (conjugate for unit quaternions).
    public var inverse: Quaternion4 {
        Quaternion4(x: -x, y: -y, z: -z, w: w)
    }
    
    /// Rotate a 3D vector by this quaternion.
    public func rotate(_ v: SIMD3<Double>) -> SIMD3<Double> {
        let qv = SIMD3(x, y, z)
        let uv = cross(qv, v)
        let uuv = cross(qv, uv)
        return v + 2.0 * (w * uv + uuv)
    }
}

/// Linear interpolation for SIMD3.
func lerp(_ a: SIMD3<Double>, _ b: SIMD3<Double>, t: Double) -> SIMD3<Double> {
    a + t * (b - a)
}

/// Cross product for SIMD3.
private func cross(_ a: SIMD3<Double>, _ b: SIMD3<Double>) -> SIMD3<Double> {
    SIMD3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    )
}
