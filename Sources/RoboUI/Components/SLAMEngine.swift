// RoboUI — SLAMEngine
// Pure Swift port of BreezySLAM (CoreSLAM / tinySLAM).
// Implements RMHC (Random-Mutation Hill-Climbing) SLAM for real-time
// 2D LiDAR-based mapping and localization on iOS/macOS.
//
// Ported from BreezySLAM C core by Simon D. Levy (LGPL v3).
// Original: https://github.com/simondlevy/BreezySLAM

import Foundation
import Accelerate

// MARK: - SLAMEngine

/// Real-time 2D SLAM engine using RMHC (Random-Mutation Hill-Climbing) search.
///
/// Converts LiDAR scans into a corrected robot position and occupancy map.
/// Thread-safe — all mutable state is protected by an internal lock.
///
/// Usage:
/// ```swift
/// let slam = SLAMEngine(scanSize: 360, mapSizePixels: 800, mapSizeMeters: 20)
/// slam.update(scanDistancesMM: distances, velocity: (dxyMM: 0, dthetaDeg: 0, dtSec: 0.2))
/// let pos = slam.getPosition()
/// let grid = slam.buildOccupancyGrid()
/// ```
public final class SLAMEngine: @unchecked Sendable {

    // MARK: - Constants

    fileprivate static let noObstacle: UInt16 = 65500
    fileprivate static let obstacle: UInt16 = 0

    // MARK: - Configuration

    private let scanSize: Int
    private let scanRateHz: Double
    private let detectionAngleDeg: Double
    private let distanceNoDetectionMM: Int
    private let mapSizePixels: Int
    private let mapSizeMeters: Double
    private let mapQuality: Int
    private let holeWidthMM: Double
    private let sigmaXYMM: Double
    private let sigmaThetaDeg: Double
    private let maxSearchIter: Int

    // MARK: - State

    private let lock = NSLock()
    private var mapPixels: [UInt16]
    private let scalePixelsPerMM: Double
    private var position: SLAMPosition
    private var scanForDistance: SLAMScan
    private var scanForMapBuild: SLAMScan
    private var randomizer: ZigguratRNG
    private var updateCount: Int = 0

    // MARK: - Init

    /// Create a SLAM engine.
    /// - Parameters:
    ///   - scanSize: Number of rays per LiDAR scan (default 360).
    ///   - scanRateHz: Scans per second (default 5).
    ///   - detectionAngleDeg: Total detection angle in degrees (default 360).
    ///   - distanceNoDetectionMM: Default distance when laser returns 0 (default 3500).
    ///   - mapSizePixels: Square map dimension in pixels (default 800).
    ///   - mapSizeMeters: Square map dimension in meters (default 20).
    ///   - mapQuality: Integration speed 0–255 (default 50).
    ///   - holeWidthMM: Obstacle width for ray extension (default 600).
    ///   - sigmaXYMM: XY standard deviation for RMHC search (default 100).
    ///   - sigmaThetaDeg: Theta standard deviation for RMHC search (default 20).
    ///   - maxSearchIter: Maximum RMHC iterations (default 1000).
    public init(
        scanSize: Int = 360,
        scanRateHz: Double = 5,
        detectionAngleDeg: Double = 360,
        distanceNoDetectionMM: Int = 3500,
        mapSizePixels: Int = 800,
        mapSizeMeters: Double = 20,
        mapQuality: Int = 50,
        holeWidthMM: Double = 600,
        sigmaXYMM: Double = 100,
        sigmaThetaDeg: Double = 20,
        maxSearchIter: Int = 1000
    ) {
        self.scanSize = scanSize
        self.scanRateHz = scanRateHz
        self.detectionAngleDeg = detectionAngleDeg
        self.distanceNoDetectionMM = distanceNoDetectionMM
        self.mapSizePixels = mapSizePixels
        self.mapSizeMeters = mapSizeMeters
        self.mapQuality = mapQuality
        self.holeWidthMM = holeWidthMM
        self.sigmaXYMM = sigmaXYMM
        self.sigmaThetaDeg = sigmaThetaDeg
        self.maxSearchIter = maxSearchIter

        self.scalePixelsPerMM = Double(mapSizePixels) / (mapSizeMeters * 1000.0)

        // Initialize map to midpoint (unknown)
        let npix = mapSizePixels * mapSizePixels
        let midVal = UInt16((Int(Self.obstacle) + Int(Self.noObstacle)) / 2)
        self.mapPixels = [UInt16](repeating: midVal, count: npix)

        // Position starts at center of map
        let initCoordMM = 500.0 * mapSizeMeters
        self.position = SLAMPosition(xMM: initCoordMM, yMM: initCoordMM, thetaDeg: 0)

        // Two scans: span=1 for distance matching, span=3 for map building
        self.scanForDistance = SLAMScan(
            span: 1, size: scanSize, rateHz: scanRateHz,
            detectionAngleDeg: detectionAngleDeg,
            distanceNoDetectionMM: distanceNoDetectionMM
        )
        self.scanForMapBuild = SLAMScan(
            span: 3, size: scanSize, rateHz: scanRateHz,
            detectionAngleDeg: detectionAngleDeg,
            distanceNoDetectionMM: distanceNoDetectionMM
        )

        // Ziggurat RNG seeded from current time
        let seed = UInt32(UInt64(Date().timeIntervalSince1970 * 1000) & 0xFFFF)
        self.randomizer = ZigguratRNG(seed: seed)
    }

    // MARK: - Public API

    /// Feed a new LiDAR scan and optional odometry velocity.
    /// - Parameters:
    ///   - scanDistancesMM: Array of distance values in millimeters (count must equal scanSize).
    ///   - velocity: Optional odometry change `(dxyMM, dthetaDeg, dtSec)`.
    public func update(
        scanDistancesMM: [Int],
        velocity: (dxyMM: Double, dthetaDeg: Double, dtSec: Double)?
    ) {
        lock.lock()
        defer { lock.unlock() }

        let vel = velocity ?? (dxyMM: 0.0, dthetaDeg: 0.0, dtSec: 0.0)

        // Convert pose change to velocities (per second)
        let factor = vel.dtSec > 0 ? (1.0 / vel.dtSec) : 0.0
        let dxyMMdt = vel.dxyMM * factor
        let dthetaDegDt = vel.dthetaDeg * factor

        // Update both scans
        scanForDistance.update(
            distances: scanDistancesMM, holeWidthMM: holeWidthMM,
            velocityDxyMM: dxyMMdt, velocityDthetaDeg: dthetaDegDt
        )
        scanForMapBuild.update(
            distances: scanDistancesMM, holeWidthMM: holeWidthMM,
            velocityDxyMM: dxyMMdt, velocityDthetaDeg: dthetaDegDt
        )

        // Compute start position from odometry
        var startPos = position
        let thetaRad = startPos.thetaDeg * .pi / 180.0
        startPos.xMM += vel.dxyMM * cos(thetaRad)
        startPos.yMM += vel.dxyMM * sin(thetaRad)
        startPos.thetaDeg += vel.dthetaDeg

        // RMHC search for best position
        let newPos = rmhcPositionSearch(startPos: startPos)

        // Update position
        position = newPos

        // Update map with the map-build scan
        mapUpdate(scan: &scanForMapBuild, position: newPos)

        updateCount += 1
    }

    /// Get the current corrected position.
    /// - Returns: Tuple `(xMM, yMM, thetaDeg)` in map coordinates.
    public func getPosition() -> (xMM: Double, yMM: Double, thetaDeg: Double) {
        lock.lock()
        defer { lock.unlock() }
        return (position.xMM, position.yMM, position.thetaDeg)
    }

    /// Get the current map as bytes.
    /// - Returns: Array of `UInt8` where 0=obstacle, ~127=unknown, ~255=free.
    public func getMap() -> [UInt8] {
        lock.lock()
        defer { lock.unlock() }
        return mapPixels.map { UInt8($0 >> 8) }
    }

    /// Build an `OccupancyGrid` from the current SLAM state for rendering.
    public func buildOccupancyGrid() -> OccupancyGrid {
        lock.lock()
        defer { lock.unlock() }

        let resolutionMeters = Float(mapSizeMeters) / Float(mapSizePixels)
        let originM = -mapSizeMeters / 2.0

        let data: [Int8] = mapPixels.map { pixel in
            let byte = pixel >> 8
            if byte < 80 { return 100 }    // occupied
            if byte > 180 { return 0 }     // free
            return -1                       // unknown
        }

        return OccupancyGrid(
            resolution: resolutionMeters,
            width: mapSizePixels,
            height: mapSizePixels,
            originX: originM,
            originY: originM,
            originYaw: 0,
            frameId: "map",
            data: data
        )
    }

    /// Reset the SLAM engine to initial state.
    public func reset() {
        lock.lock()
        defer { lock.unlock() }

        let npix = mapSizePixels * mapSizePixels
        let midVal = UInt16((Int(Self.obstacle) + Int(Self.noObstacle)) / 2)
        mapPixels = [UInt16](repeating: midVal, count: npix)

        let initCoordMM = 500.0 * mapSizeMeters
        position = SLAMPosition(xMM: initCoordMM, yMM: initCoordMM, thetaDeg: 0)

        scanForDistance = SLAMScan(
            span: 1, size: scanSize, rateHz: scanRateHz,
            detectionAngleDeg: detectionAngleDeg,
            distanceNoDetectionMM: distanceNoDetectionMM
        )
        scanForMapBuild = SLAMScan(
            span: 3, size: scanSize, rateHz: scanRateHz,
            detectionAngleDeg: detectionAngleDeg,
            distanceNoDetectionMM: distanceNoDetectionMM
        )

        let seed = UInt32(UInt64(Date().timeIntervalSince1970 * 1000) & 0xFFFF)
        randomizer = ZigguratRNG(seed: seed)

        updateCount = 0
    }
}

// MARK: - Internal Types

private struct SLAMPosition {
    var xMM: Double
    var yMM: Double
    var thetaDeg: Double
}

private struct SLAMScanPoint {
    var xMM: Double
    var yMM: Double
    var value: UInt16  // OBSTACLE or NO_OBSTACLE
}

private struct SLAMScan {
    let span: Int
    let size: Int
    let rateHz: Double
    let detectionAngleDeg: Double
    let distanceNoDetectionMM: Int

    var points: [SLAMScanPoint] = []
    var npoints: Int = 0

    init(span: Int, size: Int, rateHz: Double,
         detectionAngleDeg: Double, distanceNoDetectionMM: Int) {
        self.span = span
        self.size = size
        self.rateHz = rateHz
        self.detectionAngleDeg = detectionAngleDeg
        self.distanceNoDetectionMM = distanceNoDetectionMM
        self.points = []
        self.npoints = 0
    }

    /// Update scan from raw LiDAR distances with velocity correction.
    mutating func update(
        distances: [Int],
        holeWidthMM: Double,
        velocityDxyMM: Double,
        velocityDthetaDeg: Double
    ) {
        let degreesPerSecond = Double(Int(rateHz * 360))
        let horzMM = velocityDxyMM / degreesPerSecond
        let rotation = 1.0 + velocityDthetaDeg / degreesPerSecond

        points.removeAll(keepingCapacity: true)
        npoints = 0

        let totalSpanPoints = size * span
        let angleRange = detectionAngleDeg

        for i in 1 ..< (size - 1) {
            let lidarValue = i < distances.count ? distances[i] : 0

            if lidarValue == 0 {
                // No obstacle detected — use max range
                appendXY(
                    offset: i, distance: distanceNoDetectionMM,
                    scanVal: SLAMEngine.noObstacle,
                    horzMM: horzMM, rotation: rotation,
                    angleRange: angleRange, totalSpanPoints: totalSpanPoints
                )
            } else if Double(lidarValue) > holeWidthMM / 2.0 {
                // Valid obstacle
                appendXY(
                    offset: i, distance: lidarValue,
                    scanVal: SLAMEngine.obstacle,
                    horzMM: horzMM, rotation: rotation,
                    angleRange: angleRange, totalSpanPoints: totalSpanPoints
                )
            }
        }

        npoints = points.count
    }

    private mutating func appendXY(
        offset: Int, distance: Int, scanVal: UInt16,
        horzMM: Double, rotation: Double,
        angleRange: Double, totalSpanPoints: Int
    ) {
        for j in 0 ..< span {
            let k = Double(offset * span + j) * angleRange / Double(totalSpanPoints - 1)
            let angle = (-angleRange / 2.0 + k * rotation) * .pi / 180.0
            let dist = Double(distance)
            let x = dist * cos(angle) - k * horzMM
            let y = dist * sin(angle)
            points.append(SLAMScanPoint(xMM: x, yMM: y, value: scanVal))
        }
    }
}

// MARK: - Ziggurat RNG

/// Fast Gaussian random number generator using the Ziggurat method.
/// Ported from Marsaglia & Tsang's ziggurat algorithm.
private struct ZigguratRNG {
    private var seed: UInt32
    private var kn: [UInt32]
    private var fn: [Float]
    private var wn: [Float]

    init(seed: UInt32) {
        self.seed = seed == 0 ? 1 : seed
        self.kn = [UInt32](repeating: 0, count: 128)
        self.fn = [Float](repeating: 0, count: 128)
        self.wn = [Float](repeating: 0, count: 128)
        setupNormal()
    }

    private mutating func setupNormal() {
        var dn = 3.442619855899
        let m1 = 2147483648.0
        let vn = 9.91256303526217e-03
        var tn = 3.442619855899

        let q = vn / exp(-0.5 * dn * dn)

        kn[0] = UInt32((dn / q) * m1)
        kn[1] = 0

        wn[0] = Float(q / m1)
        wn[127] = Float(dn / m1)

        fn[0] = 1.0
        fn[127] = Float(exp(-0.5 * dn * dn))

        for i in stride(from: 126, through: 1, by: -1) {
            dn = sqrt(-2.0 * log(vn / dn + exp(-0.5 * dn * dn)))
            kn[i + 1] = UInt32((dn / tn) * m1)
            tn = dn
            fn[i] = Float(exp(-0.5 * dn * dn))
            wn[i] = Float(dn / m1)
        }
    }

    /// SHR3 shift-register generator.
    private mutating func shr3() -> UInt32 {
        let value = seed
        seed = seed ^ (seed << 13)
        seed = seed ^ (seed >> 17)
        seed = seed ^ (seed << 5)
        return value &+ seed
    }

    /// Uniform [0,1) float.
    private mutating func uniform() -> Float {
        let jsrInput = seed
        seed = seed ^ (seed << 13)
        seed = seed ^ (seed >> 17)
        seed = seed ^ (seed << 5)
        let combined = jsrInput &+ seed
        return fmodf(0.5 + Float(combined) / 65536.0 / 65536.0, 1.0)
    }

    /// Standard normal variate (mean 0, variance 1).
    mutating func normal() -> Float {
        let r: Float = 3.442620

        let hz = Int32(bitPattern: shr3())
        let iz = Int(UInt32(bitPattern: hz) & 127)

        if Float(abs(hz)) < Float(kn[iz]) {
            return Float(hz) * wn[iz]
        }

        var currentIz = iz
        var currentHz = hz

        while true {
            if currentIz == 0 {
                var x: Float
                var y: Float
                repeat {
                    x = -0.2904764 * logf(uniform())
                    y = -logf(uniform())
                } while x * x > y + y

                return currentHz <= 0 ? -r - x : r + x
            }

            let x = Float(currentHz) * wn[currentIz]
            if fn[currentIz] + uniform() * (fn[currentIz - 1] - fn[currentIz])
                < expf(-0.5 * x * x) {
                return x
            }

            currentHz = Int32(bitPattern: shr3())
            currentIz = Int(UInt32(bitPattern: currentHz) & 127)

            if Float(abs(currentHz)) < Float(kn[currentIz]) {
                return Float(currentHz) * wn[currentIz]
            }
        }
    }

    /// Normal variate with given mean and standard deviation.
    mutating func normal(mu: Double, sigma: Double) -> Double {
        return mu + sigma * Double(normal())
    }
}

// MARK: - Core Algorithms

private extension SLAMEngine {

    // MARK: distance_scan_to_map

    /// Score how well a scan matches the map at a given position.
    /// Returns sum of map values at scan obstacle points (scaled), or -1 if no points match.
    func distanceScanToMap(scan: SLAMScan, position: SLAMPosition) -> Int {
        let thetaRad = position.thetaDeg * .pi / 180.0
        let cosTheta = cos(thetaRad) * scalePixelsPerMM
        let sinTheta = sin(thetaRad) * scalePixelsPerMM
        let posXPix = position.xMM * scalePixelsPerMM
        let posYPix = position.yMM * scalePixelsPerMM

        var sum: Int64 = 0
        var npoints = 0

        for i in 0 ..< scan.npoints {
            let pt = scan.points[i]
            guard pt.value == Self.obstacle else { continue }

            let x = Int(floor(posXPix + cosTheta * pt.xMM - sinTheta * pt.yMM + 0.5))
            let y = Int(floor(posYPix + sinTheta * pt.xMM + cosTheta * pt.yMM + 0.5))

            if x >= 0, x < mapSizePixels, y >= 0, y < mapSizePixels {
                sum += Int64(mapPixels[y * mapSizePixels + x])
                npoints += 1
            }
        }

        return npoints > 0 ? Int(sum * 1024 / Int64(npoints)) : -1
    }

    // MARK: map_update

    /// Update the map by ray-tracing from robot to each scan point.
    func mapUpdate(scan: inout SLAMScan, position: SLAMPosition) {
        let thetaRad = position.thetaDeg * .pi / 180.0
        let cosTheta = cos(thetaRad)
        let sinTheta = sin(thetaRad)

        let x1 = Self.roundup(position.xMM * scalePixelsPerMM)
        let y1 = Self.roundup(position.yMM * scalePixelsPerMM)

        for i in 0 ..< scan.npoints {
            let pt = scan.points[i]
            var x2p = cosTheta * pt.xMM - sinTheta * pt.yMM
            var y2p = sinTheta * pt.xMM + cosTheta * pt.yMM

            let xp = Self.roundup((position.xMM + x2p) * scalePixelsPerMM)
            let yp = Self.roundup((position.yMM + y2p) * scalePixelsPerMM)

            let dist = sqrt(x2p * x2p + y2p * y2p)
            guard dist > 0 else { continue }
            let add = holeWidthMM / 2.0 / dist

            x2p *= scalePixelsPerMM * (1.0 + add)
            y2p *= scalePixelsPerMM * (1.0 + add)

            let x2 = Self.roundup(position.xMM * scalePixelsPerMM + x2p)
            let y2 = Self.roundup(position.yMM * scalePixelsPerMM + y2p)

            var value = Int(Self.obstacle)
            var q = mapQuality

            if pt.value == Self.noObstacle {
                q = mapQuality / 4
                value = Int(Self.noObstacle)
            }

            mapLaserRay(x1: x1, y1: y1, x2: x2, y2: y2, xp: xp, yp: yp, value: value, alpha: q)
        }
    }

    // MARK: map_laser_ray (Bresenham)

    /// Bresenham ray-cast from (x1,y1) to (x2,y2) through obstacle point (xp,yp).
    /// Updates map pixels with blended values.
    func mapLaserRay(x1: Int, y1: Int, x2: Int, y2: Int, xp: Int, yp: Int, value: Int, alpha: Int) {
        guard !outOfBounds(x1, mapSizePixels), !outOfBounds(y1, mapSizePixels) else { return }

        var x2c = x2
        var y2c = y2

        // Clip the endpoint to map bounds
        if Self.clip(&x2c, &y2c, x1, y1, mapSizePixels)
            || Self.clip(&y2c, &x2c, y1, x1, mapSizePixels) {
            return
        }

        var dx = abs(x2 - x1)
        var dy = abs(y2 - y1)
        var dxc = abs(x2c - x1)
        var dyc = abs(y2c - y1)
        var incptrx = (x2 > x1) ? 1 : -1
        var incptry = (y2 > y1) ? mapSizePixels : -mapSizePixels
        let sincv = value > Int(Self.noObstacle) ? 1 : -1

        var derrorv: Int
        if dx > dy {
            derrorv = abs(xp - x2)
        } else {
            Swift.swap(&dx, &dy)
            Swift.swap(&dxc, &dyc)
            Swift.swap(&incptrx, &incptry)
            derrorv = abs(yp - y2)
        }

        guard derrorv != 0 else { return }

        var error = 2 * dyc - dxc
        let horiz = 2 * dyc
        let diago = 2 * (dyc - dxc)
        var errorv = derrorv / 2

        let incv = (value - Int(Self.noObstacle)) / derrorv
        let incerrorv = value - Int(Self.noObstacle) - derrorv * incv

        var ptrIdx = y1 * mapSizePixels + x1
        var pixval = Int(Self.noObstacle)

        let alpha256 = 256 - alpha

        for x in 0 ... dxc {
            if x > dx - 2 * derrorv {
                if x <= dx - derrorv {
                    pixval += incv
                    errorv += incerrorv
                    if errorv > derrorv {
                        pixval += sincv
                        errorv -= derrorv
                    }
                } else {
                    pixval -= incv
                    errorv -= incerrorv
                    if errorv < 0 {
                        pixval -= sincv
                        errorv += derrorv
                    }
                }
            }

            // Blend into map
            if ptrIdx >= 0, ptrIdx < mapPixels.count {
                mapPixels[ptrIdx] = UInt16(
                    (alpha256 * Int(mapPixels[ptrIdx]) + alpha * pixval) >> 8
                )
            }

            if error > 0 {
                ptrIdx += incptry
                error += diago
            } else {
                error += horiz
            }
            ptrIdx += incptrx
        }
    }

    // MARK: rmhc_position_search

    /// Random-Mutation Hill-Climbing search for the best position.
    func rmhcPositionSearch(startPos: SLAMPosition) -> SLAMPosition {
        var bestPos = startPos
        var lastBestPos = startPos

        var currentDistance = distanceScanToMap(scan: scanForDistance, position: startPos)
        var lowestDistance = currentDistance
        var lastLowestDistance = currentDistance

        var sigmaXY = sigmaXYMM
        var sigmaTheta = sigmaThetaDeg

        var counter = 0

        while counter < maxSearchIter {
            var currentPos = lastBestPos
            currentPos.xMM = randomizer.normal(mu: currentPos.xMM, sigma: sigmaXY)
            currentPos.yMM = randomizer.normal(mu: currentPos.yMM, sigma: sigmaXY)
            currentPos.thetaDeg = randomizer.normal(mu: currentPos.thetaDeg, sigma: sigmaTheta)

            currentDistance = distanceScanToMap(scan: scanForDistance, position: currentPos)

            if currentDistance > -1, currentDistance < lowestDistance {
                lowestDistance = currentDistance
                bestPos = currentPos
            } else {
                counter += 1
            }

            if counter > maxSearchIter / 3 {
                if lowestDistance < lastLowestDistance {
                    lastBestPos = bestPos
                    lastLowestDistance = lowestDistance
                    counter = 0
                    sigmaXY *= 0.5
                    sigmaTheta *= 0.5
                }
            }
        }

        return bestPos
    }

    // MARK: Helpers

    static func roundup(_ x: Double) -> Int {
        Int(floor(x + 0.5))
    }

    func outOfBounds(_ value: Int, _ bound: Int) -> Bool {
        value < 0 || value >= bound
    }

    /// Clip a line endpoint to [0, mapSize). Returns true if the line is entirely outside.
    static func clip(_ xyc: inout Int, _ yxc: inout Int, _ xy: Int, _ yx: Int, _ mapSize: Int) -> Bool {
        if xyc < 0 {
            if xyc == xy { return true }
            yxc += (yxc - yx) * (-xyc) / (xyc - xy)
            xyc = 0
        }
        if xyc >= mapSize {
            if xyc == xy { return true }
            yxc += (yxc - yx) * (mapSize - 1 - xyc) / (xyc - xy)
            xyc = mapSize - 1
        }
        return false
    }
}
