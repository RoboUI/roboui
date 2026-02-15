// RoboUI — SLAMEngine
// Pure Swift SLAM engine using RMHC (Random-Mutation Hill-Climbing) search
// with log-odds occupancy grid and distance-weighted inverse sensor model.
//
// Based on BreezySLAM/CoreSLAM concepts, but with proper probabilistic
// map update (Thrun, Burgard, Fox — "Probabilistic Robotics").
//
// Key improvement over CoreSLAM: distance-weighted updates prevent
// far-away observations from corrupting close-range wall measurements.

import Foundation

// MARK: - SLAMEngine

/// Real-time 2D SLAM engine for LiDAR-based mapping and localization.
///
/// Uses RMHC position search (scan-to-map matching) and log-odds occupancy
/// grid with distance-weighted inverse sensor model.
///
/// Thread-safe — all mutable state is protected by an internal lock.
public final class SLAMEngine: @unchecked Sendable {

    // MARK: - Log-Odds Constants

    /// Log-odds update for occupied cells (strong positive).
    /// p(occ) ≈ 0.7 → log-odds = ln(0.7/0.3) ≈ 0.85
    private static let logOccBase: Float = 0.85

    /// Log-odds update for free cells (moderate negative).
    /// p(free) ≈ 0.35 → log-odds = ln(0.35/0.65) ≈ -0.62
    private static let logFreeBase: Float = -0.62

    /// Maximum absolute log-odds value. Prevents over-commitment.
    /// ±5.0 → p ≈ 0.993 / 0.007 — very confident but not irreversible.
    private static let logOddsClamp: Float = 5.0

    /// Minimum distance weight. Even far observations contribute slightly.
    private static let minDistanceWeight: Float = 0.05

    // MARK: - Scan Matching Constants (for RMHC compatibility)

    /// Value representing obstacle in scan point classification.
    fileprivate static let obstacle: UInt16 = 0
    /// Value representing no obstacle (free/max-range).
    fileprivate static let noObstacle: UInt16 = 65500

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

    /// Log-odds occupancy map. 0 = unknown, positive = occupied, negative = free.
    private var logOddsMap: [Float]

    private let scalePixelsPerMM: Double
    private var position: SLAMPosition
    private var scanForDistance: SLAMScan
    private var scanForMapBuild: SLAMScan
    private var randomizer: ZigguratRNG
    private var updateCount: Int = 0

    // MARK: - Init

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

        // Initialize log-odds map to 0 (unknown)
        let npix = mapSizePixels * mapSizePixels
        self.logOddsMap = [Float](repeating: 0, count: npix)

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
            span: 1, size: scanSize, rateHz: scanRateHz,
            detectionAngleDeg: detectionAngleDeg,
            distanceNoDetectionMM: distanceNoDetectionMM
        )

        let seed = UInt32(UInt64(Date().timeIntervalSince1970 * 1000) & 0xFFFF)
        self.randomizer = ZigguratRNG(seed: seed)
    }

    // MARK: - Public API

    /// Feed a new LiDAR scan and odometry delta in SLAM map frame.
    ///
    /// - Parameters:
    ///   - scanDistancesMM: LiDAR ranges in mm (0 = no detection).
    ///   - odomDelta: Odometry displacement in SLAM map coordinates (dxMM, dyMM, dthetaDeg).
    ///     Pass nil if no odometry available (first scan or lost).
    public func update(
        scanDistancesMM: [Int],
        odomDelta: (dxMM: Double, dyMM: Double, dthetaDeg: Double)?
    ) {
        lock.lock()
        defer { lock.unlock() }

        let delta = odomDelta ?? (dxMM: 0.0, dyMM: 0.0, dthetaDeg: 0.0)

        // For scan motion compensation, derive forward velocity and angular rate.
        // dtSec=0.2 at 5Hz. Forward velocity ≈ magnitude of displacement.
        let dtSec = 1.0 / scanRateHz
        let dxyMM = hypot(delta.dxMM, delta.dyMM)
        let dxyMMdt = dxyMM / dtSec
        let dthetaDegDt = delta.dthetaDeg / dtSec

        // Update both scans
        scanForDistance.update(
            distances: scanDistancesMM, holeWidthMM: holeWidthMM,
            velocityDxyMM: dxyMMdt, velocityDthetaDeg: dthetaDegDt
        )
        scanForMapBuild.update(
            distances: scanDistancesMM, holeWidthMM: holeWidthMM,
            velocityDxyMM: dxyMMdt, velocityDthetaDeg: dthetaDegDt
        )

        // Predict position directly from odom dx/dy (no heading-based projection)
        var odomPos = position
        odomPos.xMM += delta.dxMM
        odomPos.yMM += delta.dyMM
        odomPos.thetaDeg += delta.dthetaDeg

        // RMHC search for best position
        let rmhcPos = rmhcPositionSearch(startPos: odomPos)

        // Match quality gate: if RMHC result is poor, trust odometry instead.
        // This prevents the robot from "jumping behind walls" when scan matching fails.
        let odomDist = distanceScanToMap(scan: scanForDistance, position: odomPos)
        let rmhcDist = distanceScanToMap(scan: scanForDistance, position: rmhcPos)

        let newPos: SLAMPosition
        let usedOdomFallback: Bool
        if rmhcDist >= 0 && (odomDist < 0 || rmhcDist <= odomDist) {
            newPos = rmhcPos
            usedOdomFallback = false
        } else {
            newPos = odomPos
            usedOdomFallback = true
        }
        // Clamp position to map bounds (prevent off-map drift)
        let margin = 20.0  // mm from edge
        let mapExtentMM = mapSizeMeters * 1000.0
        var clampedPos = newPos
        clampedPos.xMM = max(margin, min(mapExtentMM - margin, clampedPos.xMM))
        clampedPos.yMM = max(margin, min(mapExtentMM - margin, clampedPos.yMM))
        position = clampedPos

        // Count obstacle points in current scan
        let obstacleCount = scanForDistance.points.prefix(scanForDistance.npoints)
            .filter { $0.value == Self.obstacle }.count
        let maxRangeCount = scanForDistance.points.prefix(scanForDistance.npoints)
            .filter { $0.value == Self.noObstacle }.count

        // Detailed logging every 10 scans
        if updateCount % 10 == 0 {
            let posPx = (Int(newPos.xMM * scalePixelsPerMM), Int(newPos.yMM * scalePixelsPerMM))
            let fb = usedOdomFallback ? " FALLBACK" : ""
            print(String(format: "[SLAM #%d] pos=(%.0f,%.0f) px=(%d,%d) θ=%.1f° pts=%d/%d rmhc=%d odom=%d%@",
                         updateCount,
                         newPos.xMM, newPos.yMM, posPx.0, posPx.1,
                         newPos.thetaDeg,
                         obstacleCount, obstacleCount + maxRangeCount,
                         rmhcDist, odomDist, fb))
        }

        // Update map with log-odds
        mapUpdateLogOdds(scan: &scanForMapBuild, position: newPos)

        updateCount += 1
    }

    /// Set the initial heading before the first update.
    public func setInitialHeading(thetaDeg: Double) {
        lock.lock()
        defer { lock.unlock() }
        position.thetaDeg = thetaDeg
    }

    /// Get the current corrected position.
    public func getPosition() -> (xMM: Double, yMM: Double, thetaDeg: Double) {
        lock.lock()
        defer { lock.unlock() }
        return (position.xMM, position.yMM, position.thetaDeg)
    }

    /// Get the current map as bytes (0=obstacle, ~127=unknown, ~255=free).
    public func getMap() -> [UInt8] {
        lock.lock()
        defer { lock.unlock() }
        return logOddsMap.map { logOdds in
            // Convert log-odds [-5..+5] → byte [255..0]
            // Positive (occupied) → low value (dark), negative (free) → high value (bright)
            let normalized = (-logOdds / Self.logOddsClamp + 1.0) * 0.5  // [0..1]
            return UInt8(max(0, min(255, Int(normalized * 255))))
        }
    }

    /// Build an `OccupancyGrid` from the current SLAM state for rendering.
    public func buildOccupancyGrid() -> OccupancyGrid {
        lock.lock()
        defer { lock.unlock() }

        let resolutionMeters = Float(mapSizeMeters) / Float(mapSizePixels)
        let originM = -mapSizeMeters / 2.0

        var wallCount = 0
        var protectedCount = 0
        var freeCount = 0
        var maxLogOdds: Float = -999
        var minLogOdds: Float = 999

        // Build data: only flip Y (SLAM Y-down → ROS Y-up). X stays same direction.
        let sz = mapSizePixels
        var data = [Int8](repeating: -1, count: sz * sz)
        for y in 0 ..< sz {
            let flippedY = sz - 1 - y  // flip Y only
            for x in 0 ..< sz {
                let logOdds = logOddsMap[y * sz + x]
                
                if logOdds > maxLogOdds { maxLogOdds = logOdds }
                if logOdds < minLogOdds { minLogOdds = logOdds }
                
                let val: Int8
                if logOdds > 0.5 {
                    wallCount += 1
                    if logOdds > 2.0 { protectedCount += 1 }
                    val = 100
                } else if logOdds < -0.5 {
                    freeCount += 1
                    val = 0
                } else {
                    val = -1
                }
                data[flippedY * sz + x] = val  // x unchanged
            }
        }

        if updateCount % 50 == 0 {
            print(String(format: "[MAP] walls=%d (protected=%d) free=%d logOdds=[%.1f, %.1f] updates=%d",
                         wallCount, protectedCount, freeCount, minLogOdds, maxLogOdds, updateCount))

            // Check map corners (SLAM pixel coords) for wall data
            // Map is 800px = 12m. Room corners at ±2.5m from center.
            // Center = pixel 400. ±2.5m = ±167px. So corners at ~233,233 / 567,233 / 233,567 / 567,567
            let cornerZones = [
                ("TL(x-,y-)", 233, 233),  // SLAM top-left = world (x-, y+)
                ("TR(x+,y-)", 567, 233),  // SLAM top-right = world (x+, y+)
                ("BL(x-,y+)", 233, 567),  // SLAM bottom-left = world (x-, y-)
                ("BR(x+,y+)", 567, 567),  // SLAM bottom-right = world (x+, y-)
            ]
            var cornerInfo: [String] = []
            for (name, cx, cy) in cornerZones {
                // Check 20x20 pixel area around corner
                var wallPx = 0, freePx = 0, unkPx = 0
                var maxLO: Float = -10, minLO: Float = 10
                for dy in -10...10 {
                    for dx in -10...10 {
                        let px = cx + dx, py = cy + dy
                        if px >= 0, px < sz, py >= 0, py < sz {
                            let lo = logOddsMap[py * sz + px]
                            if lo > maxLO { maxLO = lo }
                            if lo < minLO { minLO = lo }
                            if lo > 0.5 { wallPx += 1 }
                            else if lo < -0.5 { freePx += 1 }
                            else { unkPx += 1 }
                        }
                    }
                }
                cornerInfo.append(String(format: "%@:w%d/f%d/u%d[%.1f,%.1f]",
                                         name, wallPx, freePx, unkPx, minLO, maxLO))
            }
            print("[MAP corners] \(cornerInfo.joined(separator: "  "))")
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
        logOddsMap = [Float](repeating: 0, count: npix)

        let initCoordMM = 500.0 * mapSizeMeters
        position = SLAMPosition(xMM: initCoordMM, yMM: initCoordMM, thetaDeg: 0)

        scanForDistance = SLAMScan(
            span: 1, size: scanSize, rateHz: scanRateHz,
            detectionAngleDeg: detectionAngleDeg,
            distanceNoDetectionMM: distanceNoDetectionMM
        )
        scanForMapBuild = SLAMScan(
            span: 1, size: scanSize, rateHz: scanRateHz,
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
    var value: UInt16
    var distanceMM: Double  // distance from robot to this point
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
    }

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
                appendXY(
                    offset: i, distance: distanceNoDetectionMM,
                    scanVal: SLAMEngine.noObstacle,
                    horzMM: horzMM, rotation: rotation,
                    angleRange: angleRange, totalSpanPoints: totalSpanPoints
                )
            } else if Double(lidarValue) > holeWidthMM / 2.0 {
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
            // Negate Y: scan uses math convention (Y-up) but SLAM map is Y-down.
            // Without negation, "left" in robot frame maps to SLAM Y+ (down) = world Y-.
            let y = -dist * sin(angle)
            points.append(SLAMScanPoint(xMM: x, yMM: y, value: scanVal, distanceMM: dist))
        }
    }
}

// MARK: - Ziggurat RNG

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

    private mutating func shr3() -> UInt32 {
        let value = seed
        seed = seed ^ (seed << 13)
        seed = seed ^ (seed >> 17)
        seed = seed ^ (seed << 5)
        return value &+ seed
    }

    private mutating func uniform() -> Float {
        let jsrInput = seed
        seed = seed ^ (seed << 13)
        seed = seed ^ (seed >> 17)
        seed = seed ^ (seed << 5)
        let combined = jsrInput &+ seed
        return fmodf(0.5 + Float(combined) / 65536.0 / 65536.0, 1.0)
    }

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

    mutating func normal(mu: Double, sigma: Double) -> Double {
        return mu + sigma * Double(normal())
    }
}

// MARK: - Core Algorithms

private extension SLAMEngine {

    // MARK: - Scan-to-Map Distance (for RMHC position search)

    /// Score how well a scan matches the map at a given position.
    /// Returns a distance metric (lower = better match). -1 if no points match.
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
                // Convert log-odds to pixel-equivalent for matching:
                // occupied (+5) → low value (good match), free (-5) → high value (bad match)
                let logOdds = logOddsMap[y * mapSizePixels + x]
                let pixelEquiv = Int(32768.0 - Double(logOdds) * 6000.0)
                sum += Int64(max(0, min(65535, pixelEquiv)))
                npoints += 1
            }
        }

        return npoints > 0 ? Int(sum * 1024 / Int64(npoints)) : -1
    }

    // MARK: - Log-Odds Map Update

    /// Update map using log-odds with distance-weighted inverse sensor model.
    ///
    /// For each scan ray:
    /// - Cells along the ray before the hit → free update (weighted by distance)
    /// - Cell at the hit point → occupied update (weighted by distance)
    /// - Cells beyond the hit point → not updated (no information)
    ///
    /// Close-range observations get high weight, far observations get low weight.
    /// Log-odds are clamped so confident measurements can't be easily overwritten.
    func mapUpdateLogOdds(scan: inout SLAMScan, position: SLAMPosition) {
        let thetaRad = position.thetaDeg * .pi / 180.0
        let cosTheta = cos(thetaRad)
        let sinTheta = sin(thetaRad)

        let robotPixX = Int(position.xMM * scalePixelsPerMM + 0.5)
        let robotPixY = Int(position.yMM * scalePixelsPerMM + 0.5)

        guard robotPixX >= 0, robotPixX < mapSizePixels,
              robotPixY >= 0, robotPixY < mapSizePixels else { return }

        let maxRangeMM = Double(distanceNoDetectionMM)

        for i in 0 ..< scan.npoints {
            let pt = scan.points[i]
            let isObstacle = pt.value == Self.obstacle

            // Scan point in world frame
            let wx = cosTheta * pt.xMM - sinTheta * pt.yMM
            let wy = sinTheta * pt.xMM + cosTheta * pt.yMM

            // Distance-based weight: quadratic falloff
            // Close range → weight ≈ 1.0, far range → weight → minWeight
            let distRatio = Float(pt.distanceMM / maxRangeMM)
            let weight = max(Self.minDistanceWeight, 1.0 - distRatio * distRatio)

            // Endpoint in pixel coordinates
            let endPixX = Int((position.xMM + wx) * scalePixelsPerMM + 0.5)
            let endPixY = Int((position.yMM + wy) * scalePixelsPerMM + 0.5)

            // Bresenham ray-cast with log-odds update
            bresenhamLogOdds(
                x0: robotPixX, y0: robotPixY,
                x1: endPixX, y1: endPixY,
                isObstacle: isObstacle,
                weight: weight
            )
        }
    }

    /// Bresenham ray-cast that updates log-odds along the ray.
    ///
    /// Key protection: cells already confidently marked as occupied (logOdds > wallProtection)
    /// are NOT degraded by free-ray updates. This prevents far-away observations from
    /// erasing walls that were accurately mapped at close range.
    func bresenhamLogOdds(
        x0: Int, y0: Int, x1: Int, y1: Int,
        isObstacle: Bool, weight: Float
    ) {
        let freeUpdate = Self.logFreeBase * weight
        let occUpdate = Self.logOccBase * weight
        let clampHi = Self.logOddsClamp
        let clampLo = -Self.logOddsClamp
        let sz = mapSizePixels

        // Confident wall threshold: don't apply free updates to cells above this.
        // logOccBase=0.85, so 3 close hits → 2.55, protected after ~3 scans.
        // This prevents heading-drift free rays from erasing walls.
        let wallProtection: Float = 2.0

        var x = x0, y = y0
        let dx = abs(x1 - x0), dy = abs(y1 - y0)
        let sx = x0 < x1 ? 1 : -1
        let sy = y0 < y1 ? 1 : -1
        var err = dx - dy

        let totalSteps = max(dx, dy)
        // Mark a small zone at the endpoint as occupied.
        // 1px leaves gaps between rays at medium range; 2px fills them.
        let obstThickness = 2
        var step = 0

        while true {
            if x >= 0, x < sz, y >= 0, y < sz {
                let idx = y * sz + x
                let atEndpoint = (x == x1 && y == y1)
                let nearEndpoint = (totalSteps - step) <= obstThickness

                if atEndpoint || nearEndpoint {
                    if isObstacle {
                        // Occupied update — always apply (wall confirmation)
                        logOddsMap[idx] = min(clampHi, logOddsMap[idx] + occUpdate)
                    } else {
                        // Max-range free ray endpoint
                        if logOddsMap[idx] < wallProtection {
                            logOddsMap[idx] = max(clampLo, logOddsMap[idx] + freeUpdate)
                        }
                    }
                } else {
                    // Free space update — but protect confident walls
                    if logOddsMap[idx] < wallProtection {
                        logOddsMap[idx] = max(clampLo, logOddsMap[idx] + freeUpdate)
                    }
                }
            }

            if x == x1 && y == y1 { break }

            let e2 = 2 * err
            if e2 > -dy { err -= dy; x += sx }
            if e2 < dx { err += dx; y += sy }
            step += 1
        }
    }

    // MARK: - RMHC Position Search

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
}
