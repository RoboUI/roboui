import XCTest
@testable import RoboUI

final class SLAMEngineTests: XCTestCase {
    func testBasicSLAM() {
        let engine = SLAMEngine(scanSize: 360, mapSizePixels: 200, mapSizeMeters: 10)
        
        var scan = [Int](repeating: 0, count: 360)
        for i in 0..<360 {
            let angle = Double(i) * .pi * 2.0 / 360.0
            let cosA = abs(cos(angle))
            let sinA = abs(sin(angle))
            let distX = cosA > 0.01 ? 2000.0 / cosA : 3500.0
            let distY = sinA > 0.01 ? 1500.0 / sinA : 3500.0
            scan[i] = min(Int(min(distX, distY)), 3500)
        }
        
        // First update
        engine.update(scanDistancesMM: scan, velocity: nil)
        let pos1 = engine.getPosition()
        print("After update 1: pos=(\(pos1.xMM), \(pos1.yMM), \(pos1.thetaDeg)°)")
        
        // Map should have content
        let map = engine.getMap()
        let occupied = map.filter { $0 < 100 }.count
        let free = map.filter { $0 > 200 }.count
        print("Map: \(occupied) occupied, \(free) free")
        XCTAssertGreaterThan(occupied, 0, "Should have occupied cells after scan")
        
        // Multiple updates should refine position
        for _ in 0..<4 {
            engine.update(scanDistancesMM: scan, velocity: (dxyMM: 100, dthetaDeg: 5, dtSec: 0.2))
        }
        let pos2 = engine.getPosition()
        print("After 5 updates: pos=(\(pos2.xMM), \(pos2.yMM), \(pos2.thetaDeg)°)")
        
        // OccupancyGrid
        let grid = engine.buildOccupancyGrid()
        XCTAssertEqual(grid.width, 200)
        XCTAssertEqual(grid.height, 200)
        let wallCells = grid.data.filter { $0 == 100 }.count
        print("OccupancyGrid: \(grid.width)x\(grid.height), walls=\(wallCells)")
        XCTAssertGreaterThan(wallCells, 0)
    }
}
