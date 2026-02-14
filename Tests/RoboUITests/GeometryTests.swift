import XCTest
@testable import RoboUI

final class GeometryTests: XCTestCase {
    
    func testTwistDict() {
        let twist = Twist(
            linear: Vector3(x: 0.5, y: 0, z: 0),
            angular: Vector3(x: 0, y: 0, z: 1.0)
        )
        
        let dict = twist.dict
        let linear = dict["linear"] as! [String: Any]
        let angular = dict["angular"] as! [String: Any]
        
        XCTAssertEqual(linear["x"] as! Double, 0.5)
        XCTAssertEqual(angular["z"] as! Double, 1.0)
    }
    
    func testVector3Defaults() {
        let v = Vector3()
        XCTAssertEqual(v.x, 0)
        XCTAssertEqual(v.y, 0)
        XCTAssertEqual(v.z, 0)
    }
    
    func testTwistDefaults() {
        let t = Twist()
        XCTAssertEqual(t.linear.x, 0)
        XCTAssertEqual(t.angular.z, 0)
    }
}
