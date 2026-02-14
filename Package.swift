// swift-tools-version: 5.9
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription

let package = Package(
    name: "RoboUI",
    platforms: [
        .iOS(.v16),
        .macOS(.v13)
    ],
    products: [
        .library(
            name: "RoboUI",
            targets: ["RoboUI"]
        ),
    ],
    targets: [
        .target(
            name: "RoboUI",
            dependencies: []
        ),
        .testTarget(
            name: "RoboUITests",
            dependencies: ["RoboUI"]
        ),
    ]
)
