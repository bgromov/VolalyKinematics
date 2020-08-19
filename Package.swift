// swift-tools-version:5.2
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription

let package = Package(
    name: "VolalyKinematics",
    platforms: [.iOS(.v13)],
    products: [
        .library(name: "VolalyKinematics", targets: ["VolalyKinematics"]),
    ],
    dependencies: [
         .package(url: "https://github.com/bgromov/TransformSwift.git", from: "0.1.0"),
    ],
    targets: [
        .target(
            name: "VolalyKinematics",
            dependencies: [.product(name: "Transform", package: "TransformSwift")]),
    ]
)
