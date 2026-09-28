// swift-tools-version: 6.0
import PackageDescription

let package = Package(
    name: "vfc",
    platforms: [.macOS(.v13)],
    products: [
        .library(name: "VirtualFC", targets: ["VirtualFC"]),
        .library(name: "VFC", targets: ["VFC"]),
    ],
    targets: [
        // The interpreter and the virtual board, in C. Always optimised: it
        // runs every instruction of a flight controller's main loop. Floating
        // point contraction stays off so the firmware's separately rounded
        // operations are never fused by the host compiler.
        .target(
            name: "VFC",
            cSettings: [
                .unsafeFlags(["-O3", "-ffp-contract=off"]),
            ]
        ),
        // The Swift interface.
        .target(name: "VirtualFC", dependencies: ["VFC"]),
    ]
)
