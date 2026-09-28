// swift-tools-version: 6.0
import PackageDescription

let package = Package(
    name: "vfc",
    products: [
        .library(name: "VFC", targets: ["VFC"]),
    ],
    targets: [
        // The interpreter and the virtual board, in C. Always optimised: it
        // runs every instruction of a flight controller's main loop.
        .target(
            name: "VFC",
            cSettings: [
                .unsafeFlags(["-O3", "-ffp-contract=off"]),
            ]
        ),
    ]
)
