// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See LICENSE.

import Foundation
import VFC

/// A virtual flight controller: firmware built for the virtual board, run by
/// the interpreter in lockstep with the host.
///
/// Time is the host's. The firmware runs until it has nothing to do before
/// some future time and sleeps; `advance(to:)` wakes it whenever that time
/// arrives, until the requested time is reached.
public final class VirtualFlightController {

    public enum LoadError: Error, Equatable {
        /// Not an image for the virtual board.
        case notAnImage
        /// Built for a different mailbox ABI than this library implements.
        case abiMismatch
    }

    public enum RunError: Error, Equatable {
        case fault(String)
        /// The firmware asked for a reset (after `save`, or `exit` from the CLI).
        case resetRequested
    }

    private let handle: OpaquePointer
    public let firmwareVersion: String

    /// Instructions to run before giving up on the firmware ever sleeping.
    public var budget: UInt64 = 200_000_000

    public init(firmware: Data) throws {
        guard let handle = vfc_create() else { throw LoadError.notAnImage }
        self.handle = handle
        let status = firmware.withUnsafeBytes { raw in
            vfc_load(handle, raw.bindMemory(to: UInt8.self).baseAddress, raw.count, 0x0800_0000)
        }
        switch status {
        case VFC_OK: break
        case VFC_ERR_ABI:
            vfc_destroy(handle)
            throw LoadError.abiMismatch
        default:
            vfc_destroy(handle)
            throw LoadError.notAnImage
        }
        firmwareVersion = String(cString: vfc_firmware_version(handle))
    }

    deinit {
        vfc_destroy(handle)
    }

    // MARK: Time

    /// Nanoseconds since power-on.
    public var timeNanoseconds: UInt64 { vfc_time_ns(handle) }

    /// Runs the firmware until time reaches `nanoseconds`.
    public func advance(to nanoseconds: UInt64) throws {
        let stop = vfc_advance_to(handle, nanoseconds, budget)
        try check(stop)
    }

    public func advance(by nanoseconds: UInt64) throws {
        try advance(to: timeNanoseconds + nanoseconds)
    }

    /// Power cycles the board, keeping its config storage.
    public func reset() {
        vfc_reset(handle)
    }

    private func check(_ stop: vfc_stop_t) throws {
        switch stop {
        case VFC_STOP_IDLE:
            return
        case VFC_STOP_RESET:
            throw RunError.resetRequested
        case VFC_STOP_FAULT:
            throw RunError.fault(vfc_fault(handle).map { String(cString: $0) } ?? "fault")
        default:
            throw RunError.fault("the firmware ran \(budget) instructions without sleeping (pc 0x\(String(vfc_pc(handle), radix: 16)))")
        }
    }

    /// 0 at reset, 3 once the scheduler is running.
    public var stage: UInt32 { vfc_stage(handle) }
    public var instructions: UInt64 { vfc_instructions(handle) }

    /// Native code for the firmware (Apple silicon), rather than the
    /// interpreter: the same results, about ten times faster. On by default
    /// where available; `VFC_JIT=0` in the environment turns it off.
    public var usesJIT: Bool {
        get { vfc_jit_enabled(handle) }
        set { vfc_set_jit(handle, newValue) }
    }

    // MARK: Inputs

    /// One gyro and accelerometer sample in sensor counts: 16.4 per deg/s for
    /// the gyro, `acc_1G` per g for the accelerometer.
    public func post(gyro: (Int16, Int16, Int16), acc: (Int16, Int16, Int16)) {
        var g = [gyro.0, gyro.1, gyro.2]
        var a = [acc.0, acc.1, acc.2]
        vfc_post_sensor(handle, &g, &a)
    }

    /// One RC frame, channel values in microseconds.
    public func post(rc channels: [UInt16]) {
        channels.withUnsafeBufferPointer { vfc_post_rc(handle, $0.baseAddress, Int32($0.count)) }
    }

    public func setBattery(millivolts: UInt32, milliamps: UInt32) {
        vfc_set_battery(handle, millivolts, milliamps)
    }

    /// A motor's speed as bidirectional DShot reports it, in eRPM / 100: the
    /// unit a blackbox log records.
    public func setMotorSpeed(motor: Int, erpm100: UInt32) {
        vfc_set_erpm100(handle, Int32(motor), erpm100)
    }

    // MARK: Outputs

    public var motors: [Float] {
        (0..<Int(vfc_motor_count(handle))).map { vfc_motor(handle, Int32($0)) }
    }

    /// Incremented by the firmware after each motor update.
    public var motorUpdates: UInt32 { vfc_motor_seq(handle) }

    // MARK: Byte streams

    public func writeSerial(_ data: Data) {
        data.withUnsafeBytes { raw in
            vfc_serial_write(handle, raw.bindMemory(to: UInt8.self).baseAddress, raw.count)
        }
    }

    public func readSerial() -> Data {
        var out = Data()
        var buffer = [UInt8](repeating: 0, count: 4096)
        while true {
            let n = vfc_serial_read(handle, &buffer, buffer.count)
            if n == 0 { break }
            out.append(buffer, count: n)
        }
        return out
    }

    /// Blackbox log bytes written since the last call.
    public func readBlackbox() -> Data {
        let pending = vfc_blackbox_pending(handle)
        guard pending > 0 else { return Data() }
        var out = Data(count: pending)
        let n = out.withUnsafeMutableBytes { raw in
            vfc_blackbox_read(handle, raw.bindMemory(to: UInt8.self).baseAddress, pending)
        }
        return out.prefix(n)
    }

    public var blackboxLogs: UInt32 { vfc_blackbox_logs(handle) }
    public var isLogging: Bool { vfc_blackbox_logging(handle) }

    // MARK: Config storage

    /// The board's flash: the saved config, kept across resets.
    public var configImage: Data {
        get {
            var out = Data(count: vfc_config_size(handle))
            let count = out.count
            _ = out.withUnsafeMutableBytes { raw in
                vfc_config_read(handle, raw.bindMemory(to: UInt8.self).baseAddress, count)
            }
            return out
        }
        set {
            newValue.withUnsafeBytes { raw in
                vfc_config_write(handle, raw.bindMemory(to: UInt8.self).baseAddress, raw.count)
            }
        }
    }

    /// Text the firmware wrote to its debug console.
    public func readConsole() -> String {
        var buffer = [CChar](repeating: 0, count: 4096)
        let n = vfc_console_read(handle, &buffer, buffer.count)
        return String(decoding: buffer.prefix(n).map { UInt8(bitPattern: $0) }, as: UTF8.self)
    }

    // MARK: Snapshots

    /// The whole board at this moment: core, RAM, mailbox and time. Pending
    /// serial and blackbox output are not included. Only valid for the same
    /// image and the same build of this library.
    public func snapshot() -> Data {
        var out = Data(count: vfc_snapshot_size(handle))
        out.withUnsafeMutableBytes { raw in
            vfc_snapshot(handle, raw.bindMemory(to: UInt8.self).baseAddress)
        }
        return out
    }

    /// Puts the board back as a snapshot of it (or of another board running
    /// the same image) left it.
    public func restore(_ snapshot: Data) throws {
        let status = snapshot.withUnsafeBytes { raw in
            vfc_restore(handle, raw.bindMemory(to: UInt8.self).baseAddress, raw.count)
        }
        guard status == VFC_OK else { throw LoadError.notAnImage }
    }
}
