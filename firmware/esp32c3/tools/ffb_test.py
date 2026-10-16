#!/usr/bin/env python3
"""Linux serial FFB signal generator for the ESP32-C3 wheel firmware."""

from __future__ import annotations

import argparse
import glob
import math
import os
import select
import sys
import termios
import time
from collections.abc import Callable


DEFAULT_FORCE_LIMIT = 30
HARD_FORCE_LIMIT = 100
DEFAULT_INTERVAL_MS = 20


class SerialConnection:
    def __init__(self, path: str, quiet: bool) -> None:
        self.path = path
        self.quiet = quiet
        self.fd: int | None = None
        self.old_attributes: list | None = None
        self.receive_buffer = bytearray()

    def open(self) -> None:
        self.fd = os.open(
            self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
        )
        self.old_attributes = termios.tcgetattr(self.fd)
        attributes = termios.tcgetattr(self.fd)
        attributes[0] = 0
        attributes[1] = 0
        attributes[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attributes[3] = 0
        attributes[4] = termios.B115200
        attributes[5] = termios.B115200
        attributes[6][termios.VMIN] = 0
        attributes[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attributes)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def close(self) -> None:
        if self.fd is None:
            return
        try:
            if self.old_attributes is not None:
                termios.tcsetattr(
                    self.fd, termios.TCSANOW, self.old_attributes
                )
        except (OSError, termios.error):
            pass
        finally:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None

    def write_command(self, command: str) -> None:
        if self.fd is None:
            raise RuntimeError("Serial port is not open")
        pending = memoryview((command + "\n").encode("ascii"))
        deadline = time.monotonic() + 0.05
        while pending:
            try:
                written = os.write(self.fd, pending)
                pending = pending[written:]
            except BlockingIOError:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("Serial write timed out")
                select.select([], [self.fd], [], remaining)

    def print_received_lines(self) -> None:
        if self.fd is None:
            return
        while True:
            try:
                block = os.read(self.fd, 4096)
            except BlockingIOError:
                break
            if not block:
                break
            self.receive_buffer.extend(block)

        while b"\n" in self.receive_buffer:
            raw_line, _, remainder = self.receive_buffer.partition(b"\n")
            self.receive_buffer = bytearray(remainder)
            if not self.quiet:
                line = raw_line.rstrip(b"\r").decode("utf-8", "replace")
                print(f"[ESP32] {line}")


class DryRunConnection:
    def __init__(self) -> None:
        self.last_command = ""

    def open(self) -> None:
        pass

    def close(self) -> None:
        pass

    def write_command(self, command: str) -> None:
        if command != self.last_command:
            print(f"[DRY-RUN] {command}")
            self.last_command = command

    def print_received_lines(self) -> None:
        pass


def discover_serial_port() -> str:
    candidates = sorted(
        glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    )
    if not candidates:
        raise RuntimeError(
            "ESP32 serial port not found; use --port /dev/ttyACM0"
        )
    if len(candidates) > 1:
        choices = ", ".join(candidates)
        raise RuntimeError(f"Multiple serial ports found ({choices}); use --port")
    return candidates[0]


def validate_arguments(args: argparse.Namespace) -> None:
    if not 5 <= args.interval_ms <= 50:
        raise ValueError("--interval-ms must be between 5 and 50")
    if not 1 <= args.max_force <= HARD_FORCE_LIMIT:
        raise ValueError("--max-force must be between 1 and 100")
    if args.max_force > DEFAULT_FORCE_LIMIT and not args.allow_high_force:
        raise ValueError(
            "A force limit above 30 requires --allow-high-force"
        )

    requested_force = getattr(args, "force", None)
    requested_amplitude = getattr(args, "amplitude", None)
    for label, value in (
        ("force", requested_force),
        ("amplitude", requested_amplitude),
    ):
        if value is not None and abs(value) > args.max_force:
            raise ValueError(
                f"Requested {label} {value} exceeds --max-force {args.max_force}"
            )

    if args.startup_delay < 0:
        raise ValueError("--startup-delay cannot be negative")

    for name in ("duration", "frequency", "pulse_ms", "settle", "ramp",
                 "hold", "release", "drive", "silence"):
        value = getattr(args, name, None)
        if value is not None and value <= 0:
            raise ValueError(f"--{name.replace('_', '-')} must be positive")


def clamp_force(value: float, force_limit: int) -> int:
    return max(-force_limit, min(force_limit, round(value)))


def run_profile(
    connection: SerialConnection | DryRunConnection,
    profile: Callable[[float], float],
    duration: float,
    interval_seconds: float,
    force_limit: int,
) -> None:
    started = time.monotonic()
    next_command = started
    while True:
        now = time.monotonic()
        elapsed = now - started
        if elapsed >= duration:
            break

        if now >= next_command:
            force = clamp_force(profile(elapsed), force_limit)
            connection.write_command(f"f {force}")
            next_command += interval_seconds
            if next_command < now:
                next_command = now + interval_seconds

        connection.print_received_lines()
        sleep_time = max(0.0, min(0.002, next_command - time.monotonic()))
        if sleep_time:
            time.sleep(sleep_time)


def run_effect(
    args: argparse.Namespace,
    connection: SerialConnection | DryRunConnection,
) -> None:
    interval_seconds = args.interval_ms / 1000.0

    if args.effect == "constant":
        run_profile(
            connection,
            lambda _elapsed: args.force,
            args.duration,
            interval_seconds,
            args.max_force,
        )
        return

    if args.effect == "sine":
        run_profile(
            connection,
            lambda elapsed: args.amplitude
            * math.sin(2.0 * math.pi * args.frequency * elapsed),
            args.duration,
            interval_seconds,
            args.max_force,
        )
        return

    if args.effect == "impact":
        pulse_seconds = args.pulse_ms / 1000.0
        duration = pulse_seconds + args.settle
        run_profile(
            connection,
            lambda elapsed: args.force if elapsed < pulse_seconds else 0,
            duration,
            interval_seconds,
            args.max_force,
        )
        return

    if args.effect == "corner":
        duration = args.ramp + args.hold + args.release

        def corner_profile(elapsed: float) -> float:
            if elapsed < args.ramp:
                return args.force * elapsed / args.ramp
            elapsed -= args.ramp
            if elapsed < args.hold:
                return args.force
            elapsed -= args.hold
            return args.force * max(0.0, 1.0 - elapsed / args.release)

        run_profile(
            connection,
            corner_profile,
            duration,
            interval_seconds,
            args.max_force,
        )
        return

    if args.effect == "watchdog":
        run_profile(
            connection,
            lambda _elapsed: args.force,
            args.drive,
            interval_seconds,
            args.max_force,
        )
        print(
            f"FFB transmission paused for {args.silence:.2f}s; "
            "ESP32 should stop after 100 ms."
        )
        silence_started = time.monotonic()
        while time.monotonic() - silence_started < args.silence:
            connection.print_received_lines()
            time.sleep(0.002)
        return

    raise RuntimeError(f"Unknown effect: {args.effect}")


def add_common_force_argument(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--force", type=int, default=20)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send safe test FFB effects to the ESP32-C3 wheel"
    )
    parser.add_argument("--port", help="serial device; auto-detected if omitted")
    parser.add_argument(
        "--interval-ms", type=int, default=DEFAULT_INTERVAL_MS,
        help="command period (5-50 ms, default: 20)",
    )
    parser.add_argument(
        "--max-force", type=int, default=DEFAULT_FORCE_LIMIT,
        help="absolute safety clamp (default: 30)",
    )
    parser.add_argument(
        "--allow-high-force", action="store_true",
        help="required when --max-force is above 30",
    )
    parser.add_argument(
        "--initialize", action="store_true",
        help="send 'a' first; the wheel must already be physically centered",
    )
    parser.add_argument("--quiet", action="store_true", help="hide telemetry")
    parser.add_argument(
        "--startup-delay", type=float, default=0.5,
        help="seconds to wait after opening the port",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="show commands without serial I/O"
    )

    subparsers = parser.add_subparsers(dest="effect", required=True)

    constant = subparsers.add_parser("constant", help="constant left/right torque")
    add_common_force_argument(constant)
    constant.add_argument("--duration", type=float, default=3.0)

    sine = subparsers.add_parser("sine", help="alternating sine torque")
    sine.add_argument("--amplitude", type=int, default=20)
    sine.add_argument("--frequency", type=float, default=1.0)
    sine.add_argument("--duration", type=float, default=5.0)

    impact = subparsers.add_parser("impact", help="short collision pulse")
    add_common_force_argument(impact)
    impact.set_defaults(force=30)
    impact.add_argument("--pulse-ms", type=float, default=120.0)
    impact.add_argument("--settle", type=float, default=0.5)

    corner = subparsers.add_parser("corner", help="ramp, hold and release torque")
    add_common_force_argument(corner)
    corner.set_defaults(force=25)
    corner.add_argument("--ramp", type=float, default=1.0)
    corner.add_argument("--hold", type=float, default=2.0)
    corner.add_argument("--release", type=float, default=1.0)

    watchdog = subparsers.add_parser(
        "watchdog", help="stop transmission and verify the 100 ms watchdog"
    )
    add_common_force_argument(watchdog)
    watchdog.add_argument("--drive", type=float, default=0.3)
    watchdog.add_argument("--silence", type=float, default=0.5)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        validate_arguments(args)
    except ValueError as error:
        parser.error(str(error))

    if args.dry_run:
        connection: SerialConnection | DryRunConnection = DryRunConnection()
        port_description = "dry-run"
    else:
        try:
            serial_path = args.port or discover_serial_port()
        except RuntimeError as error:
            parser.error(str(error))
        connection = SerialConnection(serial_path, args.quiet)
        port_description = serial_path

    print(
        f"Port: {port_description} | effect: {args.effect} | "
        f"force limit: {args.max_force}% | period: {args.interval_ms} ms"
    )
    if args.max_force > DEFAULT_FORCE_LIMIT:
        print("WARNING: high-force mode is enabled")

    try:
        connection.open()
        if args.startup_delay > 0:
            time.sleep(args.startup_delay)
        connection.print_received_lines()

        if args.initialize:
            print("Initializing: current wheel position will become zero")
            connection.write_command("a")
            time.sleep(0.1)

        run_effect(args, connection)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted; stopping motor")
        return 130
    except (OSError, RuntimeError, TimeoutError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    finally:
        try:
            connection.write_command("f 0")
            connection.write_command("s")
        except (OSError, RuntimeError, TimeoutError):
            pass
        connection.close()


if __name__ == "__main__":
    raise SystemExit(main())
