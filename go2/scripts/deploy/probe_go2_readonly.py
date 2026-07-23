#!/usr/bin/env python3
"""Read-only Go2 state probe.

This script is intentionally conservative:

- no lowcmd publisher
- no mode switch
- no sport client RPC calls
- no DDS writes of any kind

It is meant to answer the very first deployment question:
"What state can we actually read from the robot right now?"

Primary topics:

- `rt/lowstate`
- `rt/sportmodestate` (best effort)
- `rt/lowcmd` (optional read-only tap of commanded joint targets/gains)
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
SDK2PY_ROOT = REPO_ROOT / "reference_repos" / "sim2real_unitree_sdk2py"


class KeyMap:
    R1 = 0
    L1 = 1
    start = 2
    select = 3
    R2 = 4
    L2 = 5
    F1 = 6
    F2 = 7
    A = 8
    B = 9
    X = 10
    Y = 11
    up = 12
    right = 13
    down = 14
    left = 15


BUTTON_NAMES = [
    "R1",
    "L1",
    "start",
    "select",
    "R2",
    "L2",
    "F1",
    "F2",
    "A",
    "B",
    "X",
    "Y",
    "up",
    "right",
    "down",
    "left",
]


class RemoteController:
    def __init__(self) -> None:
        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        self.ry = 0.0
        self.button = [0] * 16

    def set(self, data: bytes) -> None:
        if len(data) < 24:
            return
        keys = struct.unpack("H", data[2:4])[0]
        for i in range(16):
            self.button[i] = (keys & (1 << i)) >> i
        self.lx = struct.unpack("f", data[4:8])[0]
        self.rx = struct.unpack("f", data[8:12])[0]
        self.ry = struct.unpack("f", data[12:16])[0]
        self.ly = struct.unpack("f", data[20:24])[0]

    def active_buttons(self) -> list[str]:
        return [BUTTON_NAMES[i] for i, pressed in enumerate(self.button) if pressed]


def _ensure_sdk_import_path() -> None:
    sdk_path = str(SDK2PY_ROOT)
    if sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)


@dataclass
class TopicStats:
    name: str
    count: int = 0
    first_wall_time: float | None = None
    last_wall_time: float | None = None

    def mark(self) -> None:
        now = time.time()
        if self.first_wall_time is None:
            self.first_wall_time = now
        self.last_wall_time = now
        self.count += 1

    def hz(self) -> float | None:
        if self.first_wall_time is None or self.last_wall_time is None or self.count < 2:
            return None
        duration = self.last_wall_time - self.first_wall_time
        if duration <= 0:
            return None
        return float(self.count - 1) / duration


class Go2ReadOnlyProbe:
    def __init__(
        self,
        net_if: str,
        subscribe_sport: bool = True,
        subscribe_lowcmd: bool = False,
    ) -> None:
        self.net_if = net_if
        self.subscribe_sport = subscribe_sport
        self.subscribe_lowcmd = subscribe_lowcmd
        self.remote = RemoteController()
        self.low_state = None
        self.sport_state = None
        self.low_cmd = None
        self.lowstate_stream_jsonl_out: Path | None = None
        self.lowcmd_stream_jsonl_out: Path | None = None
        self._lowstate_stream_handle = None
        self._lowstate_stream_lock = threading.Lock()
        self._lowcmd_stream_handle = None
        self._lowcmd_stream_lock = threading.Lock()
        self.low_stats = TopicStats("rt/lowstate")
        self.sport_stats = TopicStats("rt/sportmodestate")
        self.lowcmd_stats = TopicStats("rt/lowcmd")

        _ensure_sdk_import_path()
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
            from unitree_sdk2py.idl.default import (
                unitree_go_msg_dds__LowCmd_ as LowCmdGo,
                unitree_go_msg_dds__LowState_ as LowStateGo,
                unitree_go_msg_dds__SportModeState_ as SportModeStateGo,
            )
        except ModuleNotFoundError as exc:
            missing = exc.name or "unknown module"
            raise SystemExit(
                "Missing Unitree DDS Python dependency while starting read-only probe.\n"
                f"Missing module: {missing}\n"
                "Activate the hardware SDK environment and make sure "
                "`reference_repos/sim2real_unitree_sdk2py` is installed."
            ) from exc

        ChannelFactoryInitialize(0, net_if)

        self.low_state = LowStateGo()
        self.low_sub = ChannelSubscriber("rt/lowstate", type(self.low_state))
        self.low_sub.Init(self._on_lowstate, 10)

        self.sport_sub = None
        if subscribe_sport:
            self.sport_state = SportModeStateGo()
            self.sport_sub = ChannelSubscriber("rt/sportmodestate", type(self.sport_state))
            self.sport_sub.Init(self._on_sportstate, 10)

        self.lowcmd_sub = None
        if subscribe_lowcmd:
            self.low_cmd = LowCmdGo()
            self.lowcmd_sub = ChannelSubscriber("rt/lowcmd", type(self.low_cmd))
            self.lowcmd_sub.Init(self._on_lowcmd, 10)

    def _on_lowstate(self, msg: Any) -> None:
        self.low_state = msg
        self.low_stats.mark()
        try:
            self.remote.set(bytes(self.low_state.wireless_remote))
        except Exception:
            pass
        if self._lowstate_stream_handle is not None:
            sample = {
                "wall_time": time.time(),
                "monotonic_ns": time.perf_counter_ns(),
                "iface": self.net_if,
                "lowstate": {
                    "count": self.low_stats.count,
                    "hz_estimate": self.low_stats.hz(),
                    "snapshot": self._low_snapshot(),
                },
            }
            with self._lowstate_stream_lock:
                self._lowstate_stream_handle.write(json.dumps(sample) + "\n")

    def _on_sportstate(self, msg: Any) -> None:
        self.sport_state = msg
        self.sport_stats.mark()

    def _on_lowcmd(self, msg: Any) -> None:
        self.low_cmd = msg
        self.lowcmd_stats.mark()
        if self._lowcmd_stream_handle is not None:
            sample = {
                "wall_time": time.time(),
                "monotonic_ns": time.perf_counter_ns(),
                "iface": self.net_if,
                "lowcmd": {
                    "count": self.lowcmd_stats.count,
                    "hz_estimate": self.lowcmd_stats.hz(),
                    "snapshot": self._lowcmd_snapshot(),
                },
            }
            with self._lowcmd_stream_lock:
                self._lowcmd_stream_handle.write(json.dumps(sample) + "\n")
                self._lowcmd_stream_handle.flush()

    def _wait_for_first_lowstate(self, timeout_s: float) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.low_state is not None and getattr(self.low_state, "tick", 0) != 0:
                return True
            time.sleep(0.05)
        return False

    def _low_snapshot(self) -> dict[str, Any] | None:
        if self.low_state is None or getattr(self.low_state, "tick", 0) == 0:
            return None
        q = [float(self.low_state.motor_state[i].q) for i in range(12)]
        dq = [float(self.low_state.motor_state[i].dq) for i in range(12)]
        quat = [float(x) for x in self.low_state.imu_state.quaternion]
        gyro = [float(x) for x in self.low_state.imu_state.gyroscope]
        accel = [float(x) for x in self.low_state.imu_state.accelerometer]
        return {
            "tick": int(getattr(self.low_state, "tick", 0)),
            "imu_quaternion_wxyz": quat,
            "imu_gyro_xyz": gyro,
            "imu_accel_xyz": accel,
            "joint_q_12": q,
            "joint_dq_12": dq,
            "remote": {
                "lx": float(self.remote.lx),
                "ly": float(self.remote.ly),
                "rx": float(self.remote.rx),
                "ry": float(self.remote.ry),
                "active_buttons": self.remote.active_buttons(),
            },
            "foot_force": [int(x) for x in self.low_state.foot_force],
            "temperature_hint": [int(self.low_state.motor_state[i].temperature) for i in range(12)],
        }

    def _sport_snapshot(self) -> dict[str, Any] | None:
        if self.sport_state is None:
            return None
        return {
            "mode": int(self.sport_state.mode),
            "gait_type": int(self.sport_state.gait_type),
            "progress": float(self.sport_state.progress),
            "body_height": float(self.sport_state.body_height),
            "position_xyz": [float(x) for x in self.sport_state.position],
            "velocity_xyz": [float(x) for x in self.sport_state.velocity],
            "yaw_speed": float(self.sport_state.yaw_speed),
            "foot_force": [int(x) for x in self.sport_state.foot_force],
            "foot_position_body": [float(x) for x in self.sport_state.foot_position_body],
            "foot_speed_body": [float(x) for x in self.sport_state.foot_speed_body],
        }

    def _lowcmd_snapshot(self) -> dict[str, Any] | None:
        if self.low_cmd is None:
            return None
        motor_cmd = self.low_cmd.motor_cmd
        q = [float(motor_cmd[i].q) for i in range(12)]
        dq = [float(motor_cmd[i].dq) for i in range(12)]
        tau = [float(motor_cmd[i].tau) for i in range(12)]
        kp = [float(motor_cmd[i].kp) for i in range(12)]
        kd = [float(motor_cmd[i].kd) for i in range(12)]
        return {
            "level_flag": int(self.low_cmd.level_flag),
            "gpio": int(self.low_cmd.gpio),
            "joint_q_des_12": q,
            "joint_dq_des_12": dq,
            "joint_tau_ff_12": tau,
            "joint_kp_12": kp,
            "joint_kd_12": kd,
        }

    def run(
        self,
        duration_s: float,
        print_every_s: float,
        series_jsonl_out: Path | None = None,
        lowstate_stream_jsonl_out: Path | None = None,
        lowcmd_stream_jsonl_out: Path | None = None,
    ) -> dict[str, Any]:
        print(f"[INFO] Read-only probe on iface={self.net_if}")
        print("[INFO] Subscribing to rt/lowstate")
        if self.subscribe_sport:
            print("[INFO] Subscribing to rt/sportmodestate (best effort)")
        if self.subscribe_lowcmd:
            print("[INFO] Subscribing to rt/lowcmd (read-only commanded targets/gains)")
        print("[INFO] No commands will be sent.")
        if series_jsonl_out is not None:
            print(f"[INFO] Writing time-series snapshots to {series_jsonl_out}")
            series_jsonl_out.parent.mkdir(parents=True, exist_ok=True)
            series_jsonl_out.write_text("")
        if lowstate_stream_jsonl_out is not None:
            print(f"[INFO] Writing full-rate lowstate stream to {lowstate_stream_jsonl_out}")
            lowstate_stream_jsonl_out.parent.mkdir(parents=True, exist_ok=True)
            lowstate_stream_jsonl_out.write_text("")
            self.lowstate_stream_jsonl_out = lowstate_stream_jsonl_out
            self._lowstate_stream_handle = lowstate_stream_jsonl_out.open(
                "a",
                encoding="utf-8",
                buffering=1,
            )
        if lowcmd_stream_jsonl_out is not None:
            if not self.subscribe_lowcmd:
                raise SystemExit("--lowcmd-stream-jsonl-out requires --subscribe-lowcmd.")
            print(f"[INFO] Writing full-rate lowcmd stream to {lowcmd_stream_jsonl_out}")
            lowcmd_stream_jsonl_out.parent.mkdir(parents=True, exist_ok=True)
            lowcmd_stream_jsonl_out.write_text("")
            self.lowcmd_stream_jsonl_out = lowcmd_stream_jsonl_out
            self._lowcmd_stream_handle = lowcmd_stream_jsonl_out.open(
                "a",
                encoding="utf-8",
                buffering=1,
            )

        got_low = self._wait_for_first_lowstate(timeout_s=max(2.0, min(duration_s, 10.0)))
        if not got_low:
            raise SystemExit(
                "Timed out waiting for rt/lowstate. Check the robot-facing NIC, DDS setup, and cable."
            )

        end_time = time.time() + duration_s
        next_print = time.time()
        while time.time() < end_time:
            now = time.time()
            if now >= next_print:
                low = self._low_snapshot()
                sport = self._sport_snapshot()
                lowcmd = self._lowcmd_snapshot()
                wall_time = time.time()
                print("\n[SNAPSHOT]")
                print(
                    f"  lowstate count={self.low_stats.count} hz="
                    f"{self.low_stats.hz():.1f}" if self.low_stats.hz() is not None else
                    f"  lowstate count={self.low_stats.count} hz=unknown"
                )
                if low is not None:
                    print(
                        f"  low.tick={low['tick']} gyro={np.round(low['imu_gyro_xyz'], 3).tolist()} "
                        f"q0-2={np.round(low['joint_q_12'][:3], 3).tolist()} "
                        f"buttons={low['remote']['active_buttons']}"
                    )
                else:
                    print("  lowstate snapshot unavailable")
                if self.subscribe_sport:
                    if sport is not None:
                        hz = self.sport_stats.hz()
                        hz_str = f"{hz:.1f}" if hz is not None else "unknown"
                        print(
                            f"  sport count={self.sport_stats.count} hz={hz_str} "
                            f"mode={sport['mode']} gait={sport['gait_type']} "
                            f"vel={np.round(sport['velocity_xyz'], 3).tolist()} "
                            f"yaw_speed={sport['yaw_speed']:.3f}"
                        )
                    else:
                        print("  sport snapshot unavailable")
                if self.subscribe_lowcmd:
                    if lowcmd is not None:
                        hz = self.lowcmd_stats.hz()
                        hz_str = f"{hz:.1f}" if hz is not None else "unknown"
                        print(
                            f"  lowcmd count={self.lowcmd_stats.count} hz={hz_str} "
                            f"kp0-2={np.round(lowcmd['joint_kp_12'][:3], 3).tolist()} "
                            f"kd0-2={np.round(lowcmd['joint_kd_12'][:3], 3).tolist()} "
                            f"qdes0-2={np.round(lowcmd['joint_q_des_12'][:3], 3).tolist()}"
                        )
                    else:
                        print("  lowcmd snapshot unavailable")
                if series_jsonl_out is not None:
                    sample = {
                        "wall_time": wall_time,
                        "iface": self.net_if,
                        "lowstate": {
                            "count": self.low_stats.count,
                            "hz_estimate": self.low_stats.hz(),
                            "snapshot": low,
                        },
                        "sportmodestate": {
                            "subscribed": self.subscribe_sport,
                            "count": self.sport_stats.count,
                            "hz_estimate": self.sport_stats.hz(),
                            "snapshot": sport,
                        },
                        "lowcmd": {
                            "subscribed": self.subscribe_lowcmd,
                            "count": self.lowcmd_stats.count,
                            "hz_estimate": self.lowcmd_stats.hz(),
                            "snapshot": lowcmd,
                        },
                    }
                    with series_jsonl_out.open("a", encoding="utf-8") as handle:
                        handle.write(json.dumps(sample) + "\n")
                next_print = now + print_every_s
            time.sleep(0.05)

        try:
            return {
                "iface": self.net_if,
                "duration_s": duration_s,
                "lowstate": {
                    "count": self.low_stats.count,
                    "hz_estimate": self.low_stats.hz(),
                    "snapshot": self._low_snapshot(),
                    "stream_jsonl_out": (
                        str(self.lowstate_stream_jsonl_out) if self.lowstate_stream_jsonl_out else None
                    ),
                },
                "sportmodestate": {
                    "subscribed": self.subscribe_sport,
                    "count": self.sport_stats.count,
                    "hz_estimate": self.sport_stats.hz(),
                    "snapshot": self._sport_snapshot(),
                },
                "lowcmd": {
                    "subscribed": self.subscribe_lowcmd,
                    "count": self.lowcmd_stats.count,
                    "hz_estimate": self.lowcmd_stats.hz(),
                    "snapshot": self._lowcmd_snapshot(),
                    "stream_jsonl_out": (
                        str(self.lowcmd_stream_jsonl_out) if self.lowcmd_stream_jsonl_out else None
                    ),
                },
            }
        finally:
            if self._lowstate_stream_handle is not None:
                with self._lowstate_stream_lock:
                    self._lowstate_stream_handle.close()
                self._lowstate_stream_handle = None
            if self._lowcmd_stream_handle is not None:
                with self._lowcmd_stream_lock:
                    self._lowcmd_stream_handle.close()
                self._lowcmd_stream_handle = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--net-if", required=True)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--print-every", type=float, default=1.0)
    parser.add_argument("--no-sport", action="store_true", help="Skip subscription to rt/sportmodestate.")
    parser.add_argument(
        "--subscribe-lowcmd",
        action="store_true",
        help="Also subscribe read-only to rt/lowcmd to log commanded q/dq/tau/kp/kd.",
    )
    parser.add_argument("--json-out", type=str, default=None)
    parser.add_argument(
        "--series-jsonl-out",
        type=str,
        default=None,
        help="Optional JSONL path for periodic read-only snapshots during the capture window.",
    )
    parser.add_argument(
        "--lowstate-stream-jsonl-out",
        type=str,
        default=None,
        help="Optional JSONL path for full-rate rt/lowstate samples.",
    )
    parser.add_argument(
        "--lowcmd-stream-jsonl-out",
        type=str,
        default=None,
        help="Optional JSONL path for full-rate rt/lowcmd samples.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    probe = Go2ReadOnlyProbe(
        net_if=args.net_if,
        subscribe_sport=not args.no_sport,
        subscribe_lowcmd=args.subscribe_lowcmd,
    )
    result = probe.run(
        duration_s=args.duration,
        print_every_s=args.print_every,
        series_jsonl_out=Path(args.series_jsonl_out) if args.series_jsonl_out else None,
        lowstate_stream_jsonl_out=(
            Path(args.lowstate_stream_jsonl_out) if args.lowstate_stream_jsonl_out else None
        ),
        lowcmd_stream_jsonl_out=(
            Path(args.lowcmd_stream_jsonl_out) if args.lowcmd_stream_jsonl_out else None
        ),
    )
    print("\n[SUMMARY]")
    print(json.dumps(result, indent=2))
    if args.json_out:
        Path(args.json_out).write_text(json.dumps(result, indent=2))
        print(f"[INFO] Wrote summary to {args.json_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
