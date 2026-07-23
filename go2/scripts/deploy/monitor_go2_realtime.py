#!/usr/bin/env python3
"""Live read-only Go2 monitor for deployment tests.

This tool subscribes to DDS topics and renders rolling plots so we can watch
motor/joint/internal signals while another controller is running.

It performs no DDS writes.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np

from go2_monitor_schema import (
    POLICY_JOINT_NAMES,
    POLICY_TO_SDK,
    SCHEMA_NAME,
    SCHEMA_VERSION,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
SDK2PY_ROOT = REPO_ROOT / "reference_repos" / "sim2real_unitree_sdk2py"
JOINT_NAMES = POLICY_JOINT_NAMES
SPARK_BARS = " .:-=+*#%@"


def _ensure_sdk_import_path() -> None:
    sdk_path = str(SDK2PY_ROOT)
    if sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)


class RemoteController:
    def __init__(self) -> None:
        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        self.ry = 0.0

    def set(self, data: bytes) -> None:
        if len(data) < 24:
            return
        self.lx = struct.unpack("f", data[4:8])[0]
        self.rx = struct.unpack("f", data[8:12])[0]
        self.ry = struct.unpack("f", data[12:16])[0]
        self.ly = struct.unpack("f", data[20:24])[0]


class RollingSeries:
    def __init__(self, maxlen: int) -> None:
        self.t = deque(maxlen=maxlen)
        self.values = deque(maxlen=maxlen)

    def append(self, t: float, values: np.ndarray) -> None:
        self.t.append(float(t))
        self.values.append(np.asarray(values, dtype=np.float32))

    def arrays(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.t:
            return np.empty((0,), dtype=np.float32), np.empty((0, 0), dtype=np.float32)
        # Keep timestamps in float64 so recent samples do not collapse to the
        # same x-value when subtracting large Unix epoch times.
        return np.asarray(self.t, dtype=np.float64), np.asarray(self.values, dtype=np.float32)


class Go2RealtimeMonitor:
    def __init__(
        self,
        net_if: str,
        history_sec: float,
        sample_hz: float,
        subscribe_lowcmd: bool,
        jsonl_out: Path | None = None,
    ) -> None:
        self.net_if = net_if
        self.history_sec = history_sec
        self.sample_hz = sample_hz
        self.subscribe_lowcmd = subscribe_lowcmd
        self.jsonl_out = jsonl_out
        self.remote = RemoteController()
        self.lock = threading.Lock()

        maxlen = max(10, int(history_sec * sample_hz) + 5)
        self.joint_pos = RollingSeries(maxlen)
        self.joint_vel = RollingSeries(maxlen)
        self.tau_est = RollingSeries(maxlen)
        self.temperature = RollingSeries(maxlen)
        self.foot_force = RollingSeries(maxlen)
        self.imu_gyro = RollingSeries(maxlen)
        self.sport_vel = RollingSeries(maxlen)
        self.sport_yaw = RollingSeries(maxlen)
        self.q_err = RollingSeries(maxlen)
        self.latest_status: dict[str, Any] = {
            "low_hz": 0.0,
            "sport_hz": 0.0,
            "lowcmd_hz": 0.0,
            "remote": (0.0, 0.0, 0.0),
        }

        self._last_low_sample_t = 0.0
        self._last_sport_sample_t = 0.0
        self._latest_q = np.zeros(12, dtype=np.float32)
        self._latest_dq = np.zeros(12, dtype=np.float32)
        self._latest_tau_est = np.zeros(12, dtype=np.float32)
        self._latest_temperature = np.zeros(12, dtype=np.float32)
        self._latest_foot_force = np.zeros(4, dtype=np.float32)
        self._latest_gyro = np.zeros(3, dtype=np.float32)
        self._latest_q_des = np.zeros(12, dtype=np.float32)
        self._has_lowcmd = False
        self._latest_sport_vel = np.zeros(3, dtype=np.float32)
        self._latest_sport_yaw = 0.0

        self._jsonl_handle = None
        if self.jsonl_out is not None:
            self.jsonl_out.parent.mkdir(parents=True, exist_ok=True)
            self._jsonl_handle = self.jsonl_out.open("w", encoding="utf-8", buffering=1)

        _ensure_sdk_import_path()
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.default import (
            unitree_go_msg_dds__LowCmd_ as LowCmdGo,
            unitree_go_msg_dds__LowState_ as LowStateGo,
            unitree_go_msg_dds__SportModeState_ as SportModeStateGo,
        )

        self._topic_counts = {"low": 0, "sport": 0, "lowcmd": 0}
        self._topic_first_t = {"low": None, "sport": None, "lowcmd": None}
        self._topic_last_t = {"low": None, "sport": None, "lowcmd": None}

        ChannelFactoryInitialize(0, net_if)

        self.low_state = LowStateGo()
        self.low_sub = ChannelSubscriber("rt/lowstate", type(self.low_state))
        self.low_sub.Init(self._on_lowstate, 10)

        self.sport_state = SportModeStateGo()
        self.sport_sub = ChannelSubscriber("rt/sportmodestate", type(self.sport_state))
        self.sport_sub.Init(self._on_sportstate, 10)

        self.lowcmd_sub = None
        if subscribe_lowcmd:
            self.low_cmd = LowCmdGo()
            self.lowcmd_sub = ChannelSubscriber("rt/lowcmd", type(self.low_cmd))
            self.lowcmd_sub.Init(self._on_lowcmd, 10)

    def _mark_topic(self, name: str) -> float:
        now = time.time()
        if self._topic_first_t[name] is None:
            self._topic_first_t[name] = now
        self._topic_last_t[name] = now
        self._topic_counts[name] += 1
        first = self._topic_first_t[name]
        count = self._topic_counts[name]
        hz = 0.0 if count < 2 or first is None or now <= first else float(count - 1) / (now - first)
        key = {"low": "low_hz", "sport": "sport_hz", "lowcmd": "lowcmd_hz"}[name]
        self.latest_status[key] = hz
        return now

    def _on_lowstate(self, msg: Any) -> None:
        self.low_state = msg
        now = self._mark_topic("low")
        try:
            self.remote.set(bytes(self.low_state.wireless_remote))
        except Exception:
            pass
        if now - self._last_low_sample_t < (1.0 / self.sample_hz):
            return
        self._last_low_sample_t = now

        q = np.asarray([float(msg.motor_state[i].q) for i in POLICY_TO_SDK], dtype=np.float32)
        dq = np.asarray([float(msg.motor_state[i].dq) for i in POLICY_TO_SDK], dtype=np.float32)
        tau_est = np.asarray([float(msg.motor_state[i].tau_est) for i in POLICY_TO_SDK], dtype=np.float32)
        temp = np.asarray([float(msg.motor_state[i].temperature) for i in POLICY_TO_SDK], dtype=np.float32)
        foot_force = np.asarray([float(x) for x in msg.foot_force[:4]], dtype=np.float32)
        gyro = np.asarray([float(x) for x in msg.imu_state.gyroscope], dtype=np.float32)

        with self.lock:
            self._latest_q = q
            self._latest_dq = dq
            self._latest_tau_est = tau_est
            self._latest_temperature = temp
            self._latest_foot_force = foot_force
            self._latest_gyro = gyro
            self.joint_pos.append(now, q)
            self.joint_vel.append(now, dq)
            self.tau_est.append(now, tau_est)
            self.temperature.append(now, temp)
            self.foot_force.append(now, foot_force)
            self.imu_gyro.append(now, gyro)
            if self._has_lowcmd:
                self.q_err.append(now, self._latest_q_des - q)
            self.latest_status["remote"] = (self.remote.ly, -self.remote.lx, -self.remote.rx)
            self._write_jsonl_locked(now)

    def _on_sportstate(self, msg: Any) -> None:
        now = self._mark_topic("sport")
        if now - self._last_sport_sample_t < (1.0 / self.sample_hz):
            return
        self._last_sport_sample_t = now

        vel = np.asarray([float(x) for x in msg.velocity], dtype=np.float32)
        yaw = np.asarray([float(msg.yaw_speed)], dtype=np.float32)
        with self.lock:
            self._latest_sport_vel = vel
            self._latest_sport_yaw = float(yaw[0])
            self.sport_vel.append(now, vel)
            self.sport_yaw.append(now, yaw)

    def _on_lowcmd(self, msg: Any) -> None:
        self.low_cmd = msg
        now = self._mark_topic("lowcmd")
        q_des = np.asarray([float(msg.motor_cmd[i].q) for i in POLICY_TO_SDK], dtype=np.float32)
        with self.lock:
            self._latest_q_des = q_des
            self._has_lowcmd = True

    def _snapshot_payload_locked(self, now: float) -> dict[str, Any]:
        cmd_vx, cmd_vy, cmd_wz = self.latest_status["remote"]
        q_err = self._latest_q_des - self._latest_q if self._has_lowcmd else np.zeros(12, dtype=np.float32)
        return {
            "schema": {
                "name": SCHEMA_NAME,
                "version": SCHEMA_VERSION,
                "joint_order": "policy",
                "joint_names": JOINT_NAMES,
                "policy_to_sdk": POLICY_TO_SDK.tolist(),
                "foot_force_order": "sdk_raw",
            },
            "wall_time": now,
            "dds_hz": {
                "low": self.latest_status["low_hz"],
                "sport": self.latest_status["sport_hz"],
                "lowcmd": self.latest_status["lowcmd_hz"],
            },
            "remote_cmd": {"vx": cmd_vx, "vy": cmd_vy, "wz": cmd_wz},
            "latest": {
                "q": self._latest_q.tolist(),
                "q_des": self._latest_q_des.tolist(),
                "q_err": q_err.tolist(),
                "lowcmd_ready": self._has_lowcmd,
                "joint_vel": self._latest_dq.tolist(),
                "tau_est": self._latest_tau_est.tolist(),
                "temperature": self._latest_temperature.tolist(),
                "foot_force": self._latest_foot_force.tolist(),
                "imu_gyro": self._latest_gyro.tolist(),
                "sport_vel": self._latest_sport_vel.tolist(),
                "sport_yaw": self._latest_sport_yaw,
            },
        }

    def _write_jsonl_locked(self, now: float) -> None:
        if self._jsonl_handle is None:
            return
        self._jsonl_handle.write(json.dumps(self._snapshot_payload_locked(now)) + "\n")
        self._jsonl_handle.flush()

    def close(self) -> None:
        if self._jsonl_handle is not None:
            self._jsonl_handle.close()
            self._jsonl_handle = None

    def run(self) -> int:
        return self.run_plot_or_text()

    def run_plot_or_text(self) -> int:
        try:
            import matplotlib.pyplot as plt
            from matplotlib.animation import FuncAnimation
        except Exception:
            return self.run_text()

        return self.run_plot(plt, FuncAnimation)

    def run_plot(self, plt: Any, FuncAnimation: Any) -> int:
        fig, axes = plt.subplots(3, 2, figsize=(16, 10), sharex="col")
        fig.suptitle("Go2 Real-Time Read-Only Monitor")

        axes[0, 0].set_title("Joint Position Error (q_des - q)")
        axes[0, 1].set_title("Estimated Joint Torque")
        axes[1, 0].set_title("Joint Velocity")
        axes[1, 1].set_title("Motor Temperature")
        axes[2, 0].set_title("Base Motion")
        axes[2, 1].set_title("Foot Force")
        fig.text(
            0.5,
            0.985,
            "Joint traces ordered: FL_hip FR_hip RL_hip RR_hip FL_thigh FR_thigh RL_thigh RR_thigh FL_calf FR_calf RL_calf RR_calf",
            ha="center",
            va="top",
            fontsize=8,
        )

        line_sets: dict[str, list[Any]] = {}
        colors = plt.cm.tab20(np.linspace(0, 1, 12))
        for key, ax in (
            ("q_err", axes[0, 0]),
            ("tau_est", axes[0, 1]),
            ("joint_vel", axes[1, 0]),
            ("temperature", axes[1, 1]),
        ):
            lines = []
            for i, joint_name in enumerate(JOINT_NAMES):
                line, = ax.plot([], [], lw=1.0, color=colors[i], label=joint_name)
                lines.append(line)
            ax.grid(True, alpha=0.25)
            line_sets[key] = lines

        motion_labels = ["vx", "vy", "vz", "imu_gx", "imu_gy", "imu_gz", "yaw_speed"]
        motion_colors = ["C0", "C1", "C2", "C3", "C4", "C5", "C6"]
        motion_lines = []
        for label, color in zip(motion_labels, motion_colors):
            line, = axes[2, 0].plot([], [], lw=1.2, color=color, label=label)
            motion_lines.append(line)
        axes[2, 0].grid(True, alpha=0.25)
        axes[2, 0].legend(loc="upper left", ncol=4, fontsize=8, framealpha=0.8)
        line_sets["motion"] = motion_lines

        foot_lines = []
        for i, color in enumerate(["C0", "C1", "C2", "C3"]):
            line, = axes[2, 1].plot([], [], lw=1.2, color=color, label=f"foot_{i}")
            foot_lines.append(line)
        axes[2, 1].grid(True, alpha=0.25)
        axes[2, 1].legend(loc="upper left", ncol=4, fontsize=8, framealpha=0.8)
        line_sets["foot_force"] = foot_lines

        for ax in axes.ravel():
            ax.set_xlim(-self.history_sec, 0.0)

        status_text = fig.text(0.02, 0.01, "", fontsize=9)

        def _set_axis_ylim(ax: Any, values: np.ndarray) -> None:
            finite_values = values[np.isfinite(values)]
            if finite_values.size == 0:
                return
            lower = float(np.percentile(finite_values, 2))
            upper = float(np.percentile(finite_values, 98))
            vmin = min(lower, float(np.min(finite_values)))
            vmax = max(upper, float(np.max(finite_values)))
            if abs(vmax - vmin) < 1e-6:
                pad = max(0.1, abs(vmax) * 0.1 + 0.05)
            else:
                pad = max(0.05, 0.12 * (vmax - vmin))
            ax.set_ylim(vmin - pad, vmax + pad)

        def _set_lines(ax: Any, lines: list[Any], series: RollingSeries) -> None:
            t, values = series.arrays()
            if t.size == 0 or values.size == 0:
                return
            t_rel = t - t[-1]
            for i, line in enumerate(lines):
                if i < values.shape[1]:
                    line.set_data(t_rel, values[:, i])
            _set_axis_ylim(ax, values.reshape(-1))

        def _update(_: int) -> list[Any]:
            with self.lock:
                _set_lines(axes[0, 0], line_sets["q_err"], self.q_err)
                _set_lines(axes[0, 1], line_sets["tau_est"], self.tau_est)
                _set_lines(axes[1, 0], line_sets["joint_vel"], self.joint_vel)
                _set_lines(axes[1, 1], line_sets["temperature"], self.temperature)
                _set_lines(axes[2, 1], line_sets["foot_force"], self.foot_force)

                t_vel, sport_vel = self.sport_vel.arrays()
                t_gyro, gyro = self.imu_gyro.arrays()
                t_yaw, yaw = self.sport_yaw.arrays()
                if t_vel.size and sport_vel.size:
                    t_ref = t_vel - t_vel[-1]
                    for i in range(3):
                        line_sets["motion"][i].set_data(t_ref, sport_vel[:, i])
                if t_gyro.size and gyro.size:
                    t_gyro_rel = t_gyro - t_gyro[-1]
                    for i in range(3):
                        line_sets["motion"][3 + i].set_data(t_gyro_rel, gyro[:, i])
                if t_yaw.size:
                    t_yaw_rel = t_yaw - t_yaw[-1]
                    line_sets["motion"][6].set_data(t_yaw_rel, yaw[:, 0])
                motion_values = []
                if sport_vel.size:
                    motion_values.append(sport_vel)
                if gyro.size:
                    motion_values.append(gyro)
                if t_yaw.size and yaw.size:
                    motion_values.append(yaw)
                if motion_values:
                    finite_chunks = []
                    for value in motion_values:
                        finite = value[np.isfinite(value)]
                        if finite.size:
                            finite_chunks.append(finite)
                    finite_values = np.concatenate(finite_chunks) if finite_chunks else np.empty((0,), dtype=np.float32)
                    if finite_values.size:
                        _set_axis_ylim(axes[2, 0], finite_values)

                cmd_vx, cmd_vy, cmd_wz = self.latest_status["remote"]
                status_text.set_text(
                    "DDS hz "
                    f"low={self.latest_status['low_hz']:.1f} "
                    f"sport={self.latest_status['sport_hz']:.1f} "
                    f"lowcmd={self.latest_status['lowcmd_hz']:.1f} | "
                    f"remote cmd approx vx={cmd_vx:+.2f} vy={cmd_vy:+.2f} wz={cmd_wz:+.2f}"
                )

            artists: list[Any] = [status_text]
            for lines in line_sets.values():
                artists.extend(lines)
            return artists

        stop_requested = threading.Event()

        def _request_close(_: Any = None) -> None:
            stop_requested.set()

        fig.canvas.mpl_connect("close_event", _request_close)
        anim = FuncAnimation(fig, _update, interval=250, blit=False, cache_frame_data=False)
        fig._go2_anim = anim
        plt.tight_layout(rect=(0, 0.03, 1, 0.96))
        plt.show(block=False)
        try:
            while not stop_requested.is_set() and plt.fignum_exists(fig.number):
                plt.pause(0.2)
        except KeyboardInterrupt:
            stop_requested.set()
        finally:
            try:
                anim.event_source.stop()
            except Exception:
                pass
            if plt.fignum_exists(fig.number):
                plt.close(fig)
        return 0

    def _sparkline(self, values: np.ndarray, width: int = 48) -> str:
        if values.size == 0:
            return "(no data)"
        if values.ndim > 1:
            values = np.linalg.norm(values, axis=1)
        if values.size > width:
            indices = np.linspace(0, values.size - 1, width).astype(int)
            values = values[indices]
        vmin = float(np.min(values))
        vmax = float(np.max(values))
        if abs(vmax - vmin) < 1e-6:
            idx = min(len(SPARK_BARS) - 1, max(0, int(round(abs(vmax)))))
            return SPARK_BARS[idx] * len(values)
        scaled = (values - vmin) / (vmax - vmin)
        chars = [SPARK_BARS[min(len(SPARK_BARS) - 1, int(x * (len(SPARK_BARS) - 1)))] for x in scaled]
        return "".join(chars)

    def _latest_metric_summary(self) -> list[str]:
        with self.lock:
            _, q_err = self.q_err.arrays()
            _, tau = self.tau_est.arrays()
            _, dq = self.joint_vel.arrays()
            _, temp = self.temperature.arrays()
            _, foot = self.foot_force.arrays()
            _, gyro = self.imu_gyro.arrays()
            _, sport_vel = self.sport_vel.arrays()
            _, sport_yaw = self.sport_yaw.arrays()
            cmd_vx, cmd_vy, cmd_wz = self.latest_status["remote"]

        lines = [
            f"DDS hz low={self.latest_status['low_hz']:.1f} sport={self.latest_status['sport_hz']:.1f} lowcmd={self.latest_status['lowcmd_hz']:.1f}",
            f"remote cmd approx vx={cmd_vx:+.2f} vy={cmd_vy:+.2f} wz={cmd_wz:+.2f}",
        ]

        def add_series(title: str, arr: np.ndarray, scalar_fmt: str) -> None:
            if arr.size == 0:
                lines.append(f"{title:<18} (no data)")
                return
            series = np.linalg.norm(arr, axis=1) if arr.ndim > 1 else arr
            lines.append(
                f"{title:<18} now={scalar_fmt.format(float(series[-1]))} "
                f"max={scalar_fmt.format(float(np.max(series)))} {self._sparkline(series)}"
            )

        add_series("q_err_norm", q_err, "{:+.3f}")
        add_series("tau_est_norm", tau, "{:+.3f}")
        add_series("joint_vel_norm", dq, "{:+.3f}")
        add_series("temp_max", np.max(temp, axis=1) if temp.size else temp, "{:+.1f}")
        add_series("foot_force_norm", foot, "{:+.1f}")
        add_series("imu_gyro_norm", gyro, "{:+.3f}")
        add_series("sport_vel_norm", sport_vel, "{:+.3f}")
        add_series("sport_yaw", sport_yaw[:, 0] if sport_yaw.size else sport_yaw, "{:+.3f}")
        if q_err.size:
            latest_q_err = np.abs(q_err[-1])
            top_q = np.argsort(latest_q_err)[::-1][:3]
            lines.append(
                "top_q_err         "
                + " ".join(f"{JOINT_NAMES[i]}={latest_q_err[i]:.3f}" for i in top_q)
            )
        if tau.size:
            latest_tau = np.abs(tau[-1])
            top_tau = np.argsort(latest_tau)[::-1][:3]
            lines.append(
                "top_tau_est       "
                + " ".join(f"{JOINT_NAMES[i]}={latest_tau[i]:.2f}" for i in top_tau)
            )
        return lines

    def run_text(self) -> int:
        print("[INFO] matplotlib not available in this environment; using terminal monitor mode.")
        print("[INFO] Press Ctrl-C to stop.")
        try:
            while True:
                lines = self._latest_metric_summary()
                sys.stdout.write("\x1b[2J\x1b[H")
                sys.stdout.write("Go2 Real-Time Read-Only Monitor (text mode)\n")
                sys.stdout.write("=" * 72 + "\n")
                for line in lines:
                    sys.stdout.write(line + "\n")
                sys.stdout.flush()
                time.sleep(0.2)
        except KeyboardInterrupt:
            sys.stdout.write("\n")
            return 0

    def run_headless_stream(self, jsonl_out: Path) -> int:
        print(f"[INFO] Headless streaming mode -> {jsonl_out}")
        jsonl_out.parent.mkdir(parents=True, exist_ok=True)
        with jsonl_out.open("w", encoding="utf-8") as handle:
            try:
                while True:
                    with self.lock:
                        payload = self._snapshot_payload_locked(time.time())
                    handle.write(json.dumps(payload) + "\n")
                    handle.flush()
                    time.sleep(max(0.02, 1.0 / self.sample_hz))
            except KeyboardInterrupt:
                return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--net-if", required=True)
    parser.add_argument("--history-sec", type=float, default=20.0)
    parser.add_argument("--sample-hz", type=float, default=25.0)
    parser.add_argument("--no-lowcmd", action="store_true", help="Skip rt/lowcmd subscription.")
    parser.add_argument("--headless-jsonl-out", type=str, default=None)
    parser.add_argument("--jsonl-out", type=str, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    monitor = Go2RealtimeMonitor(
        net_if=args.net_if,
        history_sec=args.history_sec,
        sample_hz=args.sample_hz,
        subscribe_lowcmd=not args.no_lowcmd,
        jsonl_out=Path(args.jsonl_out) if args.jsonl_out else None,
    )
    try:
        if args.headless_jsonl_out:
            return monitor.run_headless_stream(Path(args.headless_jsonl_out))
        return monitor.run()
    finally:
        monitor.close()


if __name__ == "__main__":
    raise SystemExit(main())
