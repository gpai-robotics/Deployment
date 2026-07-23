#!/usr/bin/env python3
"""Run the fixed deployment-validation gate for an exported Go2 bundle.

This is intentionally a gate runner, not another policy-specific benchmark.
It records the same contract checks for every candidate before we decide
whether to spend time on full IsaacSim/MuJoCo rollouts or hardware bring-up.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import torch


REPO_ROOT = Path(__file__).resolve().parents[2]
RUN_SIM2SIM = REPO_ROOT/ "deploy" / "run_sim2sim.py"
RUN_MUJOCO_SUITE = REPO_ROOT/ "deploy" / "run_mujoco_ood_suite.py"
VALIDATE_BUNDLE = REPO_ROOT/ "deploy" / "validate_bundle.py"
VALIDATE_INFERENCE_PARITY = REPO_ROOT/ "deploy" / "validate_policy_inference_parity.py"
VALIDATE_UNITREE_MJLAB_FSM = REPO_ROOT/ "deploy" / "validate_unitree_mjlab_go2_fsm_runtime.py"
DEFAULT_OUTPUT_DIR = REPO_ROOT/ "artifacts" / "deployment_validation"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle-dir", required=True)
    parser.add_argument("--policy-name", default="", help="Optional output-name override.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--net-if", default="enp0s31f6")
    parser.add_argument("--expected-policy-obs-dim", type=int, default=0)
    parser.add_argument("--expected-history-length", type=int, default=0)
    parser.add_argument("--expected-action-dim", type=int, default=12)
    parser.add_argument("--python-exe", default=sys.executable)
    parser.add_argument("--run-mujoco-suite", action="store_true", help="Run the moderate MuJoCo rollout suite.")
    parser.add_argument("--mujoco-suite", default="mujoco_disturb_v2_moderate")
    parser.add_argument("--mujoco-rollouts", type=int, default=3)
    parser.add_argument("--mujoco-max-steps", type=int, default=900)
    parser.add_argument("--mujoco-trace-steps", type=int, default=0)
    parser.add_argument(
        "--mujoco-viewer",
        action="store_true",
        help="Open the MuJoCo viewer while running the optional suite.",
    )
    parser.add_argument(
        "--mujoco-viewer-first-rollout-only",
        action="store_true",
        help="Open the MuJoCo viewer only for rollout 0 of each scenario.",
    )
    parser.add_argument("--mujoco-viewer-dt", type=float, default=0.02)
    parser.add_argument("--mujoco-real-time-factor", type=float, default=1.0)
    parser.add_argument("--max-vel-err", type=float, default=0.45)
    parser.add_argument("--max-yaw-err", type=float, default=0.45)
    parser.add_argument("--max-tilt-xy", type=float, default=0.45)
    parser.add_argument("--min-base-height", type=float, default=0.28)
    parser.add_argument("--max-ctrl-abs", type=float, default=8.0)
    parser.add_argument(
        "--max-non-foot-terrain-contact-step-fraction",
        type=float,
        default=0.15,
        help=(
            "Maximum allowed fraction of rollout steps with non-foot robot geoms "
            "touching terrain. Persistent values indicate leg/base terrain wedging."
        ),
    )
    parser.add_argument("--continue-on-error", action="store_true")
    return parser.parse_args()


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text())


def _artifact(bundle_dir: Path, manifest: dict, suffix: str) -> Path:
    for name in manifest.get("exported_artifacts", []):
        if str(name).endswith(suffix):
            path = bundle_dir / str(name)
            if path.exists():
                return path
    raise FileNotFoundError(f"Missing artifact ending with {suffix!r} in {bundle_dir}")


def _run_step(name: str, cmd: list[str], *, check: bool = False) -> dict:
    completed = subprocess.run(cmd, capture_output=True, text=True)
    ok = completed.returncode == 0
    if check and not ok:
        raise RuntimeError(f"{name} failed with returncode={completed.returncode}")
    return {
        "name": name,
        "ok": ok,
        "returncode": completed.returncode,
        "cmd": cmd,
        "stdout_tail": completed.stdout[-4000:] if completed.stdout else "",
        "stderr_tail": completed.stderr[-4000:] if completed.stderr else "",
    }


def _contract_checks(args: argparse.Namespace, bundle_dir: Path, manifest: dict, metadata: dict, deploy_cfg: dict) -> list[dict]:
    tensor_contract = metadata["tensor_contract"]
    obs_cfg = deploy_cfg["observations"]
    policy_order = obs_cfg["policy_order"]
    checks = []

    def add(name: str, ok: bool, detail: str, blocking: bool = True) -> None:
        checks.append({"name": name, "ok": bool(ok), "detail": detail, "blocking": bool(blocking)})

    policy_dim = int(tensor_contract["policy_obs_dim"])
    history_dim = int(tensor_contract["policy_history_dim"])
    history_len = int(obs_cfg["policy_history_length"])
    action_dim = int(tensor_contract["action_dim"])
    order_dim = sum(int(term["dim"]) for term in policy_order)
    expected_history_dim = policy_dim * history_len

    add("policy_kind", manifest.get("policy_kind") == "blind_history_policy", f"found={manifest.get('policy_kind')}")
    add("policy_obs_dim_matches_metadata", int(obs_cfg["policy_dim"]) == policy_dim, f"deploy={obs_cfg['policy_dim']} meta={policy_dim}")
    add("policy_order_dim_sum", order_dim == policy_dim, f"order_sum={order_dim} policy_dim={policy_dim}")
    add("history_dim_consistent", history_dim == expected_history_dim, f"history_dim={history_dim} expected={expected_history_dim}")
    add("action_dim", action_dim == args.expected_action_dim, f"found={action_dim} expected={args.expected_action_dim}")

    if args.expected_policy_obs_dim > 0:
        add(
            "expected_policy_obs_dim",
            policy_dim == args.expected_policy_obs_dim,
            f"found={policy_dim} expected={args.expected_policy_obs_dim}",
        )
    if args.expected_history_length > 0:
        add(
            "expected_history_length",
            history_len == args.expected_history_length,
            f"found={history_len} expected={args.expected_history_length}",
        )

    names = [str(term["name"]) for term in policy_order]
    add("no_duplicate_observation_terms", len(names) == len(set(names)), f"policy_order={names}")
    if policy_dim == 45:
        add("mjlab_no_base_lin_vel", "base_lin_vel" not in names, f"policy_order={names}")
    if policy_dim == 48:
        add("c1_has_base_lin_vel", "base_lin_vel" in names, f"policy_order={names}")

    return checks


def _torchscript_smoke(policy_path: Path, metadata: dict) -> dict:
    tensor_contract = metadata["tensor_contract"]
    policy_dim = int(tensor_contract["policy_obs_dim"])
    history_dim = int(tensor_contract["policy_history_dim"])
    action_dim = int(tensor_contract["action_dim"])
    policy = torch.jit.load(str(policy_path), map_location="cpu")
    policy.eval()
    with torch.inference_mode():
        output = policy(torch.zeros(1, policy_dim), torch.zeros(1, history_dim))
    ok = tuple(output.shape) == (1, action_dim) and bool(torch.isfinite(output).all())
    return {
        "name": "torchscript_forward_smoke",
        "ok": ok,
        "output_shape": list(output.shape),
        "output_finite": bool(torch.isfinite(output).all()),
        "output_abs_max": float(output.abs().max()),
    }


def _behavior_checks(args: argparse.Namespace, suite_summary_path: Path | None) -> list[dict]:
    if suite_summary_path is None or not suite_summary_path.exists():
        return []

    suite = _load_json(suite_summary_path)
    checks: list[dict] = []

    def add(name: str, ok: bool, detail: str, blocking: bool = True) -> None:
        checks.append({"name": name, "ok": bool(ok), "detail": detail, "blocking": bool(blocking)})

    add(
        "mujoco_suite_complete",
        suite.get("status") == "complete",
        f"status={suite.get('status')} scenario_count={suite.get('scenario_count')} expected={suite.get('expected_scenario_count')}",
    )

    for row in suite.get("results", []):
        scenario = str(row.get("scenario"))
        successful = int(row.get("successful_rollouts") or 0)
        total = int(row.get("rollout_count") or 0)
        vel_err = float(row.get("vel_err_step_mean") or 0.0)
        yaw_err = float(row.get("yaw_err_step_mean") or 0.0)
        tilt = float(row.get("base_tilt_projected_gravity_xy_mean") or 0.0)
        base_height = float(row.get("base_height_mean") or 0.0)
        ctrl_abs = float(row.get("ctrl_abs_mean") or 0.0)
        non_foot_contact_frac = float(row.get("non_foot_terrain_contact_step_fraction") or 0.0)

        add(f"{scenario}:all_rollouts_successful", successful == total and total > 0, f"successful={successful}/{total}")
        add(f"{scenario}:vel_err", vel_err <= args.max_vel_err, f"{vel_err:.4f} <= {args.max_vel_err:.4f}")
        add(f"{scenario}:yaw_err", yaw_err <= args.max_yaw_err, f"{yaw_err:.4f} <= {args.max_yaw_err:.4f}")
        add(f"{scenario}:tilt_xy", tilt <= args.max_tilt_xy, f"{tilt:.4f} <= {args.max_tilt_xy:.4f}")
        add(f"{scenario}:base_height", base_height >= args.min_base_height, f"{base_height:.4f} >= {args.min_base_height:.4f}")
        add(f"{scenario}:ctrl_abs", ctrl_abs <= args.max_ctrl_abs, f"{ctrl_abs:.4f} <= {args.max_ctrl_abs:.4f}")
        add(
            f"{scenario}:non_foot_terrain_contact_step_fraction",
            non_foot_contact_frac <= args.max_non_foot_terrain_contact_step_fraction,
            f"{non_foot_contact_frac:.4f} <= {args.max_non_foot_terrain_contact_step_fraction:.4f}",
        )

    return checks


def main() -> int:
    args = parse_args()
    bundle_dir = Path(args.bundle_dir).resolve()
    manifest = _load_json(bundle_dir / "bundle_manifest.json")
    metadata = _load_json(_artifact(bundle_dir, manifest, ".export_metadata.json"))
    deploy_cfg = _load_json(_artifact(bundle_dir, manifest, ".deploy_config.json"))
    policy_path = _artifact(bundle_dir, manifest, ".torchscript.pt")

    bundle_name = args.policy_name or bundle_dir.name
    output_root = args.output_dir if args.output_dir.is_absolute() else (Path.cwd() / args.output_dir).resolve()
    output_dir = output_root / bundle_name
    output_dir.mkdir(parents=True, exist_ok=True)

    steps: list[dict] = []
    steps.append(
        _run_step(
            "bundle_structural_validation",
            [args.python_exe, str(VALIDATE_BUNDLE), "--bundle-dir", str(bundle_dir)],
        )
    )
    contract_checks = _contract_checks(args, bundle_dir, manifest, metadata, deploy_cfg)
    steps.append(_torchscript_smoke(policy_path, metadata))
    steps.append(
        _run_step(
            "golden_inference_parity",
            [
                args.python_exe,
                str(VALIDATE_INFERENCE_PARITY),
                "--bundle-dir",
                str(bundle_dir),
                "--output-dir",
                str(output_dir / "golden_inference"),
            ],
        )
    )
    steps.append(
        _run_step(
            "mujoco_preflight",
            [
                args.python_exe,
                str(RUN_SIM2SIM),
                "--bundle-dir",
                str(bundle_dir),
                "--strict",
                "--actuator-model",
                "isaac_dc_motor",
            ],
        )
    )
    steps.append(
        _run_step(
            "unitree_mjlab_fsm_runtime_audit",
            [
                args.python_exe,
                str(VALIDATE_UNITREE_MJLAB_FSM),
                "--expected-policy-name",
                str(manifest.get("policy_name", bundle_name)),
                "--strict-fixstand-gains",
                "--json-out",
                str(output_dir / "unitree_mjlab_fsm_runtime_audit.json"),
            ],
        )
    )
    mujoco_suite_summary = None
    if args.run_mujoco_suite:
        suite_output = output_dir / "mujoco_suite"
        suite_step = _run_step(
            "mujoco_moderate_suite",
            [
                args.python_exe,
                str(RUN_MUJOCO_SUITE),
                "--bundle-dir",
                str(bundle_dir),
                "--suite",
                args.mujoco_suite,
                "--num-rollouts",
                str(args.mujoco_rollouts),
                "--max-steps",
                str(args.mujoco_max_steps),
                "--trace-steps",
                str(args.mujoco_trace_steps),
                "--actuator-model",
                "isaac_dc_motor",
                "--output-dir",
                str(suite_output),
                *(
                    [
                        "--viewer",
                        "--viewer-dt",
                        str(args.mujoco_viewer_dt),
                        "--real-time-factor",
                        str(args.mujoco_real_time_factor),
                    ]
                    if args.mujoco_viewer
                    else []
                ),
                *(
                    ["--viewer-first-rollout-only"]
                    if args.mujoco_viewer_first_rollout_only
                    else []
                ),
                *(
                    ["--continue-on-error"]
                    if args.continue_on_error
                    else []
                ),
            ],
        )
        steps.append(suite_step)
        summary_path = suite_output / bundle_dir.name / args.mujoco_suite / "suite_summary.json"
        if summary_path.exists():
            mujoco_suite_summary = str(summary_path)

    behavior_checks = _behavior_checks(args, Path(mujoco_suite_summary) if mujoco_suite_summary else None)
    all_contract_ok = all(check["ok"] or not check["blocking"] for check in contract_checks)
    all_required_steps_ok = all(step["ok"] for step in steps if step["name"] != "mujoco_preflight")
    mujoco_preflight = next(step for step in steps if step["name"] == "mujoco_preflight")
    all_behavior_ok = all(check["ok"] or not check["blocking"] for check in behavior_checks)
    status = (
        "pass"
        if all_contract_ok and all_required_steps_ok and mujoco_preflight["ok"] and all_behavior_ok
        else "blocked"
    )
    blockers = [
        f"contract:{check['name']}:{check['detail']}"
        for check in contract_checks
        if not check["ok"] and check["blocking"]
    ]
    blockers += [
        f"behavior:{check['name']}:{check['detail']}"
        for check in behavior_checks
        if not check["ok"] and check["blocking"]
    ]
    blockers += [
        f"step:{step['name']}:returncode={step['returncode']}"
        for step in steps
        if not step["ok"]
    ]

    report = {
        "status": status,
        "bundle_dir": str(bundle_dir),
        "bundle_name": bundle_name,
        "manifest": manifest,
        "tensor_contract": metadata["tensor_contract"],
        "deploy_observation_order": [term["name"] for term in deploy_cfg["observations"]["policy_order"]],
        "contract_checks": contract_checks,
        "behavior_checks": behavior_checks,
        "behavior_thresholds": {
            "max_vel_err": args.max_vel_err,
            "max_yaw_err": args.max_yaw_err,
            "max_tilt_xy": args.max_tilt_xy,
            "min_base_height": args.min_base_height,
            "max_ctrl_abs": args.max_ctrl_abs,
            "max_non_foot_terrain_contact_step_fraction": args.max_non_foot_terrain_contact_step_fraction,
        },
        "steps": steps,
        "blockers": blockers,
        "mujoco_suite_summary": mujoco_suite_summary,
        "policy_path": str(policy_path),
        "report_dir": str(output_dir),
    }
    report_path = output_dir / "validation_gate_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if status == "pass" else 2


if __name__ == "__main__":
    raise SystemExit(main())
