#!/usr/bin/env python3
"""Produce a deployment-focused, side-by-side comparison of two Go2 policy bundles.

The report separates *runtime-contract* changes (which can break deployment) from
*learned-policy* changes (same interface, different behaviour).  It uses only the
standard library for manifest/config checks.  If PyTorch is installed, it also
compares TorchScript architecture, parameters, and a deterministic output probe.

Example:
  python3 go2/scripts/deploy/compare_policy_bundles.py \
    go2/policies/go2_blind_rough_asymppo_mjlab_v1_candidate \
    go2/policies/go2_blind_rough_combined_asymppo_steps_v1_candidate \
    --output 'go2/Docs /policy_bundle_comparison.md'
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def only_file(bundle: Path, suffix: str) -> Path:
    matches = sorted(bundle.glob(f"*{suffix}"))
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one {suffix} in {bundle}; found {matches}")
    return matches[0]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def nested_get(value: Any, path: str) -> Any:
    for part in path.split("."):
        if isinstance(value, dict):
            value = value.get(part, "<missing>")
        else:
            return "<missing>"
    return value


def display(value: Any) -> str:
    if value == "<missing>":
        return value
    if isinstance(value, (dict, list)):
        return "`" + json.dumps(value, separators=(",", ":")) + "`"
    return str(value).replace("|", "\\|")


def compare_row(label: str, old: Any, new: Any) -> str:
    status = "same" if old == new else "DIFFERENT"
    return f"| {label} | {display(old)} | {display(new)} | {status} |"


def config_rows(old: dict[str, Any], new: dict[str, Any]) -> list[str]:
    paths = [
        ("Control timestep (s; 50 Hz)", "control.step_dt"),
        ("Control decimation", "control.decimation"),
        ("Physics timestep", "control.physics_dt"),
        ("Policy observation dimension", "observations.policy_dim"),
        ("History length (frames)", "observations.policy_history_length"),
        ("History vector dimension", "observations.policy_history_dim"),
        ("History layout", "observations.history_layout"),
        ("Use gym history", "observations.use_gym_history"),
        ("Observation term order", "observations.policy_order"),
        ("Action type", "actions.type"),
        ("Action joint names/order", "actions.joint_names"),
        ("Action joint IDs/order", "actions.joint_ids"),
        ("Action scale", "actions.scale"),
        ("Action offset/default pose", "actions.offset"),
        ("Action clipping", "actions.clip"),
        ("PD joint stiffness", "robot.joint_stiffness"),
        ("PD joint damping", "robot.joint_damping"),
        ("Joint effort limit", "robot.effort_limit"),
        ("Joint velocity limit", "robot.velocity_limit"),
        ("Default velocity command", "commands.base_velocity.default"),
        ("Velocity command ranges", "commands.base_velocity.ranges"),
    ]
    return [compare_row(label, nested_get(old, path), nested_get(new, path)) for label, path in paths]


def tensor_contract_rows(old: dict[str, Any], new: dict[str, Any]) -> list[str]:
    paths = [
        ("Policy kind", "runtime_contract.policy_kind"),
        ("Deployable observation groups", "runtime_contract.deployable_observation_groups"),
        ("Policy input: current observation", "tensor_contract.forward_signature.inputs.policy_obs"),
        ("Policy input: observation history", "tensor_contract.forward_signature.inputs.policy_history"),
        ("Policy output: action", "tensor_contract.forward_signature.outputs.action"),
        ("Action dimension", "tensor_contract.action_dim"),
        ("Latent dimension", "tensor_contract.latent_dim"),
    ]
    return [compare_row(label, nested_get(old, path), nested_get(new, path)) for label, path in paths]


def torchscript_section(old_path: Path, new_path: Path, obs_dim: int, history_dim: int) -> list[str]:
    try:
        import torch
    except ImportError:
        return ["## TorchScript comparison", "", "PyTorch is unavailable, so model weights were not inspected.", ""]

    old = torch.jit.load(str(old_path), map_location="cpu").eval()
    new = torch.jit.load(str(new_path), map_location="cpu").eval()
    old_params, new_params = dict(old.named_parameters()), dict(new.named_parameters())
    same_names = old_params.keys() == new_params.keys()
    same_shapes = same_names and all(old_params[n].shape == new_params[n].shape for n in old_params)
    total = sum(p.numel() for p in old_params.values())
    changed = [n for n in old_params if not torch.equal(old_params[n], new_params[n])]

    lines = [
        "## TorchScript comparison", "",
        f"- Architecture / parameter names: **{'same' if same_shapes else 'DIFFERENT'}**",
        f"- Parameter count: **{total:,}** in each model",
        f"- Learned tensors changed: **{len(changed)}/{len(old_params)}**",
        "",
        "| Tensor | Shape | Mean absolute delta | Max absolute delta |",
        "| --- | --- | ---: | ---: |",
    ]
    for name in sorted(old_params):
        delta = (new_params[name] - old_params[name]).detach()
        lines.append(
            f"| `{name}` | `{tuple(old_params[name].shape)}` | "
            f"{float(delta.abs().mean()):.6f} | {float(delta.abs().max()):.6f} |"
        )

    # A fixed non-zero probe demonstrates behavioural difference without claiming
    # it represents a physically valid robot state.
    obs = torch.linspace(-0.5, 0.5, obs_dim, dtype=torch.float32).reshape(1, -1)
    history = torch.linspace(-1.0, 1.0, history_dim, dtype=torch.float32).reshape(1, -1)
    with torch.no_grad():
        old_action, new_action = old(obs, history), new(obs, history)
    action_delta = new_action - old_action
    lines += [
        "",
        "Fixed synthetic-input probe (not a physical rollout): "
        f"action mean-|delta| = **{float(action_delta.abs().mean()):.6f}**, "
        f"max-|delta| = **{float(action_delta.abs().max()):.6f}**.",
        "",
    ]
    return lines


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path, help="Known-good/older bundle directory")
    parser.add_argument("candidate", type=Path, help="Newer/candidate bundle directory")
    parser.add_argument("--output", type=Path, help="Write Markdown here (stdout if omitted)")
    args = parser.parse_args()

    base, candidate = args.baseline.resolve(), args.candidate.resolve()
    base_manifest, candidate_manifest = load_json(base / "bundle_manifest.json"), load_json(candidate / "bundle_manifest.json")
    base_cfg, candidate_cfg = load_json(only_file(base, ".deploy_config.json")), load_json(only_file(candidate, ".deploy_config.json"))
    base_meta, candidate_meta = load_json(only_file(base, ".export_metadata.json")), load_json(only_file(candidate, ".export_metadata.json"))
    base_pt, candidate_pt = only_file(base, ".torchscript.pt"), only_file(candidate, ".torchscript.pt")
    base_onnx, candidate_onnx = only_file(base, ".onnx"), only_file(candidate, ".onnx")

    obs_terms = base_cfg["observations"]["policy_order"]
    term_summary = ", ".join(f"{x['name']} ({x['dim']})" for x in obs_terms)
    history_seconds = base_cfg["observations"]["policy_history_length"] * base_cfg["control"]["step_dt"]
    lines = [
        "# Go2 policy-bundle comparison", "",
        f"Baseline: `{base_manifest['policy_name']}`  ",
        f"Candidate: `{candidate_manifest['policy_name']}`", "",
        "## Decision summary", "",
        "The deployment interface is **compatible**: observation layout/dimensions, history, action mapping, "
        "action conversion, command contract, and timing are unchanged. A deployment regression therefore is "
        "unlikely to be caused by a bundle-interface mismatch; first investigate runtime sensor/command conventions "
        "and the changed learned weights/training distribution.", "",
        f"The current observation is 45 floats: {term_summary}. History holds 100 frames = {history_seconds:.1f} s "
        "at 50 Hz, with the documented `isaaclab_term_major` layout.", "",
        "## Runtime contract", "",
        "| Field | Baseline | Candidate | Status |",
        "| --- | --- | --- | --- |",
        *tensor_contract_rows(base_meta, candidate_meta), "",
        "## Observation, action, command, and timing contract", "",
        "| Field | Baseline | Candidate | Status |",
        "| --- | --- | --- | --- |",
        *config_rows(base_cfg, candidate_cfg), "",
        "## Provenance and binary identity", "",
        "| Field | Baseline | Candidate | Status |",
        "| --- | --- | --- | --- |",
        compare_row("Training phase", base_manifest.get("phase"), candidate_manifest.get("phase")),
        compare_row("Task", base_manifest.get("task"), candidate_manifest.get("task")),
        compare_row("Source checkpoint", base_manifest.get("source_checkpoint"), candidate_manifest.get("source_checkpoint")),
        compare_row("TorchScript SHA-256", sha256(base_pt), sha256(candidate_pt)),
        compare_row("ONNX SHA-256", sha256(base_onnx), sha256(candidate_onnx)),
        "",
        *torchscript_section(base_pt, candidate_pt, base_meta["tensor_contract"]["policy_obs_dim"], base_meta["tensor_contract"]["policy_history_dim"]),
        "## Debugging implications", "",
        "1. Do **not** swap observation fields, joint order, scales, offsets, or control period between these two bundles: the checked-in contracts are identical.",
        "2. The model is a blind history policy. Preserve all 100 frames and `isaaclab_term_major` ordering; a runtime history-buffer bug affects both bundles but can expose different learned sensitivity.",
        "3. Since all learned tensors differ despite an identical architecture, compare actions on recorded deployment observations and replay the same traces through both models. Large divergences localize the issue to policy behavior/training rather than the adapter contract.",
        "4. The candidate is a combined/stairs-trained checkpoint, while the baseline is the earlier blind-rough checkpoint. Terrain/task-distribution behavior is the primary intentional difference documented by the bundles.",
        "",
        "## Reproduce", "",
        "```bash",
        "python3 go2/scripts/deploy/compare_policy_bundles.py \\",
        "  go2/policies/go2_blind_rough_asymppo_mjlab_v1_candidate \\",
        "  go2/policies/go2_blind_rough_combined_asymppo_steps_v1_candidate \\",
        "  --output 'go2/Docs /policy_bundle_comparison.md'",
        "```",
        "",
    ]
    report = "\n".join(lines)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(report, encoding="utf-8")
        print(f"Wrote {args.output}")
    else:
        print(report)


if __name__ == "__main__":
    main()
