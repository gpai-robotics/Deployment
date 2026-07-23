"""Shared schema helpers for Go2 low-level monitor artifacts."""

from __future__ import annotations

from typing import Any

import numpy as np


SCHEMA_NAME = "go2_lowlevel_monitor"
SCHEMA_VERSION = 2

POLICY_JOINT_NAMES = [
    "FL_hip",
    "FR_hip",
    "RL_hip",
    "RR_hip",
    "FL_thigh",
    "FR_thigh",
    "RL_thigh",
    "RR_thigh",
    "FL_calf",
    "FR_calf",
    "RL_calf",
    "RR_calf",
]

# Policy index i reads SDK motor index POLICY_TO_SDK[i].
POLICY_TO_SDK = np.asarray([3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8], dtype=np.int32)

JOINT_VECTOR_KEYS = ("q", "q_des", "q_err", "joint_vel", "tau_est", "temperature")


def sdk_vector_to_policy(values: Any) -> list[float]:
    """Convert a 12-element SDK-order vector to the deployed policy order."""
    array = np.asarray(values, dtype=np.float32)
    if array.shape != (12,):
        raise ValueError(f"Expected a 12-element joint vector, got shape {array.shape}")
    return array[POLICY_TO_SDK].tolist()


def normalize_payload_joint_order(payload: dict[str, Any]) -> tuple[dict[str, Any], bool]:
    """Return policy-ordered telemetry and whether a legacy remap was applied.

    Monitor artifacts written before schema version 2 stored raw SDK vectors
    while labeling them as policy order. They are remapped here so old captures
    can still be analyzed correctly.
    """
    schema = payload.get("schema", {})
    order = schema.get("joint_order")
    if order == "policy":
        return payload, False

    latest = payload.get("latest", {})
    for key in JOINT_VECTOR_KEYS:
        values = latest.get(key)
        if values is not None:
            latest[key] = sdk_vector_to_policy(values)
    return payload, True
