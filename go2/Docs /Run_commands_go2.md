# Go2 Deployment Run Commands

This document contains the canonical deployment workflow for the Go2 AsymPPO policy. It covers environment setup, simulation, validation, hardware deployment, monitoring, and network configuration.

---

# Repository Overview

The deployment pipeline consists of four major components.

| Component | Purpose |
|---------------------------------------------|---------------------------------------------------------------------------|
| **deploy**                                  | Deployment wrappers, validation, networking, monitoring                   |
| **reference_repos/Unitree_mjlab_repo**      | MuJoCo simulator and C++ FSM controller                                   |
| **reference_repos/sim2real_unitree_sdk2py** | Python SDK used by deployment utilities (DDS probe, monitor, diagnostics) |

---

# Environment Setup

```bash

export REPO=$PWD

export GO2_NET_IF=<robot-facing-interface>
export GO2_ETH_IF=<ethernet-interface>
export GO2_WIFI_IF=<wifi-interface>

export RMA_MUJOCO_PYTHON=python
export MUJOCO_PYTHON=$RMA_MUJOCO_PYTHON

export go2_hw_python=python3

export GO2_USD_PATH=/path/to/go2.usd

export ASYMPPO_CKPT=/path/to/model_1999.pt

export ASYMPPO_BUNDLE=$REPO/policy/go2_blind_rough_asymppo_mjlab_v1_candidate
```

---

# Environment Variable Description

|    Variable    | Description                                  |
|----------------|----------------------------------------------|
| REPO           | Repository root                              |
| GO2_NET_IF     | Network interface used by the controller     |
| GO2_ETH_IF     | Ethernet interface                           |
| GO2_WIFI_IF    | Wi-Fi interface                              |
| GO2_HW_PYTHON  | Python environment containing unitree_sdk2py |
| GO2_USD_PATH   | Go2 USD asset used by IsaacLab               |
| ASYMPPO_BUNDLE | Deployment-ready exported policy             |

---
## Unitree MuJoCo FSM


If `reference_repos/unitree_rl_mjlab` was restored from upstream or deleted
during cleanup, reapply the local simulated-joystick/runtime patch first:

```bash
cd $REPO/reference_repos/unitree_rl_mjlab
git apply ../../patches/unitree_rl_mjlab/go2_scripted_controller.patch
cd $REPO
```


Build the C++ controller and simulator if they are missing:

```bash
bash $REPO/go2/scripts/deploy/build_unitree_mjlab_runtime.sh all
```


Activate and validate the frozen AsymPPO runtime:

```bash
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh activate
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh validate
```

Start these in separate terminals:

```bash
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh controller
```

```bash
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh sim
```

## Real Go2 Network Selection

The controller binds CycloneDDS to one interface at process startup. Transport
switching must be done while the robot is in Passive and the controller is
stopped. There is intentionally no live failover during torque control.



# Hardware Deployment




## Ethernet

### Preflight

```bash
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh activate

bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh validate

bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh network-status ethernet

bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh dds-probe ethernet
```

### Start Controller

```bash
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh hardware ethernet
```

---






## Wi-Fi

Configure the robot

```bash
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh \
robot-wifi-connect <GO2_ETHERNET_IP> <SSID>
```

Connect the laptop

```bash
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh \
wifi-scan $GO2_WIFI_IF

bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh \
wifi-connect <GO2_DONGLE_SSID> $GO2_WIFI_IF
```

Verify connectivity

```bash
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh \
network-status wifi

bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh \
wifi-peer <GO2_WIFI_IP> $GO2_WIFI_IF

bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh \
dds-probe wifi
```

Start the controller

```bash
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh hardware wifi
```

---

# Automatic Interface Selection

Specific interface

```bash
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh \
hardware $GO2_NET_IF
```

Automatic interface selection

```bash
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh \
hardware auto
```

---

# Robot Remote Commands

| Button  | Action                 |
|---------|------------------------|
| L2 + Up | FixStand               |
| R2 + A  | Start Velocity Policy  |
| L2 + B  | Passive                |

---

# Real-Time Monitor

Ethernet

```bash
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh \
monitor ethernet asymppo_walk
```

Wi-Fi

```bash
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh \
monitor wifi asymppo_wifi_walk
```

The monitor is read-only and subscribes to

- rt/lowstate
- rt/lowcmd
- rt/sportmodestate

Captured logs are stored in

```text
artifacts/go2_realtime_monitor/
```

---

# Telemetry Analysis

Analyze a monitor capture

```bash
$RMA_MUJOCO_PYTHON \
scripts/deploy/analyze_go2_realtime_monitor.py \
--jsonl artifacts/go2_realtime_monitor/<capture>.jsonl
```

Analyze mirrored legs

```bash
$RMA_MUJOCO_PYTHON \
scripts/deploy/analyze_go2_leg_mirror_pairs.py \
--jsonl artifacts/go2_realtime_monitor/<capture>.jsonl
```

---

# Network Diagnostics

Ethernet

```bash
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh \
network-status ethernet
```

Wi-Fi

```bash
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh \
network-status wifi
```

DDS Probe

```bash
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh \
dds-probe ethernet
```

or

```bash
bash $REPO/go2/scripts/deploy/run_unitree_mjlab_sim_deploy.sh \
dds-probe wifi
```

---

# Troubleshooting

| Problem | Cause                                                                                       |
|----------|-------|
| `wifi-peer` fails | Robot not on WLAN, incorrect IP, or client isolation |
| `dds-probe` fails after successful `wifi-peer` | DDS multicast blocked or Go2 DDS not exposed on Wi-Fi |
| `hardware` should not be started | DDS probe does not receive `rt/lowstate` |

For Wi-Fi deployment:

- Use a network that allows multicast.
- Disable AP/client isolation.
- Avoid guest networks.
- Ensure both the laptop and the Go2 Wi-Fi dongle are connected to the same WLAN.