# Go2 policy-bundle comparison

Baseline: `go2_blind_rough_asymppo_mjlab_v1_candidate`  
Candidate: `go2_blind_rough_combined_asymppo_steps_v1_candidate`

## Decision summary

The deployment interface is **compatible**: observation layout/dimensions, history, action mapping, action conversion, command contract, and timing are unchanged. A deployment regression therefore is unlikely to be caused by a bundle-interface mismatch; first investigate runtime sensor/command conventions and the changed learned weights/training distribution.

The current observation is 45 floats: base_ang_vel (3), projected_gravity (3), velocity_commands (3), joint_pos_rel (12), joint_vel_rel (12), last_action (12). History holds 100 frames = 2.0 s at 50 Hz, with the documented `isaaclab_term_major` layout.


## Runtime contract

| Field | Baseline | Candidate | Status |
| --- | --- | --- | --- |
| Policy kind | blind_history_policy | blind_history_policy | same |
| Deployable observation groups | `["policy","policy_history"]` | `["policy","policy_history"]` | same |
| Policy input: current observation | `["batch",45]` | `["batch",45]` | same |
| Policy input: observation history | `["batch",4500]` | `["batch",4500]` | same |
| Policy output: action | `["batch",12]` | `["batch",12]` | same |
| Action dimension | 12 | 12 | same |
| Latent dimension | 0 | 0 | same |

## Observation, action, command, and timing contract

| Field | Baseline | Candidate | Status |
| --- | --- | --- | --- |
| Control timestep (s; 50 Hz) | 0.02 | 0.02 | same |
| Control decimation | 4 | 4 | same |
| Physics timestep | 0.005 | 0.005 | same |
| Policy observation dimension | 45 | 45 | same |
| History length (frames) | 100 | 100 | same |
| History vector dimension | 4500 | 4500 | same |
| History layout | isaaclab_term_major | isaaclab_term_major | same |
| Use gym history | False | False | same |
| Observation term order | `[{"dim":3,"history_length":1,"name":"base_ang_vel","scale":[1.0,1.0,1.0]},{"dim":3,"history_length":1,"name":"projected_gravity","scale":[1.0,1.0,1.0]},{"dim":3,"history_length":1,"name":"velocity_commands","scale":[1.0,1.0,1.0]},{"dim":12,"history_length":1,"name":"joint_pos_rel","scale":[1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]},{"dim":12,"history_length":1,"name":"joint_vel_rel","scale":[1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]},{"dim":12,"history_length":1,"name":"last_action","scale":[1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]}]` | `[{"dim":3,"history_length":1,"name":"base_ang_vel","scale":[1.0,1.0,1.0]},{"dim":3,"history_length":1,"name":"projected_gravity","scale":[1.0,1.0,1.0]},{"dim":3,"history_length":1,"name":"velocity_commands","scale":[1.0,1.0,1.0]},{"dim":12,"history_length":1,"name":"joint_pos_rel","scale":[1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]},{"dim":12,"history_length":1,"name":"joint_vel_rel","scale":[1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]},{"dim":12,"history_length":1,"name":"last_action","scale":[1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]}]` | same |
| Action type | JointPositionAction | JointPositionAction | same |
| Action joint names/order | `["FL_hip_joint","FR_hip_joint","RL_hip_joint","RR_hip_joint","FL_thigh_joint","FR_thigh_joint","RL_thigh_joint","RR_thigh_joint","FL_calf_joint","FR_calf_joint","RL_calf_joint","RR_calf_joint"]` | `["FL_hip_joint","FR_hip_joint","RL_hip_joint","RR_hip_joint","FL_thigh_joint","FR_thigh_joint","RL_thigh_joint","RR_thigh_joint","FL_calf_joint","FR_calf_joint","RL_calf_joint","RR_calf_joint"]` | same |
| Action joint IDs/order | `[0,1,2,3,4,5,6,7,8,9,10,11]` | `[0,1,2,3,4,5,6,7,8,9,10,11]` | same |
| Action scale | `[0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25]` | `[0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25]` | same |
| Action offset/default pose | `[0.1,-0.1,0.1,-0.1,0.8,0.8,1.0,1.0,-1.5,-1.5,-1.5,-1.5]` | `[0.1,-0.1,0.1,-0.1,0.8,0.8,1.0,1.0,-1.5,-1.5,-1.5,-1.5]` | same |
| Action clipping | `[[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0]]` | `[[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0],[-100.0,100.0]]` | same |
| PD joint stiffness | `[25.0,25.0,25.0,25.0,25.0,25.0,25.0,25.0,25.0,25.0,25.0,25.0]` | `[25.0,25.0,25.0,25.0,25.0,25.0,25.0,25.0,25.0,25.0,25.0,25.0]` | same |
| PD joint damping | `[0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5]` | `[0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5]` | same |
| Joint effort limit | `[23.5,23.5,23.5,23.5,23.5,23.5,23.5,23.5,23.5,23.5,23.5,23.5]` | `[23.5,23.5,23.5,23.5,23.5,23.5,23.5,23.5,23.5,23.5,23.5,23.5]` | same |
| Joint velocity limit | `[30.0,30.0,30.0,30.0,30.0,30.0,30.0,30.0,30.0,30.0,30.0,30.0]` | `[30.0,30.0,30.0,30.0,30.0,30.0,30.0,30.0,30.0,30.0,30.0,30.0]` | same |
| Default velocity command | `[0.5,0.0,0.0]` | `[0.5,0.0,0.0]` | same |
| Velocity command ranges | `{"ang_vel_z":[-0.6,0.6],"lin_vel_x":[-0.8,0.8],"lin_vel_y":[-0.3,0.3]}` | `{"ang_vel_z":[-0.6,0.6],"lin_vel_x":[-0.8,0.8],"lin_vel_y":[-0.3,0.3]}` | same |

## Provenance and binary identity

| Field | Baseline | Candidate | Status |
| --- | --- | --- | --- |
| Training phase | blind-rough-mjlab-asymppo-v1 | combined-asymppo-stairs-v1 | DIFFERENT |
| Task | Go2-Blind-Rough-MJLAB-AsymPPO-V1 | Go2-Blind-Rough-MJLAB-Combined-AsymPPO-Steps-V1 | DIFFERENT |
| Source checkpoint | /home/bhuvan/tools/IsaacLab/logs/rsl_rl/go2_blind_rough_asymppo_mjlab_v1/2026-06-04_10-31-03/model_1999.pt | logs/rsl_rl/go2_blind_rough_combined_asymppo_steps_v1/2026-07-16_10-14-29/model_5099.pt | DIFFERENT |
| TorchScript SHA-256 | 454afb874ef196cf7187775729165a30224daf0e99dc89ffc5ce709e02ec7f55 | 75df94a39acdd86f9e0bea320c0cba797cde1b826488e307b5f5dfe6812dd7ac | DIFFERENT |
| ONNX SHA-256 | 64839040e43fc19b2f158a953a124d852e8e2f98a93e11641628837f213f0ee7 | d14e43aa00326091aa4a847d4a43a62f588ac3d71aab6bd83b7c0960a49cf775 | DIFFERENT |

## TorchScript comparison

- Architecture / parameter names: **same**
- Parameter count: **251,404** in each model
- Learned tensors changed: **14/14**

| Tensor | Shape | Mean absolute delta | Max absolute delta |
| --- | --- | ---: | ---: |
| `actor.0.bias` | `(512,)` | 0.046178 | 0.197512 |
| `actor.0.weight` | `(512, 109)` | 0.081652 | 1.724745 |
| `actor.2.bias` | `(256,)` | 0.044919 | 0.221940 |
| `actor.2.weight` | `(256, 512)` | 0.082738 | 0.586216 |
| `actor.4.bias` | `(128,)` | 0.039459 | 0.172247 |
| `actor.4.weight` | `(128, 256)` | 0.077142 | 0.595160 |
| `actor.6.bias` | `(12,)` | 0.024773 | 0.061784 |
| `actor.6.weight` | `(12, 128)` | 0.036312 | 0.261707 |
| `history_projection.0.bias` | `(64,)` | 0.015270 | 0.037222 |
| `history_projection.0.weight` | `(64, 128)` | 0.041434 | 0.268872 |
| `temporal_encoder.0.bias` | `(64,)` | 0.026523 | 0.072032 |
| `temporal_encoder.0.weight` | `(64, 45, 3)` | 0.037850 | 0.221172 |
| `temporal_encoder.2.bias` | `(64,)` | 0.021646 | 0.069900 |
| `temporal_encoder.2.weight` | `(64, 64, 3)` | 0.043786 | 0.367285 |

Fixed synthetic-input probe (not a physical rollout): action mean-|delta| = **0.528462**, max-|delta| = **1.133174**.

## Debugging implications

1. Do **not** swap observation fields, joint order, scales, offsets, or control period between these two bundles: the checked-in contracts are identical.
2. The model is a blind history policy. Preserve all 100 frames and `isaaclab_term_major` ordering; a runtime history-buffer bug affects both bundles but can expose different learned sensitivity.
3. Since all learned tensors differ despite an identical architecture, compare actions on recorded deployment observations and replay the same traces through both models. Large divergences localize the issue to policy behavior/training rather than the adapter contract.
4. The candidate is a combined/stairs-trained checkpoint, while the baseline is the earlier blind-rough checkpoint. Terrain/task-distribution behavior is the primary intentional difference documented by the bundles.

## Reproduce

```bash
python3 go2/scripts/deploy/compare_policy_bundles.py \
  go2/policies/go2_blind_rough_asymppo_mjlab_v1_candidate \
  go2/policies/go2_blind_rough_combined_asymppo_steps_v1_candidate \
  --output 'go2/Docs /policy_bundle_comparison.md'
```
