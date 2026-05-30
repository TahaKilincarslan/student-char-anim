# Design Notes — Character Plugin

## Motion Clip Selection

| Hotkey | Clip ID | Clip Name | Reason |
|--------|---------|-----------|--------|
| Q | 12 | `walk_forward` | Primary locomotion; plays during WASD movement |
| E | 47 | `push_two_handed` | Upper-body action for PUSH goals and door interaction |
| R | 83 | `climb_low_step` | Full-body action for CLIMB goals |

## PD Controller — Gain Rationale

The PD controller tracks `q_ref` (the clip's reference pose, read from `in_bones[]`) using a spring-damper model. Gains are tuned per joint based on the biomechanical role of each limb:

| Joint | Kp | Kd | Reasoning |
|-------|----|----|-----------|
| Upperarm L/R | 40 | 4.0 | Shoulder — naturally loose, allows arm swing and sway |
| Lowerarm L/R | 60 | 6.0 | Elbow — tighter tracking needed for hand placement goals |
| Thigh L/R | 45 | 4.5 | Hip — medium stiffness; supports weight but allows natural lean |
| Calf L/R | 70 | 7.0 | Knee — high stiffness; must stay close to IK target for plausible gait |
| Foot L/R | 65 | 6.5 | Ankle — high stiffness; foot contact quality depends on accurate placement |

**Damping ratio:** Kd ≈ Kp/10 gives near-critical damping for most joints without overshoot.

**Blend factor:** After PD integration, the output quaternion is slerped toward `q_ref` with `blend = min(1, Kp * dt * 0.5)`. This ensures stability at large timesteps and prevents runaway oscillation.

## Locomotion

- Walk speed: 1.4 m/s (matches `walk_forward` clip root motion velocity)
- Sprint speed: 4.0 m/s (held `LShift`)
- Strafe speed: 1.0 m/s (A/D keys)
- Facing direction: derived from `input->look_yaw_rad` (mouse look)
- Root rotation delta: yaw-only quaternion around Y axis

## Mission Goal Strategy

1. **GOTO**: navmesh path query on goal start → waypoint following → complete within `tolerance_m`
2. **PUSH**: navigate to object center within 1.0 m → report complete (host handles physics push)
3. **CLIMB**: navigate to object → report complete when `foot_l.world_y >= object.aabb.max_y`
4. **PICKUP**: navigate to object → report complete when `lowerarm_l.world_position` is inside object AABB
5. **INTERACT**: navigate to target → report complete within `tolerance_m + 0.5`

## Physics Integration

The physics pipeline per joint per tick:
1. Read `q_ref` from `in_bones[node_index]` (the host's clip sample)
2. Compute angular error: `q_err = q_ref * conj(q_prev)`
3. Estimate `ω_ref` by finite difference of q_ref
4. PD torque: `τ = Kp * err_axis − Kd * (ω_cur − ω_ref)`
5. Integrate: `ω_cur += τ * dt`, then dampen by 0.85, clamp to ±20 rad/s
6. Apply: `dq = normalize(ω * dt/2 + identity)`, `q_new = dq * q_prev`
7. Slerp `q_new` toward `q_ref` for stability
