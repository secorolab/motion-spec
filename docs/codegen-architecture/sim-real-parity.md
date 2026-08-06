<!-- SPDX-License-Identifier: MPL-2.0 -->
# Sim/real parity — the control law and its platform seams

Where the mj_kdl (sim) and robif2b (real) paths are allowed to differ, where they are not, and
the external-wrench law that unified them. Established 2026-08-06 across commits
`7a2a2f1..c56aacf`; the empirical record behind the wrench law is in `plans/codegen-architecture/1g-*`
and its branch history (`plan/1g`: `0ca431e` failure record → `052cfa4` fix).

## The fork surface

The backend template groups are the *only* fork points — every shared group (domain, assembly,
expression) forks through named dispatch rules, never through inline conditionals. Verified by
enumeration at `716eded`. Anything behavioural that forks must be one of the two categories
below; a fork that is neither is a defect.

### Platform-inherent (accepted, deliberate)

| seam | sim | real |
|---|---|---|
| clock (`clock-time-source`) | MuJoCo sim seconds | monotonic seconds |
| torque completion | explicit RNE pass | Kinova firmware (gravity + nature terms) |
| device layer | MuJoCo model | EtherCAT / serial drivers, `robot.toml` addressing |
| startup posture | joints set to `agent_homes` at setup | arm starts where it is; S_HOME drives there |
| run end | viewer close ends like S_DONE; `--headless --steps` bound | loop runs until the FSM finishes |

The gravity split is a **user decision (2026-08-06): do not "unify" it.** The firmware owns
gravity on hardware; sim has no firmware, so the RNE pass stands in for it.

### Shared by construction (never fork these)

- Loop pacing: both sleep to absolute deadlines (`sleep_until_next(_next_tick, period)`).
- Measured dt: both measure `dt_measured_s` from their clock, clamp to
  [`kDtClampMin`, `kDtClampMax`], and every integrator consumes it per call (`0766232`). In sim
  the measurement equals the nominal period by construction, so one code path serves both.
- FT processing: identical transform chain (sensor frame → reference point → as-seen-by) and
  identical tare (`bias − measured` after settle) on both sides.
- Telemetry: none, anywhere. The .pb frame log is the record (user decision 2026-08-06;
  `a3b73b2` deleted the mj_kdl-only stdout dump).
- **The constrained ACHD solve never sees `f_ext`** — the wrench law below.

## The external-wrench law (`c56aacf`)

Ruling (user, 2026-08-06): ACHD's constrained solve stays wrench-free — the wrench goes through
the dedicated `_fext` machinery; RNE-family passes may take it through their own `f_ext`
parameter. Per-tick, for an ACHD motion carrying a wrench (measured, or the virtual
elbow-support force):

```
real (robif2b)                              sim (mj_kdl)
──────────────                              ────────────
qdd, tau_c = ACHD(q,qd,α,β, f_ext=0, ff)    qdd, _ = ACHD(q,qd,α,β, f_ext=0, ff)
tau_w = ACHD_fext(q,qd, no-constraints, w)  tau  = RNE(q,qd,qdd, f_ext=w)
tau   = tau_c + tau_w                              = M·qdd + C + G − Jᵀw
firmware adds gravity + nature terms        (complete; RNE is the firmware stand-in)
```

Each backend carries the wrench through its own *torque completer*'s native interface. The
compositions that were tried and rejected — kept here so they are not re-invented:

- **`RNE(qdd,0) + tau_fext` (sim):** unstable — QACC blow-up at the elbow DOFs at S_PICK.
  `ChainHdSolver_Vereshchagin_Fext_FixedJoint` outputs *constraint-only* torque
  (header `:401`; the complete quantity is `getTotalTorque()`, `:413`). Adding a
  constraint-only torque to an already-complete RNE torque sums two different quantities.
- **`tau_c + tau_fext` verbatim in sim (robif2b's law):** the arm sags and never settles —
  both terms are constraint-only, and sim has no firmware to complete them. This is also why
  the real composition is internally consistent: firmware supplies the rest.
- Adding an explicit gravity term to the above would double-count: the generated
  `root_acc.vel = (0, 0, 9.81)` already hands gravity to Vereshchagin.

Facts about the `_fext` solver worth not re-deriving (third-party fork header):

- `f_ext` is **per-segment** — a mid-chain wrench (elbow support at `half_arm_2_link`) is
  applied where it attaches, not tip-loaded (verified down to `initial_upwards_sweep`).
- The header restricts `f_ext` to *"physical (but not artificial, i.e. not task-introduced)"*
  wrenches. The elbow-support force is a PID output — task-introduced — and both backends have
  always fed it through `f_ext` anyway. It works and is stable, but it is outside the solver's
  documented contract: **open item**, decide deliberately if it ever misbehaves.

RNE-driven motions are untouched by all of this: they always passed the wrench straight into
`rnea->CartToJnt(..., f_ext, ...)` on both backends, which the ruling sanctions.

## What the gate covers, and what it cannot

`pick_place_single` is the model that exercises the ACHD wrench law — 8 of its 10 motions
inject the virtual elbow support into `state.f_ext`. `admittance_arc_single`'s force motions
are RNE-driven; its generated controller is byte-identical under wrench-law changes, so it
gates nothing here. (Recorded because plan 1g's authors got this exactly backwards.)

Known residual asymmetries, out of codegen's hands:

- **Model asset mismatch:** Kinova bracelet mass is 0.95006 kg in the URDF (KDL side) and
  0.5 kg in the MJCF (sim plant) — skews RNE pricing vs the simulated plant, and FT/gravity
  paths differently in sim vs real. Fix belongs in the robot assets.
- The generated velocity-profile call path has no authoring model in the tree — parse-verified
  only.
