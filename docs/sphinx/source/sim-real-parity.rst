.. SPDX-License-Identifier: MPL-2.0

Sim/real parity
===============

Where the mj_kdl (simulation) and robif2b (hardware) paths are allowed to differ, where they
are not, and the external-wrench law. Established 2026-08-06; the empirical record behind the
wrench law is in ``plans/codegen-architecture/1g-*`` and branch ``plan/1g``. The authoritative
background for the simulation torque path is mj_kdl_wrapper's
``docs/howto/torque_control.md`` — read it before touching any of this.

The fork surface
----------------

The backend template groups are the *only* fork points — every shared group (domain, assembly,
expression) forks through named dispatch rules, never inline conditionals. Anything behavioural
that forks must be one of the two categories below; a fork that is neither is a defect.

Platform-inherent (accepted, deliberate)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

===============================  =================================  ===================================
seam                             sim (mj_kdl)                       real (robif2b)
===============================  =================================  ===================================
clock (``clock-time-source``)    MuJoCo sim seconds                 monotonic seconds
torque completion                RNEA bridge to ``qfrc_applied``    Kinova firmware (gravity + nature)
device layer                     MuJoCo model                       EtherCAT/serial + ``robot.toml``
startup posture                  joints set to ``agent_homes``      arm starts where it is; S_HOME drives
run end                          viewer close ends like S_DONE      loop runs until the FSM finishes
===============================  =================================  ===================================

The gravity split is a user decision (2026-08-06): **do not unify it**. On hardware the
Kinova's inner loop supplies gravity and nature terms; MuJoCo simulates raw physics and
supplies nothing (``qfrc_applied`` bypasses the actuators entirely).

Shared by construction (never fork these)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- Loop pacing: both sleep to absolute deadlines (``sleep_until_next``).
- Measured dt: both measure ``dt_measured_s`` from their own clock, clamp to
  [``kDtClampMin``, ``kDtClampMax``], and every integrator consumes it per call. In sim the
  measurement equals the nominal period by construction — one code path serves both.
- FT processing: identical transform chain (sensor frame → reference point → as-seen-by) and
  identical tare. The zero is taken in the sensor frame after the modelled load's weight is
  removed (the bodies hanging off the sensor, walked from the world model, under the gravity
  the platform states); the first tare of a run commits that zero. Every later tare splits
  what it finds by gravity: the part along gravity is the held payload, a constant in the
  as-seen-by frame, and the part across gravity is the zero having moved. The reading is
  ``payload − transform(raw − zero − load)``.
- Telemetry: none, anywhere. The protobuf frame log is the record (user decision 2026-08-06).

The external-wrench law
-----------------------

**Doctrine: the external wrench always enters ACHD-family dynamics. The RNEA bridge always
carries zero wrench. RNE's** ``f_ext`` **interface is for RNE-as-motion-driver only.**

Per tick, for an ACHD motion carrying a wrench (measured, or the virtual elbow-support
force)::

   sim (mj_kdl)                                  real (robif2b)
   ------------                                  --------------
   qdd = ACHD(q, qd, α, β, f_ext = w, ff)        qdd, tau_c = ACHD(q, qd, α, β, f_ext = 0, ff)
   tau = RNEA(q, qd, qdd, f_ext = 0)             tau_w = ACHD_fext(q, qd, no-constraints, w)
       = M·qdd + C·qd + G                        tau   = tau_c + tau_w
   → qfrc_applied                                → firmware adds gravity + nature terms

Why the two shapes are each correct for their platform (from the torque_control howto):

- MuJoCo's forward dynamics is ``v̇ = M⁻¹(τ − c)`` — the controller must supply ``M·qdd + c``
  itself. ACHD's constraint torque lives in the gravity-absorbed ABA frame
  (``constraint_tau ≈ M·qdd − c``); sent raw it produces a double-gravity residual
  (measured: ~31 mm drift vs ~7 mm with the bridge). Hence the two-step pipeline: ACHD
  resolves ``qdd`` *with the wrench coupled in*, RNEA re-prices that ``qdd`` into the full
  torque. The howto is explicit: *"do not pass ACHD task/support wrenches into RNEA... keep
  the RNEA external-wrench vector zero in this path."*
- On a robot with an inner gravity-compensation loop, ``constraint_tau`` *is* the correct
  outer-loop command (howto, "Note on real robots"). robif2b therefore superposes two
  constraint-torque quantities — same kind, firmware completes — and carries the wrench with
  the dedicated per-segment ``ACHD_fext`` solve.

Compositions that were tried and rejected (2026-08-06, ``plan/1g``) — kept so they are not
re-invented:

- ``RNEA(qdd, 0) + tau_w`` in sim: QACC blow-up at the elbow — a constraint-only torque summed
  onto a complete one, two different quantities.
- ``tau_c + tau_w`` verbatim in sim: the arm sags — both terms constraint-only and no inner
  loop exists to complete them.
- ``RNEA(qdd, w)`` in sim (wrench through the bridge): gate-green but violates the wrapper's
  documented bridge contract, and the constrained solve no longer anticipates the wrench;
  reverted in favour of the coupled form above.

Facts about the solvers worth not re-deriving:

- ``ChainHdSolver_Vereshchagin_Fext_FixedJoint`` outputs *constraint-only* torque; the
  complete quantity is ``getTotalTorque()``.
- Its ``f_ext`` is per-segment — a mid-chain wrench (elbow support at ``half_arm_2_link``) is
  applied where it attaches, not tip-loaded.
- Its header restricts ``f_ext`` to *"physical (but not artificial, i.e. not task-introduced)"*
  wrenches. The elbow-support force is a PID output — task-introduced — and both backends have
  always fed it through ``f_ext`` anyway. Stable in practice, but outside the documented
  contract: open item, decide deliberately if it ever misbehaves.

What the gate covers
--------------------

``pick_place_single`` is the model that exercises the ACHD wrench law — 8 of its 10 motions
inject the virtual elbow support into ``state.f_ext``. ``admittance_arc_single``'s force
motions are RNE-driven (wrench through RNE's own interface, sanctioned); its generated
controller is byte-identical under ACHD wrench-law changes, so it gates nothing here.

Known residual asymmetries, outside codegen:

- Kinova bracelet mass is 0.95006 kg in the URDF (KDL side) and 0.5 kg in the MJCF (sim
  plant) — skews inverse-dynamics pricing against the simulated plant, and the FT/gravity
  paths differently in sim vs real. The fix belongs in the robot assets.
- Neither model includes reflected motor/gear inertia (armature) — dominant on the real GEN3,
  omitted consistently in both models (see the torque_control howto).
- The generated velocity-profile call path has no authoring model in the tree — parse-verified
  only.
