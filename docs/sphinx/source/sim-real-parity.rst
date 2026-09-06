.. SPDX-License-Identifier: MPL-2.0

Sim/real parity
===============

Where the mj_kdl (simulation) and robif2b (hardware) paths are allowed to differ, where they
are not, and the external-wrench law. Established 2026-08-06, wrench law revised 2026-09-05
(motion-spec 51da846); the empirical record behind the original law is in
``plans/codegen-architecture/1g-*`` and branch ``plan/1g``. The authoritative
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

**Doctrine: an authored wrench is a force the arm exerts, never a disturbance the constraints
absorb. It is priced by its own ACHD pass and superposed on the acceleration pass; neither
the acceleration pass nor the RNEA bridge sees it. RNE's** ``f_ext`` **interface is for
RNE-as-motion-driver only.**

Per tick, for an ACHD motion carrying a wrench (measured, or an impedance applied at a
segment), both backends run the same two passes (motion-spec 51da846, 2026-09-05)::

   tau_w      = ACHD_fext(q, qd, α = 0, β = 0, f_ext = w, ff = 0)    = J^T·w and nothing else
   qdd, tau_c = ACHD(q, qd, α, β, f_ext = 0, ff)

and only the completion forks::

   sim (mj_kdl)                                  real (robif2b)
   ------------                                  --------------
   tau = RNEA(q, qd, qdd, f_ext = 0) + tau_w     tau = tau_c + tau_w
       = M·qdd + C·qd + G + J^T·w                → firmware adds gravity + nature terms
   → qfrc_applied

Why the wrench has a pass of its own: the default Vereshchagin solver takes ``f_ext`` as a
physical load and lets the constraint forces react to it, so it cannot prioritise a wrench over
the acceleration constraints — a mid-chain wrench under six tip rows comes out as almost
nothing at the joints (pick_place_single, 2026-09-06: a 224 Nm forearm couple priced as
16 Nm). The ``_Fext`` instance runs with zero constraints and zero feed-forward, and since the
fork fix of 2026-09-04 its ``constraint_torques`` output is the generalised force of ``f_ext``
alone, with no gravity or velocity bias in it. That quantity is complete on both platforms,
which is why one composition serves both.

Why the completion still forks (from the torque_control howto):

- MuJoCo's forward dynamics is ``v̇ = M⁻¹(τ − c)`` — the controller must supply ``M·qdd + c``
  itself. ACHD's constraint torque lives in the gravity-absorbed ABA frame
  (``constraint_tau ≈ M·qdd − c``); sent raw it produces a double-gravity residual
  (measured: ~31 mm drift vs ~7 mm with the bridge). Hence RNEA re-prices the resolved
  ``qdd`` into the full torque, and the wrench's torque is added after the bridge. The howto
  is explicit: *"do not pass ACHD task/support wrenches into RNEA... keep the RNEA
  external-wrench vector zero in this path."*
- On a robot with an inner gravity-compensation loop, ``constraint_tau`` *is* the correct
  outer-loop command (howto, "Note on real robots"). robif2b therefore superposes two
  constraint-torque quantities — same kind, firmware completes.

Consequence for authoring: the wrench is really applied now. An impedance ``apply at`` a
mid-chain segment fights the tip rows instead of vanishing into them, so its gains must be
sized as torques the joints will carry, and the redundancy of a 7-DOF arm under six tip rows
is pinned in joint space (``hold-elbow`` as torque), not by pushing a link with a wrench.
pick_place_single's forearm-align impedance (400 Nm/rad on a 0.56 rad error, a 224 Nm demand)
only ever worked because the coupled form absorbed it; exerted, it stalled the approach and
MuJoCo diverged at t≈138 s (2026-09-06).

History (``plan/1g``, 2026-08-06), kept so it is not re-derived:

- Before the fork fix ``ACHD_fext`` returned the constraint-frame reaction, so
  ``RNEA(qdd, 0) + tau_w`` in sim blew up qacc at the elbow — a constraint-only torque summed
  onto a complete one. Sim therefore coupled the wrench into the acceleration solve,
  ``qdd = ACHD(..., f_ext = w)``, until 51da846. That form is retired: the constraints absorb
  the wrench, so an impedance authored against it has no authority.
- ``tau_c + tau_w`` verbatim in sim: the arm sags — both terms constraint-only and no inner
  loop exists to complete them.
- ``RNEA(qdd, w)`` in sim (wrench through the bridge): gate-green but violates the wrapper's
  documented bridge contract.

Facts about the solvers worth not re-deriving:

- ``ChainHdSolver_Vereshchagin_Fext_FixedJoint``: ``constraint_torques`` is ``J^T·f_ext``
  (fork fix 2026-09-04); the complete quantity is ``getTotalTorque()``. The default
  ``ChainHdSolver_Vereshchagin_Fixed_Joint`` cannot prioritise ``f_ext`` over the
  constraints, which is why the second instance exists.
- Its ``f_ext`` is per-segment — a mid-chain wrench is applied where it attaches, not
  tip-loaded.
- Its header restricts ``f_ext`` to *"physical (but not artificial, i.e. not task-introduced)"*
  wrenches. An impedance's wrench is a controller output — task-introduced — and both backends
  feed it through ``f_ext`` anyway. Stable in practice, but outside the documented contract:
  open item, decide deliberately if it ever misbehaves.

What the gate covers
--------------------

``pick_place_single`` and ``pick_place_dual`` carry no wrench since 2026-09-06 (six tip rows
plus a joint-space elbow pin); they gate that composition, not the wrench law. The ACHD wrench
pass in sim is exercised by the models with an impedance applied on an ACHD chain —
``handover_dual``, ``look_cartesian_test``, ``geometric_expressions``, ``tableii_probe`` —
none of which has been re-run under the exerted law yet. ``admittance_arc_single``'s force
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
