# Dual-arm pick and place

`ms-examples/02_dual_arm_pick_and_place` uses the same Kinova and gripper definitions twice without
copying their source models.

## 1. Distinguish semantic instances

The shared `models/common/dual_arm_table.scenex`, also used by the handover
example, declares four independent instances:

```text
ktree inst (ns=dat) kinova1 of <kinova_tree>
ktree inst (ns=dat) gripper1 of <gripper_tree>
ktree inst (ns=dat) kinova2 of <kinova_tree>
ktree inst (ns=dat) gripper2 of <gripper_tree>
```

`kinova1`, `kinova2`, `gripper1`, and `gripper2` are semantic identities. Generated
runtime prefixes keep repeated MJCF element names separate; the instance names are
not string-parsing conventions. Each arm and each gripper is its own agent, because
each is commanded through its own solver.

Both arms attach to frames authored on the table body. The second mount has a
180-degree extrinsic XYZ yaw, so the arms face each other:

```text
frame kinova2_mount {
    pose arm2_on_table {
        wrt: <table.table_top>
        xyz: (0.7, 0.0, 0.0) m
        orientation: euler { axes: xyz extrinsic angles: (0, 0, 180) unit: deg }
    }
}
```

No auxiliary MJCF anchor asset is required; arbitrary semantic frames are valid
fixed-joint endpoints.

## 2. Generate and run

```bash
motion-spec run \
  src/ms-examples/02_dual_arm_pick_and_place/dual_arm_pick_and_place.robmot \
  -o /tmp/dual-arm-pick-and-place \
  --prefix /path/to/workspace/install \
  --run-id tutorial
```

The MuJoCo viewer opens by default. Add `--headless` only for unattended runs.

## 3. Follow the two control paths

The shared world context declares one pose and twist per arm. `handler-home` declares
one ACHD solver per agent. Later handlers explicitly route each controller to the
matching solver:

```robmot
arm1-solver: serial-chain {
    agent:     <agents.arm1>,
    algorithm: achd,
    limits {   torque: saturation { max: <shared.spec.arm-torque-limit> } },
    gravity:   (0.0, 0.0, 9.81) m/s^2
},
arm2-solver: serial-chain {
    agent:     <agents.arm2>,
    algorithm: achd,
    limits {   torque: saturation { max: <shared.spec.arm-torque-limit> } },
    gravity:   (0.0, 0.0, 9.81) m/s^2
}
```

The two gripper agents get their own command-forwarding solvers:

```robmot
gripper1-solver: command-forwarding {
    agent: <agents.gripper1>
},
gripper2-solver: command-forwarding {
    agent: <agents.gripper2>
}
```

## 4. Inspect velocity-profile propagation

The `pick` motion creates one lowering profile per arm. Each profile names limits
and the measured TCP z velocity. Its PID controller attaches the profile and the
same measured derivative:

```robmot
velocity-profile lower1-profile = profile {
    max-velocity: <spec.max-lower-velocity>,
    max-acceleration: <spec.max-lower-acceleration>,
    measured-velocity: <shared.world.twist-ee1-base>.linvel.z,
    max-jerk: <spec.max-lower-jerk>,
    shape: s-curve
}

pid ctrl-pick-lower1-z { constraint: <pick.lower1-z>, profile: <pick.spec.lower1-profile>, measured-derivative: <shared.world.twist-ee1-base>.linvel.z, Kp: 12, Ki: 5, Kd: 15, decay: 0 } via <handler-home.arm1-solver>
```

In `generated/model/ir.json`, find `lower1-profile`; then find the same profile on
`ctrl-pick-lower1-z` in the generated controller. This checks model-to-IR-to-codegen
propagation without relying only on the observed motion.
