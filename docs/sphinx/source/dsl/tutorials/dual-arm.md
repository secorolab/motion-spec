# Dual-arm pick and place

`models/pick_place_dual` uses the same Kinova and gripper definitions twice without
copying their source models.

## 1. Distinguish semantic instances

The expanded scene declares four independent instances:

```text
ktree inst (ns=ppd_mjc) kinova1 of <kinova_tree>
ktree inst (ns=ppd_mjc) gripper1 of <gripper_tree>
ktree inst (ns=ppd_mjc) kinova2 of <kinova_tree>
ktree inst (ns=ppd_mjc) gripper2 of <gripper_tree>
```

`kinova1`, `kinova2`, `gripper1`, and `gripper2` are semantic identities. Generated
runtime prefixes keep repeated MJCF element names separate; the instance names are
not string-parsing conventions.

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
  src/motion-spec-dsl/models/pick_place_dual/pick_place_dual.robmot \
  -o /tmp/pick-place-dual \
  --prefix /path/to/workspace/install \
  --run-id tutorial
```

The MuJoCo viewer opens by default. Add `--headless` only for unattended runs.

## 3. Follow the two control paths

The shared world context declares one pose and twist per arm. `handler-home` declares
one ACHD solver per agent. Later handlers explicitly route each controller to the
matching solver:

```robmot
solver arm1-solver {
    agent: <pickplace_agents.arm1>,
    algorithm: ACHD,
    gravity: { x: 0.0, y: 0.0, z: -9.81 m/s2 }
},
solver arm2-solver {
    agent: <pickplace_agents.arm2>,
    algorithm: ACHD,
    gravity: { x: 0.0, y: 0.0, z: -9.81 m/s2 }
}
```

The same separation applies to the two gripper command-forwarding solvers.

## 4. Inspect velocity-profile propagation

The `pick` motion creates one lowering profile per arm. Each profile names limits
and the measured TCP z velocity. Its PID controller attaches the profile and the
same measured derivative:

```robmot
velocity-profile lower1-profile = profile {
    max-velocity: <spec.lower-max-v>,
    max-acceleration: <spec.lower-max-a>,
    measured-velocity: <shared.world.twist-ee1-base>.linvel.z,
    shape: trapezoidal
}

pid ctrl-pk1-lower-z {
    constraint: <pick.lower1-z>,
    profile: <pick.spec.lower1-profile>,
    measured-derivative: <shared.world.twist-ee1-base>.linvel.z,
    Kp: 12, Ki: 5, Kd: 15, decay: 0
} via <handler-home.arm1-solver>
```

In `generated/model/ir.json`, find `lower1-profile`; then find the same profile on
`ctrl-pk1-lower-z` in the generated controller. This checks model-to-IR-to-codegen
propagation without relying only on the observed motion.
