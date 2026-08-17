# Quantity expressions

Anywhere a reference value, a snapshot, or a constraint's view, threshold, reference, or
tolerance is authored, a full arithmetic expression over quantity refs and measures is
accepted: standard precedence, parentheses, and mixed operators.

```robmot
mass  k          = 1.2 kg,
force residual   = <shared.world.ext-force>.force.x - <spec.k> * <shared.world.acc-ee-base>.linacc.x,
force with-paren = (<a>.force.x + <b>.force.x) * 0.5 1
```

`*`/`/` bind tighter than `+`/`-`; parentheses group; `-`/`/` are left-associative binary
operators; repeated `+`/`*` stay one operation. Unary minus (`- <x>`) lowers to multiplication
by a dimensionless `-1`. A leaf is either a `<quantity>` reference (with the usual
`.subspace.axis` selector) or a bare measure such as `0.5 1` or `9.81 m/s^2`.

There are no functions (`abs`, `norm`, ...) and no exponent operator, and an expression never
combines a whole vector/tensor quantity (a pose, a wrench, a velocity twist) -- select a scalar
subspace axis first.

## Named and inline expressions

A named context quantity declared with an expression is usable everywhere a context quantity
is usable, including as a constraint's view:

```robmot
spec {
    force  residual  = <shared.world.ext-force>.force.x - <spec.k> * <shared.world.acc-ee-base>.linacc.x
}
...
constraints {
    slip: keeping <spec.residual> outside <spec.min-lim> and <spec.max-lim>
}
```

Every reference slot -- a constraint's view, threshold, reference, or tolerance, a saturation
bound, a profile limit, or a solver's gravity -- also accepts a **parenthesized** inline
expression, compiled into a quantity owned by the declaration that authors it:

```robmot
slip:  keeping (<shared.world.ext-force>.force.x - <spec.k> * <shared.world.acc-ee-base>.linacc.x)
       outside <spec.min-lim> and <spec.max-lim>,
tight: keeping <spec.error> outside (0.0 N - <spec.thr>) and <spec.thr>
       within (<shared.spec.band> * 2 1)
```

Parentheses are required at these slots: a bare `<x>` still stays a plain reference, and a bare
measure still stays a literal.

A snapshot's trailing expression works the same way, over what the snapshot samples:

```robmot
length support-z = snapshot of <shared.world.pose-ee-base>.position.z - <spec.lift>
```

## Dimension inference

- `+`/`-`: both operands must be the same kind of quantity; the result keeps that kind. Kinds
  that share one physical dimension under different authored spellings (a `length` and a
  `distance`, an `angle` and an `orientation`) are the same kind here -- the result takes the
  dimension's canonical spelling (`length` wins over `distance`/`position`, `angle` wins over
  `plane-angle`/`orientation`).
- `*`/`/`: the operands' dimension vectors (mass, length, time, angle exponents) combine, and
  the result must map back to a known quantity kind, or the model fails to validate, naming the
  offending subexpression and its derived dimension.
- Geometry kinds (`position`, `pose`, `orientation`) only add/subtract with their own kind --
  exactly what a snapshot offset already did. Multiplying or dividing a geometry kind is an
  error: select a scalar subspace axis instead.
- A composite quantity referenced with no scalar axis (a whole wrench or twist, `.force` rather
  than `.force.x`) is an error inside `*`/`/`.
- A named quantity's declared type must equal what its expression infers; a mismatch is a
  validation error naming both.
- Dividing by a literal zero is a validation error, not a runtime one.

## Controlled expressions

A monitor accepts the full algebra. A *controlled* expression constraint -- one a PID or
impedance controller drives through a solver -- is driven along the expression's gradient: each
measured view contributes its coefficient to that view's own Cartesian direction, and the solver
row is their normalized sum. The controller's gains are divided by the gradient's norm, so the
loop gain is invariant and a single-view expression degenerates to exactly the row the plain
view gets.

That gradient must be one constant vector in one frame, which is what the extra rules below
guard; an expression that breaks one is still fine to monitor.

- **Affine in what it measures**: no product of two measured views, and no measured view in a
  divisor -- the gradient would depend on the measurement.
- **Generation-time coefficients**: a measured view is scaled only by literals or
  literal-valued context quantities (`mass k = 1.2 kg` works; a snapshot, config, or derived
  gain does not).
- **One frame**: every measured view is `as-seen-by` the same frame -- the gradient is one
  vector in one frame.
- **One subspace family**: all linear or all angular axes; one solver row carries one of them,
  so split a mixed expression into one constraint per subspace.
- **A direction to drive**: coefficients that cancel, or an expression measuring nothing a
  solver moves, are rejected at generation time.

```robmot
// Driven along z: identical to constraining .position.z directly.
hold-z: keeping (<shared.world.pose-ee-base>.position.z + 0.0 m) equal to <spec.hold-height>,
// Driven along the (1,1,0)/sqrt(2) diagonal; authored gains are divided by sqrt(2).
diag:   keeping (<shared.world.pose-ee-base>.position.x + <shared.world.pose-ee-base>.position.y)
        equal to <spec.diag-target>
```

## Errors

```{list-table}
:header-rows: 1

* - Expression
  - Error
* - `<spec.a-length> + <shared.world.twist>.linvel.z`
  - "'Length' and 'LinearVelocity' cannot be added or subtracted -- they are different kinds of
    quantity."
* - `<spec.mass-k> * <spec.mass-k>`
  - "dimension (2, 0, 0, 0) from 'multiply' of [Mass, Mass] names no known quantity kind."
* - `<shared.world.tcp-base>.position * 2.0 1`
  - "'Position' is a geometry kind; multiply/divide need a scalar subspace axis on each operand
    instead."
* - `<shared.world.twist>.linvel * 2.0 1`
  - "select a scalar subspace axis."
* - `<spec.a> / 0.0 kg`
  - "division by zero."
* - `keeping <spec.some-pose> outside ...`
  - "Constraint '...' names a whole pose; select '.position' or '.orientation', each with its
    own tolerance."
* - `keeping <spec.mass-k> outside ...`
  - "Constraint '...': no constraint type for kind Mass."
```
