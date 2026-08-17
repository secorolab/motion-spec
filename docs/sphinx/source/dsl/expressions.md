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
