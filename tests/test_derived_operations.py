# SPDX-License-Identifier: MPL-2.0
from __future__ import annotations

from pathlib import Path

import pytest
from motion_spec_dsl.rdf_parser.vocab import (
    CSTR,
    CSTR_EXT,
    CSTR_HDL,
    GEOM_COORD,
    GEOM_ENT,
    GEOM_OP,
    GEOM_OP_EXT,
    GEOM_REL,
    QUDT_SCHEMA,
)
from rdf_utils.constraints import ConstraintViolation
from rdf_utils.models.vocab import (
    URI_DISTRIB_TYPE_SAMPLED_QUANTITY,
    URI_GEOM_PRED_ALPHA,
    URI_GEOM_PRED_AXES_SEQ,
    URI_GEOM_PRED_BETA,
    URI_GEOM_PRED_GAMMA,
    URI_GEOM_PRED_OF_ORIENT,
    URI_GEOM_PRED_OF_POSE,
    URI_GEOM_PRED_OF_POSITION,
    URI_GEOM_PRED_SEEN_BY,
    URI_GEOM_PRED_W,
    URI_GEOM_TYPE_ANGLES_ABG,
    URI_GEOM_TYPE_EULER_ANGLES,
    URI_GEOM_TYPE_EXTRINSIC,
    URI_GEOM_TYPE_ORIENT_REF,
    URI_GEOM_TYPE_POSE,
    URI_GEOM_TYPE_POSE_COORD,
    URI_GEOM_TYPE_POSE_REF,
    URI_GEOM_TYPE_POSITION_REF,
    URI_GEOM_TYPE_VECTOR_XYZ,
    URI_QUDT_UNIT_CM,
    URI_QUDT_UNIT_M,
    URI_QUDT_UNIT_RAD,
)
from rdflib import Dataset, URIRef
from rdflib.namespace import RDF
from scipy.spatial.transform import Rotation

from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.operations import (
    ErrorEvaluator,
    materialize_linear_distance_operations,
    materialize_pose_reference_transforms,
)
from motion_spec.rdf_parser.quantities import frame_placement, relative_orientation

BASE = "https://example.test/"

PREFIXES = f"""
@prefix ex: <{BASE}> .
@prefix cstr: <{CSTR}> .
@prefix cstr-ext: <{CSTR_EXT}> .
@prefix cstr-hdl: <{CSTR_HDL}> .
@prefix geom-coord: <{GEOM_COORD}> .
@prefix geom-ent: <{GEOM_ENT}> .
@prefix geom-op: <{GEOM_OP}> .
@prefix geom-op-ext: <{GEOM_OP_EXT}> .
@prefix geom-rel: <{GEOM_REL}> .
@prefix qudt: <{QUDT_SCHEMA}> .
"""

FRAMES = "".join(
    f"ex:{frame} a geom-ent:Frame ; geom-ent:origin ex:{frame}-origin .\n"
    f"ex:{frame}-origin a geom-ent:Point .\n"
    for frame in ("frame-base", "frame-table", "frame-shoulder", "frame-ee")
)

# A pose quantity of `of` with respect to and as seen by `wrt`.
POSE = (
    "ex:{name} a qudt:Quantity, geom-rel:Pose, geom-coord:PoseCoordinate ;\n"
    "    geom-rel:of ex:{of} ; geom-rel:with-respect-to ex:{wrt} ; geom-coord:as-seen-by ex:{wrt} .\n"
)

# The distance between `shoulder wrt base` and `ee wrt base`, with a base<-table connecting pose,
# sampled by two constrained coordinates the way two motions measure the same two poses.
TWO_DISTANCE_COORDINATES = (
    PREFIXES
    + FRAMES
    + POSE.format(name="pose-table-base", of="frame-table", wrt="frame-base")
    + POSE.format(name="pose-shoulder-base", of="frame-shoulder", wrt="frame-base")
    + POSE.format(name="pose-ee-x", of="frame-ee", wrt="frame-base")
    + """
ex:dist-rel a geom-rel:LinearDistance ;
    geom-rel:between-entities ex:pose-shoulder-base, ex:pose-ee-x .
ex:dist a geom-coord:DistanceReference ; geom-coord:of ex:dist-rel .
ex:c-dist a cstr:Constraint ; cstr:quantity ex:dist .
ex:dist-2 a geom-coord:DistanceReference ; geom-coord:of ex:dist-rel .
ex:c-dist-2 a cstr:Constraint ; cstr:quantity ex:dist-2 .
"""
)

# `pose ee-wrt-base` equal to a reference `ee wrt table`.
CROSS_FRAME_EQUALITY = (
    PREFIXES
    + FRAMES
    + POSE.format(name="pose-table-base", of="frame-table", wrt="frame-base")
    + POSE.format(name="pose-ee-base", of="frame-ee", wrt="frame-base")
    + POSE.format(name="ref-ee", of="frame-ee", wrt="frame-table")
    + """
ex:c-eq a cstr:Constraint, cstr:EqualityConstraint ;
    cstr:quantity ex:pose-ee-base ; cstr:reference-value ex:ref-ee .
"""
)

# An orientation composing `pose-ee-base` with an Euler delta in canonical rdf-utils form; the
# operand slots are filled per case.
RELATIVE_ORIENTATION = (
    PREFIXES
    + FRAMES
    + POSE.format(name="pose-ee-base", of="frame-ee", wrt="frame-base")
    + f"""
ex:delta a qudt:Quantity, <{URI_GEOM_TYPE_EULER_ANGLES}>, <{URI_GEOM_TYPE_ANGLES_ABG}>,
        <{URI_GEOM_TYPE_EXTRINSIC}> ;
    <{URI_GEOM_PRED_AXES_SEQ}> "xyz" ; qudt:unit <{URI_QUDT_UNIT_RAD}> ;
    <{URI_GEOM_PRED_ALPHA}> -0.75e0 ; <{URI_GEOM_PRED_BETA}> 0.0e0 ; <{URI_GEOM_PRED_GAMMA}> 0.0e0 .
ex:relative-orientation a geom-coord:OrientationCoordinate ;
    geom-rel:of ex:frame-ee ; geom-coord:as-seen-by ex:frame-base .
ex:relative-orientation-composition a geom-op-ext:ComposeOrientation ;
    geom-op:composite ex:relative-orientation .
"""
)

SCENE_FRAMES = "".join(
    f"ex:{frame} a geom-ent:Frame ; geom-ent:origin ex:{frame}-origin .\n"
    for frame in ("frame-ground", "frame-world", "frame-object", "frame-jointed")
)

# A placement as scene-dsl emits one: a Pose over a position and an orientation. `_position_of`
# looks a frame up by its origin point, so the position relates the two origins.
PLACEMENT = f"""
ex:position-{{of}} a geom-rel:Position ;
    geom-rel:of ex:{{of}}-origin ; geom-rel:with-respect-to ex:{{wrt}}-origin .
ex:position-coord-{{of}} a geom-coord:PositionCoordinate, <{URI_GEOM_TYPE_POSITION_REF}>,
        <{URI_GEOM_TYPE_VECTOR_XYZ}> ;
    geom-coord:of-position ex:position-{{of}} ; geom-coord:as-seen-by ex:{{wrt}} ;
    geom-coord:x {{x:.17e}} ; geom-coord:y {{y:.17e}} ; geom-coord:z {{z:.17e}} ;
    qudt:unit <{{unit}}> .
ex:orientation-{{of}} a geom-rel:Orientation ;
    geom-rel:of ex:{{of}} ; geom-rel:with-respect-to ex:{{wrt}} .
ex:orientation-coord-{{of}} a geom-coord:OrientationCoordinate, <{URI_GEOM_TYPE_ORIENT_REF}>,
        geom-coord:Quaternion ;
    geom-coord:of-orientation ex:orientation-{{of}} ; geom-coord:as-seen-by ex:{{wrt}} ;
    geom-coord:x {{qx:.17e}} ; geom-coord:y {{qy:.17e}} ; geom-coord:z {{qz:.17e}} ;
    <{URI_GEOM_PRED_W}> {{qw:.17e}} .
ex:pose-{{of}} a <{URI_GEOM_TYPE_POSE}>, <{URI_GEOM_TYPE_POSITION_REF}>,
        <{URI_GEOM_TYPE_ORIENT_REF}> ;
    geom-rel:of ex:{{of}} ; geom-rel:with-respect-to ex:{{wrt}} ;
    <{URI_GEOM_PRED_OF_POSITION}> ex:position-{{of}} ;
    <{URI_GEOM_PRED_OF_ORIENT}> ex:orientation-{{of}} .
ex:pose-coord-{{of}} a <{URI_GEOM_TYPE_POSE_REF}>, <{URI_GEOM_TYPE_POSE_COORD}> ;
    <{URI_GEOM_PRED_OF_POSE}> ex:pose-{{of}} ; <{URI_GEOM_PRED_SEEN_BY}> ex:{{wrt}} .
"""

IDENTITY = {"qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}


def test_each_coordinate_of_one_relation_computes_its_own_value() -> None:
    """Two motions measuring the same two poses share the relation; sharing the derivation
    would leave the second reading the first's value.
    """
    g = Dataset(default_union=True)
    g.default_graph.parse(data=TWO_DISTANCE_COORDINATES, format="turtle")
    materialize_linear_distance_operations(Model(graph=g, app_path=Path("model-app.ld.json")))

    first, second = URIRef(BASE + "dist"), URIRef(BASE + "dist-2")
    outputs = {
        g.value(op, GEOM_OP.distance) for op in g.subjects(RDF.type, GEOM_OP.PoseToLinearDistance)
    }
    assert outputs == {first, second}
    assert f"{first}.derived-relative-pose" != f"{second}.derived-relative-pose"


def test_pose_reference_cross_frame_reexpresses_reference() -> None:
    g = Dataset(default_union=True)
    g.default_graph.parse(data=CROSS_FRAME_EQUALITY, format="turtle")
    materialize_pose_reference_transforms(Model(graph=g, app_path=Path("model-app.ld.json")))

    constraint = URIRef(BASE + "c-eq")
    reexpressed = URIRef(f"{constraint}.derived-reference-in-target")
    assert g.value(constraint, CSTR["reference-value"]) == reexpressed
    compose = URIRef(f"{constraint}.derived-compose-reference")
    assert (compose, RDF.type, GEOM_OP.ComposePose) in g
    assert g.value(compose, GEOM_OP.in2) == URIRef(BASE + "ref-ee")
    assert g.value(compose, GEOM_OP.composite) == reexpressed
    relation = URIRef(f"{reexpressed}-pose-rel")
    assert g.value(relation, GEOM_REL["with-respect-to"]) == URIRef(BASE + "frame-base")


def test_pose_reference_rejects_body_mismatch() -> None:
    g = Dataset(default_union=True)
    g.default_graph.parse(
        data=CROSS_FRAME_EQUALITY.replace(
            "ex:ref-ee a qudt:Quantity, geom-rel:Pose, geom-coord:PoseCoordinate ;\n"
            "    geom-rel:of ex:frame-ee",
            "ex:ref-ee a qudt:Quantity, geom-rel:Pose, geom-coord:PoseCoordinate ;\n"
            "    geom-rel:of ex:frame-other",
        )
        + "ex:frame-other a geom-ent:Frame ; geom-ent:origin ex:frame-other-origin .\n"
        + "ex:frame-other-origin a geom-ent:Point .\n",
        format="turtle",
    )

    with pytest.raises(ConstraintViolation, match="compares a pose"):
        materialize_pose_reference_transforms(Model(graph=g, app_path=Path("model-app.ld.json")))


@pytest.mark.parametrize(
    ("operands", "order"),
    [
        (
            (
                "ex:delta geom-coord:as-seen-by ex:frame-ee .\n"
                "ex:relative-orientation-composition geom-op:in1 ex:pose-ee-base ; geom-op:in2 ex:delta .\n"
            ),
            ("pose", "delta"),
        ),
        (
            (
                "ex:delta geom-coord:as-seen-by ex:frame-base .\n"
                "ex:relative-orientation-composition geom-op:in1 ex:delta ; geom-op:in2 ex:pose-ee-base .\n"
            ),
            ("delta", "pose"),
        ),
    ],
    ids=["base-first", "delta-first"],
)
def test_relative_orientation_reads_operands_in_slot_order(operands: str, order: tuple) -> None:
    g = Dataset(default_union=True)
    g.default_graph.parse(data=RELATIVE_ORIENTATION + operands, format="turtle")
    read = relative_orientation(
        Model(graph=g, app_path=Path("model-app.ld.json")), URIRef(BASE + "relative-orientation")
    )
    assert order[0] in read[0] and order[1] in read[1]
    # The delta is authored as an Euler triple and leaves as the quaternion it denotes.
    delta = read[order.index("delta")]
    assert delta["representation"] == "quaternion"
    assert [c["value"] for c in delta["delta"]] == pytest.approx(
        [-0.36627253, 0.0, 0.0, 0.93050762]
    )


def test_a_placement_keeps_its_quaternion_composes_to_the_anchor_and_scales_to_metres() -> None:
    """A wrong frame, unit or quaternion convention gives wrong numbers without breaking a build."""
    qx, qy, qz, qw = Rotation.from_euler("xyz", (0.3, -0.2, 0.75)).as_quat()
    g = Dataset(default_union=True)
    g.default_graph.parse(
        data=PREFIXES
        + SCENE_FRAMES
        + PLACEMENT.format(
            of="frame-world",
            wrt="frame-ground",
            x=0.0,
            y=0.0,
            z=0.72,
            unit=URI_QUDT_UNIT_M,
            **IDENTITY,
        )
        + PLACEMENT.format(
            of="frame-object",
            wrt="frame-world",
            x=-0.9,
            y=1.8,
            z=0.05,
            unit=URI_QUDT_UNIT_M,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
        ),
        format="turtle",
    )
    model = Model(graph=g, app_path=Path("model-app.ld.json"))
    anchor = URIRef(BASE + "frame-ground")

    position, orientation = frame_placement(model, URIRef(BASE + "frame-object"), anchor)
    assert position == pytest.approx([-0.9, 1.8, 0.77])
    assert orientation == pytest.approx([qx, qy, qz, qw])
    # A body a joint holds is placed by the joint, not by a pose, so it composes to nothing.
    assert frame_placement(model, URIRef(BASE + "frame-jointed"), anchor) == (None, None)

    g = Dataset(default_union=True)
    g.default_graph.parse(
        data=PREFIXES
        + SCENE_FRAMES
        + PLACEMENT.format(
            of="frame-object",
            wrt="frame-world",
            x=150.0,
            y=-50.0,
            z=720.0,
            unit=URI_QUDT_UNIT_CM,
            **IDENTITY,
        ),
        format="turtle",
    )
    assert frame_placement(
        Model(graph=g, app_path=Path("model-app.ld.json")),
        URIRef(BASE + "frame-object"),
        URIRef(BASE + "frame-world"),
    )[0] == pytest.approx([1.5, -0.5, 7.2])


def test_sampled_scene_placements_are_rejected() -> None:
    """A placement is built into the world before the run draws anything, so a drawn pose can
    only be a frame on a body, never what places the body."""
    g = Dataset(default_union=True)
    g.default_graph.parse(
        data=PREFIXES
        + SCENE_FRAMES
        + PLACEMENT.format(
            of="frame-object",
            wrt="frame-world",
            x=1.0,
            y=2.0,
            z=3.0,
            unit=URI_QUDT_UNIT_M,
            **IDENTITY,
        )
        + f"ex:position-coord-frame-object a <{URI_DISTRIB_TYPE_SAMPLED_QUANTITY}> .\n",
        format="turtle",
    )

    with pytest.raises(ConstraintViolation, match="cannot be drawn"):
        frame_placement(
            Model(graph=g, app_path=Path("model-app.ld.json")),
            URIRef(BASE + "frame-object"),
            URIRef(BASE + "frame-world"),
        )


def test_a_constraint_no_evaluator_compiles_is_rejected_by_name() -> None:
    """A relation kind no evaluator reads leaves the authored bound uncompared. Dropping the
    evaluator silently emits a program that ignores it, so generation names the constraint."""
    g = Dataset(default_union=True)
    g.default_graph.parse(
        data=PREFIXES
        + """
ex:eval-home-while-above-table a cstr-hdl:ErrorEvaluator ;
    cstr-hdl:constraint <https://example.test/home/while/above-table> .
<https://example.test/home/while/above-table> a cstr:Constraint, cstr-ext:AngleConstraint ;
    cstr:quantity ex:tcp-height .
""",
        format="turtle",
    )

    with pytest.raises(ConstraintViolation, match="above_table.*AngleConstraint"):
        ErrorEvaluator().function_step(
            Model(graph=g, app_path=Path("model-app.ld.json")),
            URIRef(BASE + "eval-home-while-above-table"),
        )
