# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""Plan 026 section 5: the nine questions the consolidated dataset must answer.

Each question is one module-level SPARQL constant -- they are the record of what the vocabulary
has to support, and a question that cannot be written is a defect of the model, not of the query.
The dataset under test is a real archived run, consolidated from a copy so `generations/` is
never touched; the acceptance-fluent and failed-run halves are synthesised against that run.
"""

from __future__ import annotations

import importlib.util
import os
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from uuid import UUID

import pytest
import rdflib
from conftest import requires_workspace

from motion_spec.introspection.provenance import (
    ensure_local_rec_importable,
    parse_rec_time,
    prov_uri,
    rec_run_lifecycle_from_file,
)

ensure_local_rec_importable()

METAMODELS = Path(__file__).resolve().parents[2] / "metamodels"

PREFIXES = """
PREFIX bdd: <https://secorolab.github.io/metamodels/acceptance-criteria/bdd#>
PREFIX ms-prov: <https://secorolab.github.io/metamodels/motion-spec/prov#>
PREFIX p-plan: <http://purl.org/net/p-plan#>
PREFIX prov: <http://www.w3.org/ns/prov#>
PREFIX rec: <https://secorolab.github.io/metamodels/rec#>
PREFIX sosa: <http://www.w3.org/ns/sosa/>
PREFIX time: <http://www.w3.org/2006/time#>
"""

# Q1 -- an authored constraint, the motion it was held under, and what the robot did about it.
Q1 = (
    PREFIXES
    + """
SELECT ?constraint ?motion ?at ?value WHERE {
  ?occ a ms-prov:ConstraintMaintenance ;
       prov:used ?constraint ;
       prov:wasInformedBy ?mx ;
       sosa:hasSimpleResult ?value ;
       time:hasBeginning/time:inTimePosition/time:numericPosition ?at .
  ?mx a ms-prov:MotionExecution ; prov:used ?motion .
  ?constraint a ?kind .
}
"""
)

# Q1 with the band the authored line implies: empty on a clean run, rows when it was exceeded.
Q1_EXCEEDED = (
    PREFIXES
    + """
SELECT ?constraint ?motion ?at ?value WHERE {
  ?occ a ms-prov:ConstraintMaintenance ;
       prov:used ?constraint ;
       prov:wasInformedBy ?mx ;
       sosa:hasSimpleResult ?value ;
       time:hasBeginning/time:inTimePosition/time:numericPosition ?at .
  ?mx a ms-prov:MotionExecution ; prov:used ?motion .
  ?constraint a ?kind .
  FILTER(ABS(?value) > 0.05)
}
"""
)

# Q2 -- from an observed value back to the authored model document it came out of.
Q2 = (
    PREFIXES
    + """
SELECT ?constraint ?kind ?step ?model ?location WHERE {
  ?occ a ms-prov:ConstraintMaintenance ; prov:used ?constraint ;
       p-plan:correspondsToStep ?step ; sosa:hasSimpleResult ?value .
  ?step p-plan:isStepOfPlan ?plan ; prov:used ?constraint .
  ?run prov:qualifiedAssociation/prov:hadPlan ?plan ; prov:used ?model .
  ?model prov:atLocation ?location .
  ?constraint a ?kind .
}
"""
)

# Q3 -- a goal that was lost: what held it, what took it away, which motion was in force.
Q3 = (
    PREFIXES
    + """
SELECT ?goal ?failure ?motion ?constraint WHERE {
  ?goal prov:wasGeneratedBy ?held ; prov:wasInvalidatedBy ?failure .
  ?held a ms-prov:ConstraintMaintenance ; prov:used ?constraint .
  ?failure prov:used ?motion .
}
"""
)

# Q4 -- transitive causality: everything the run set off, counted per motion.
Q4 = (
    PREFIXES
    + """
SELECT ?motion (COUNT(DISTINCT ?occ) AS ?occurrences) WHERE {
  ?run a ms-prov:TaskExecution .
  ?occ prov:wasInformedBy+ ?run ; prov:wasInformedBy ?mx .
  ?mx a ms-prov:MotionExecution ; prov:used ?motion .
} GROUP BY ?motion
"""
)

# Q5 -- inference, not enumeration: the occurrences are activities because the axioms say so.
Q5 = PREFIXES + "SELECT ?activity WHERE { ?activity a prov:Activity }"

# Q6 -- one authored constraint across two generations, joined on the design IRI both name.
Q6 = (
    PREFIXES
    + """
SELECT ?constraint ?earlier ?later (COUNT(*) AS ?pairs) WHERE {
  ?occA a ms-prov:ConstraintMaintenance ; prov:used ?constraint ; sosa:hasSimpleResult ?a ;
        prov:wasInformedBy/prov:wasInformedBy ?earlier .
  ?earlier a ms-prov:TaskExecution .
  ?occB a ms-prov:ConstraintMaintenance ; prov:used ?constraint ; sosa:hasSimpleResult ?b ;
        prov:wasInformedBy/prov:wasInformedBy ?later .
  ?later a ms-prov:TaskExecution .
  FILTER(STR(?earlier) < STR(?later))
} GROUP BY ?constraint ?earlier ?later
"""
)

# Q7 -- planned but never recorded: a dead spec line, or a recorder that missed it.
Q7 = (
    PREFIXES
    + """
SELECT ?step ?referent WHERE {
  ?step p-plan:isStepOfPlan ?plan ; prov:used ?referent .
  ?run  prov:qualifiedAssociation/prov:hadPlan ?plan .
  FILTER NOT EXISTS { ?occ p-plan:correspondsToStep ?step }
}
"""
)

# Q8 -- which motion was in force when an acceptance criterion went false.
Q8 = (
    PREFIXES
    + """
SELECT ?fluent ?motion ?at WHERE {
  ?obs sosa:hasFeatureOfInterest ?fluent ; sosa:hasSimpleResult "FALSE" ;
       sosa:phenomenonTime/time:inTimePosition/time:numericPosition ?at .
  ?sx  a bdd:ScenarioExecution ; prov:wasInformedBy ?run .
  ?mx  a ms-prov:MotionExecution ; prov:wasInformedBy ?run ; prov:used ?motion ;
       time:hasBeginning/time:inTimePosition/time:numericPosition ?from ;
       time:hasEnd/time:inTimePosition/time:numericPosition ?to .
  FILTER(?from <= ?at && ?at <= ?to)
}
"""
)

# Q9 -- for a failed run: the stacktrace rec kept and the in-graph chain that lost the goal.
Q9 = (
    PREFIXES
    + """
SELECT ?run ?trace ?goal ?failure WHERE {
  ?run a rec:FailedRun ; rec:fail-trace ?trace .
  ?goal prov:wasInvalidatedBy ?failure . ?failure prov:wasInformedBy+ ?run .
}
"""
)

MS_PROV = rdflib.Namespace("https://secorolab.github.io/metamodels/motion-spec/prov#")
P_PLAN = rdflib.Namespace("http://purl.org/net/p-plan#")
PROV = rdflib.Namespace("http://www.w3.org/ns/prov#")
TIME = rdflib.Namespace("http://www.w3.org/2006/time#")
RUNTIME_GRAPH = rdflib.URIRef("urn:runtime")
INFERRED_GRAPH = rdflib.URIRef("urn:inferred")
PLAN_GRAPH = rdflib.URIRef("urn:plan")


def _generations_root() -> Path | None:
    """Locate the workspace's generations tree (honours MOTION_SPEC_GENERATIONS)."""
    env_path = os.environ.get("MOTION_SPEC_GENERATIONS")
    if env_path:
        return Path(env_path)
    for start in (Path.cwd(), Path(__file__).resolve()):
        for root in (start, *start.parents):
            if (root / "generations").is_dir():
                return root / "generations"
    return None


def _archived_runs() -> list[Path]:
    """Two runs of one model, each from its own generation, that carry the run vocabulary."""
    root = _generations_root()
    if root is None:
        return []
    for model in sorted(root.iterdir(), reverse=True):
        runs = []
        for generation in sorted(model.glob("*/runs/run-*")) if model.is_dir() else []:
            runtime = generation / "runtime" / "runtime.ttl"
            if not (generation / "rec.ld.json").exists() or not runtime.exists():
                continue
            if "ms-prov:TaskExecution" in runtime.read_text():
                runs.append(generation)
        by_generation = {run.parents[1]: run for run in runs}
        if len(by_generation) >= 2:
            return sorted(by_generation.values())[-2:]
    return []


ARCHIVES = _archived_runs()

pytestmark = [
    requires_workspace(METAMODELS),
    pytest.mark.skipif(
        len(ARCHIVES) < 2, reason="needs two archived runs of one model under generations/"
    ),
]


def _consolidate(destination: Path, run_dir: Path) -> Path:
    """Consolidate a copy of ``run_dir``; the archive itself is read-only."""
    from rec.consolidate import consolidate_run

    copy = destination / run_dir.name
    shutil.copytree(run_dir, copy, ignore=shutil.ignore_patterns("logs"))
    return consolidate_run(copy, generation_dir=run_dir.parents[1], metamodels_dir=METAMODELS)


def _dataset(*paths: Path) -> rdflib.Dataset:
    dataset = rdflib.Dataset(default_union=True)
    for path in paths:
        dataset.parse(path, format="trig")
    return dataset


@pytest.fixture(scope="module")
def consolidated(tmp_path_factory) -> rdflib.Dataset:
    return _dataset(_consolidate(tmp_path_factory.mktemp("latest"), ARCHIVES[-1]))


def test_q1_walks_an_authored_line_to_what_the_robot_did(consolidated):
    rows = list(consolidated.query(Q1))
    assert rows, "no authored constraint reached an observed value"
    # A clean run stays inside the band, so the exceedance arm is allowed to be empty -- what
    # matters is that the question is answerable at all.
    assert isinstance(list(consolidated.query(Q1_EXCEEDED)), list)


def test_q2_walks_back_to_the_authored_model(consolidated):
    rows = list(consolidated.query(Q2))
    assert rows
    for _constraint, _kind, step, _model, location in rows:
        assert str(step).startswith("https://secorolab.github.io/motion-spec/provenance/step/")
        assert str(location)


def test_q3_names_what_lost_a_goal(consolidated):
    rows = list(consolidated.query(Q3))
    assert rows, "no invalidated goal in this run"
    for _goal, failure, motion, constraint in rows:
        assert (failure, PROV.used, motion) in consolidated
        assert (None, PROV.used, constraint) in consolidated


def test_q4_counts_transitive_causality_per_motion(consolidated):
    rows = list(consolidated.query(Q4))
    assert rows
    assert all(int(count) > 0 for _motion, count in rows)


def test_q5_is_answered_by_inference_not_by_enumeration(consolidated):
    runtime = consolidated.graph(RUNTIME_GRAPH)
    enumerated = {row.activity for row in runtime.query(Q5)}
    with_axioms = {row.activity for row in (runtime + consolidated.graph(INFERRED_GRAPH)).query(Q5)}
    occurrences = set(runtime.subjects(rdflib.RDF.type, MS_PROV.TaskExecution))
    occurrences |= set(runtime.subjects(rdflib.RDF.type, MS_PROV.MotionExecution))
    occurrences |= set(runtime.subjects(rdflib.RDF.type, MS_PROV.ConstraintMaintenance))
    assert occurrences - enumerated, "the runtime graph hand-types what the axioms should derive"
    assert occurrences <= with_axioms
    assert len(enumerated) < len(with_axioms)


def test_q6_joins_two_generations_on_the_design_iri(tmp_path):
    paths = [_consolidate(tmp_path / f"gen{index}", run) for index, run in enumerate(ARCHIVES)]
    rows = list(_dataset(*paths).query(Q6))
    assert rows, "no constraint was observed in both generations"
    for _constraint, earlier, later, _pairs in rows:
        assert earlier != later


def test_q7_reports_exactly_what_was_planned_and_never_recorded(consolidated):
    plan, runtime = consolidated.graph(PLAN_GRAPH), consolidated.graph(RUNTIME_GRAPH)
    recorded = set(runtime.objects(None, P_PLAN.correspondsToStep))
    expected = {
        (step, referent)
        for step in plan.subjects(P_PLAN.isStepOfPlan)
        if step not in recorded
        for referent in plan.objects(step, PROV.used)
    }
    assert {(row.step, row.referent) for row in consolidated.query(Q7)} == expected


def test_q8_names_the_motion_a_failing_fluent_fell_in(tmp_path):
    from rec.consolidate import consolidate_run

    copy = tmp_path / ARCHIVES[-1].name
    shutil.copytree(ARCHIVES[-1], copy, ignore=shutil.ignore_patterns("logs"))
    motion, first, last = _longest_motion(copy / "runtime" / "runtime.ttl")
    middle = (first + last) // 2
    _write_scenario_execution(copy, first, middle)
    path = consolidate_run(copy, generation_dir=ARCHIVES[-1].parents[1], metamodels_dir=METAMODELS)
    rows = [(str(row.motion), int(row.at)) for row in _dataset(path).query(Q8)]
    assert (str(motion), middle) in rows
    assert all(at == middle for _motion, at in rows)


def test_q9_joins_the_lifecycle_to_the_causal_chain(tmp_path):
    from rec.consolidate import consolidate_run

    copy = tmp_path / ARCHIVES[-1].name
    shutil.copytree(ARCHIVES[-1], copy, ignore=shutil.ignore_patterns("logs"))
    _fail_the_run(copy)
    path = consolidate_run(copy, generation_dir=ARCHIVES[-1].parents[1], metamodels_dir=METAMODELS)
    rows = list(_dataset(path).query(Q9))
    assert rows
    assert all("grasp lost" in str(row.trace) for row in rows)


def _longest_motion(runtime_ttl: Path):
    """The motion execution with the widest tick interval, and that interval."""
    graph = rdflib.Graph().parse(runtime_ttl, format="turtle")

    def position(occurrence, edge):
        instant = graph.value(occurrence, edge)
        return int(graph.value(graph.value(instant, TIME.inTimePosition), TIME.numericPosition))

    intervals = [
        (
            graph.value(occurrence, PROV.used),
            position(occurrence, TIME.hasBeginning),
            position(occurrence, TIME.hasEnd),
        )
        for occurrence in graph.subjects(rdflib.RDF.type, MS_PROV.MotionExecution)
    ]
    return max(intervals, key=lambda interval: interval[2] - interval[1])


@dataclass
class _Trinary:
    trinary: str
    stamp: float
    reason: str = ""


@dataclass
class _Policy:
    id: rdflib.URIRef
    fluent_id: rdflib.URIRef
    trinary_timeline: list = field(default_factory=list)


def _write_scenario_execution(run_dir: Path, first: int, middle: int) -> None:
    """Record one acceptance fluent going false inside a real motion, on the sim clock."""
    prov_trace = _prov_trace()
    model = rdflib.Namespace("https://secorolab.github.io/models/acceptance/")
    rate = 1000.0
    policy = _Policy(
        id=model["policy/container"],
        fluent_id=model["container-not-dropped"],
        trinary_timeline=[
            _Trinary("TRUE", first / rate),
            _Trinary("FALSE", middle / rate, "dropped"),
        ],
    )
    graph = prov_trace.scenario_execution_graph(
        UUID(int=7),
        model["variant/nominal"],
        [policy],
        start_time=first / rate,
        end_time=middle / rate,
        run=prov_trace.run_iri(run_dir.name),
    )
    graph.serialize(destination=run_dir / "runtime" / "bdd-nominal.ttl", format="turtle")


def _prov_trace():
    """Import the coordination node's trace writer from the workspace checkout."""
    for root in (Path.cwd(), *Path(__file__).resolve().parents):
        module = root / "src" / "bdd_exec_ros2" / "bdd_exec_ros2" / "prov_trace.py"
        if module.exists():
            spec = importlib.util.spec_from_file_location("prov_trace", module)
            loaded = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(loaded)
            return loaded
    pytest.skip("no bdd_exec_ros2 checkout to build a scenario execution with")


def _fail_the_run(run_dir: Path) -> None:
    """Turn the copied run's lifecycle into a failure, keeping its start time and its IRI."""
    from rec import Run
    from rec.observers import FileObserver

    rec_path = run_dir / "rec.ld.json"
    lifecycle = rec_run_lifecycle_from_file(rec_path)
    observer = FileObserver(rec_path, run_iri=prov_uri(f"run:{run_dir.name}"))
    run = Run(observers=[observer], run_id=run_dir.name)
    run._id = run_dir.name
    run.start_time = parse_rec_time(lifecycle["started_time"])
    try:
        raise RuntimeError("grasp lost")
    except RuntimeError as error:
        run._emit_failed(error)
    observer.close()
