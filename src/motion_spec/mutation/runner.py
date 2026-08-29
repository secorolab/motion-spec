# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

"""Generate, build, run, and decode one mutant per mutation site.

Every subprocess is the installed `motion-spec` command run under the workspace environment, from
the workspace root: the scene the model imports names its assets relative to that root, so a run
started anywhere else cannot find them.
"""

import json
import re
import shutil
import subprocess
from dataclasses import asdict
from pathlib import Path

from motion_spec.mutation.operators import MutationSite, apply

IMPORT = re.compile(r'import\s+"([^"]+)"')

GEN_TIMEOUT = 240
BUILD_TIMEOUT = 420
RUN_TIMEOUT = 180
DECODE_TIMEOUT = 180
STDERR_LINES = 30


def workspace_root(model: Path) -> Path:
    """The workspace above the model, known by the environment script every subprocess sources."""
    for parent in model.resolve().parents:
        if (parent / "setup-grc.zsh").is_file():
            return parent
    raise RuntimeError(f"no setup-grc.zsh above {model}")


def reference_runs(model: Path, out_dir: Path, count: int, steps: int) -> tuple[Path, list[Path]]:
    """Generate and build the unmutated model once, then run it `count` times.

    Built once, not once per run: the reference is there to measure how much a run of this same
    controller varies, and rebuilding identical sources would only measure the compiler.
    """
    root = workspace_root(model)
    out_dir.mkdir(parents=True, exist_ok=True)
    generation = _generate(root, model, out_dir / "gen")
    if generation is None:
        raise RuntimeError("the unmutated model was rejected; nothing to compare mutants against")
    if _build(root, generation) is None:
        raise RuntimeError(f"the unmutated model did not build: {generation}")

    decoded = []
    for index in range(count):
        run_dir, _timed_out = _execute(root, generation, steps)
        if run_dir is None:
            raise RuntimeError(f"reference run {index} produced no run directory")
        frames = out_dir / f"reference{index:02d}.jsonl"
        if _decode(root, run_dir, frames):
            decoded.append(frames)
    if not decoded:
        raise RuntimeError("no reference run could be decoded")
    return generation, decoded


def existing_reference(out_dir: Path) -> tuple[Path, list[Path]] | None:
    """The reference a campaign directory already holds, or None when it holds none.

    Same layout `reference_runs` writes: the unmutated generation under `gen`, its decoded runs
    beside it. A second campaign into the same directory scores against those rather than making a
    new envelope the first campaign's mutants were never measured against.
    """
    decoded = sorted(out_dir.glob("reference*.jsonl"))
    generation = next(
        (
            candidate
            for candidate in sorted(out_dir.glob("gen/*/*"))
            if (candidate / "generated" / "model" / "ir.json").is_file()
        ),
        None,
    )
    return (generation, decoded) if generation is not None and decoded else None


def run_mutant(site: MutationSite, index: int, model: Path, out_dir: Path, steps: int) -> dict:
    """Build and run one mutant, and report how far down the pipeline it got."""
    root = workspace_root(model)
    name = site.name.replace("-", "_")
    mutant = out_dir / "mutants" / f"m{index:03d}_{site.operator}_{name}"
    record = {
        "mutant": mutant.name,
        "operator": site.operator,
        "name": site.name,
        "element_uri": site.element_uri,
        "outcome": "",
        "gen_dir": None,
        "run_dir": None,
        "frames": None,
        "notes": "",
    }
    if mutant.exists():
        shutil.rmtree(mutant)
    patched = _materialize(model, mutant / "model")
    patched.write_text(apply(model.read_text(), site))
    (mutant / "patch.json").write_text(json.dumps(asdict(site), indent=2))

    generation = _generate(root, patched, mutant / "gen", record)
    if generation is None:
        record["outcome"] = "rejected"
        return record
    record["gen_dir"] = str(generation)
    if _build(root, generation, record) is None:
        record["outcome"] = "build_fail"
        return record

    run_dir, timed_out = _execute(root, generation, steps, record)
    record["outcome"] = "run_timeout" if timed_out else record["outcome"] or "ran"
    if run_dir is None:
        record["notes"] = record["notes"] or "the run left no run directory"
        return record
    record["run_dir"] = str(run_dir)
    frames = mutant / "frames.jsonl"
    if _decode(root, run_dir, frames, record):
        record["frames"] = str(frames)
    return record


def _materialize(model: Path, workdir: Path) -> Path:
    """Copy the model directory into `workdir`, plus whatever it imports from above itself.

    The scene files import the arm and gripper ktrees from the models root a level up, so a
    workdir holding only the model directory has nothing for those imports to resolve to. The
    layout is rebuilt instead: the model directory keeps its name inside `workdir`, and every
    parent-relative import lands where its relative path already points.
    """
    source = model.parent
    copied = workdir / source.name
    copied.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, copied, dirs_exist_ok=True)
    for path in {
        imported
        for entry in source.iterdir()
        if entry.is_file()
        for imported in IMPORT.findall(entry.read_text(errors="replace"))
        if imported.startswith("../")
    }:
        origin = (source / path).resolve()
        if origin.is_file():
            target = (copied / path).resolve()
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(origin, target)
    return copied / model.name


def _generate(root: Path, model: Path, into: Path, record: dict | None = None) -> Path | None:
    """The generation directory `gen` made, or None when the model was rejected."""
    code, stdout, stderr, timed_out = _command(root, ["motion-spec", "gen", model, "-o", into])
    if code or timed_out:
        _note(record, stderr)
        return None
    generation = _last_path(stdout)
    return generation if generation is not None and generation.is_dir() else None


def _build(root: Path, generation: Path, record: dict | None = None) -> Path | None:
    code, _stdout, stderr, timed_out = _command(
        root, ["motion-spec", "build", generation], timeout=BUILD_TIMEOUT
    )
    if code or timed_out:
        _note(record, stderr)
        return None
    return generation


def _execute(
    root: Path, generation: Path, steps: int, record: dict | None = None
) -> tuple[Path | None, bool]:
    """Run the controller headless; report its run directory and whether the clock ran out.

    A run killed on the timeout still wrote frames up to the moment it was killed, so the run
    directory is looked up on the generation rather than read off a closing line that never came.
    """
    code, stdout, stderr, timed_out = _command(
        root,
        ["motion-spec", "run", generation, "--headless", "--steps", steps],
        timeout=RUN_TIMEOUT,
    )
    if timed_out:
        _note(record, stderr)
    elif code:
        _note(record, stderr)
        if record is not None:
            record["outcome"] = "run_fail"
    named = _last_path(stdout)
    if named is not None and named.is_dir():
        return named, timed_out
    return _newest_run(generation), timed_out


def _decode(root: Path, run_dir: Path, frames: Path, record: dict | None = None) -> bool:
    """Decode the run's frame log to JSON Lines beside the mutant."""
    log = run_dir / "logs" / "frame_log.pb"
    if not log.is_file():
        _note(record, f"no frame log at {log}")
        return False
    with frames.open("w") as sink:
        code, _stdout, stderr, timed_out = _command(
            root, ["motion-spec", "replay", log, "--jsonl"], timeout=DECODE_TIMEOUT, sink=sink
        )
    if code or timed_out or frames.stat().st_size == 0:
        _note(record, stderr or f"{log}: decoded to nothing")
        frames.unlink(missing_ok=True)
        return False
    return True


def _command(
    root: Path, argv: list, timeout: int = GEN_TIMEOUT, sink=None
) -> tuple[int | None, str, str, bool]:
    """Run one motion-spec command under the workspace environment, from the workspace root."""
    shell = [
        "zsh",
        "-c",
        f'source {root / "setup-grc.zsh"} >/dev/null 2>&1 && exec "$@"',
        "--",
        *(str(argument) for argument in argv),
    ]
    try:
        done = subprocess.run(
            shell,
            cwd=root,
            stdout=sink or subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
        return done.returncode, done.stdout or "", done.stderr or "", False
    except subprocess.TimeoutExpired as expired:
        return None, _text(expired.stdout), _text(expired.stderr), True


def _text(value) -> str:
    if value is None:
        return ""
    return value.decode(errors="replace") if isinstance(value, bytes) else value


def _note(record: dict | None, message: str) -> None:
    """Keep the tail of what failed; the head is banners, the tail is the reason."""
    if record is not None:
        record["notes"] = "\n".join(message.strip().splitlines()[-STDERR_LINES:])


def _last_path(stdout: str) -> Path | None:
    lines = [line.strip() for line in stdout.splitlines() if line.strip()]
    return Path(lines[-1]) if lines else None


def _newest_run(generation: Path) -> Path | None:
    runs = [entry for entry in (generation / "runs").glob("*") if entry.is_dir()]
    return max(runs, key=lambda entry: entry.stat().st_mtime) if runs else None
