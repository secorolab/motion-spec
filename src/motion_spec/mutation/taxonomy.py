# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

"""What became of each mutant: one class per record, from the campaign's own reports.

Four classes, in the order the pipeline decides them. The toolchain either refuses a variant before
it ever runs, or it runs and the run either stays inside the reference envelope or leaves it; a run
that left it is attributed when the ranking puts the damaged element first, and detected but
unattributed when it does not. Nothing is re-scored here -- the classes are read off what the
campaign already recorded.
"""

import json
from pathlib import Path

CLASSES = ("rejected", "silent", "attributed", "detected-unattributed")

# A build failure is the same static refusal as a rejected model, one stage later: the variant was
# caught without being executed.
REFUSED = ("rejected", "build_fail")

# Each campaign report and the rescored file that supersedes its ranking, if the rescore was run.
REPORTS = (("report.jsonl", "report_v2.jsonl"), ("report_natural.jsonl", "report_natural_v2.jsonl"))


def load(campaign: Path) -> list[dict]:
    """Every record the campaign holds, classified against the best ranking available for it."""
    rows = []
    for report, rescored in REPORTS:
        ranks = {record["mutant"]: record for record in _read(campaign / rescored)}
        for record in _read(campaign / report):
            ranked = ranks.get(record["mutant"])
            rows.append(
                {
                    "mutant": record["mutant"],
                    "operator": record["operator"],
                    "name": record["name"],
                    "outcome": record["outcome"],
                    "element_uri": record.get("element_uri"),
                    "scorer": "v2" if ranked is not None else "v1",
                    "target_rank": (ranked or record).get("target_rank"),
                    "class": classify(record, (ranked or record).get("target_rank")),
                }
            )
    return rows


def classify(record: dict, target_rank: int | None) -> str:
    """The one class this record falls in.

    A run that produced no frames at all -- it died or was killed -- never showed a deviation, so it
    is counted as absorbed rather than as a detection it did not make.
    """
    if record["outcome"] in REFUSED:
        return "rejected"
    if not record.get("deviated"):
        return "silent"
    return "attributed" if target_rank == 1 else "detected-unattributed"


def tally(rows: list[dict]) -> dict:
    """Counts per class, overall and per operator."""
    overall = dict.fromkeys(CLASSES, 0)
    per_operator: dict[str, dict[str, int]] = {}
    for row in rows:
        overall[row["class"]] += 1
        per_operator.setdefault(row["operator"], dict.fromkeys(CLASSES, 0))[row["class"]] += 1
    return {"overall": overall, "per_operator": per_operator}


def named(rows: list[dict], class_name: str) -> list[str]:
    """The mutants in one class, in campaign order."""
    return [row["mutant"] for row in rows if row["class"] == class_name]


def _read(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
