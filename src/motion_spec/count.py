# SPDX-License-Identifier: MPL-2.0
"""Count entities and lines in motion specification model files."""

import sys
import os
import json
import rdflib
from motion_spec.namespace import APP

CATEGORIES = ["world-model", "constraints", "controllers", "map", "solver-specification"]

CATEGORY_BY_FILENAME = {
    "01-world-model": "world-model",
    "03-constraints": "constraints",
    "04-motion-specification": "constraints",
    "05-constraint-handler": "controllers",
    "02-map": "map",
    "06-solver-specification": "solver-specification",
    "07-scenario": "solver-specification",
}


def main():
    """Count entities and lines in motion specification models."""
    if len(sys.argv) < 2:
        print("Usage: motion-spec-count <manifest.json>")
        sys.exit(1)

    app_model = sys.argv[1]

    # Load top-level, application model
    g = rdflib.ConjunctiveGraph()
    g.parse(app_model, format="json-ld")

    # Handle the referenced models
    entities = {}
    lines = {}

    models = list(g.objects(predicate=APP["import"]))
    for model_path in models:
        split = model_path.split("/")
        folder = split[-2]
        file = split[-1]
        filename, _ = os.path.splitext(file)

        with open(os.path.join("models", folder, file)) as f:
            # Count the number of entities
            obj = json.load(f)["@graph"]
            num_entities = len(obj)

            # Count the number of lines
            serialized = json.dumps(obj, indent=4)
            num_lines = len(serialized.split("\n"))

        entities.setdefault(folder, {})[filename] = num_entities
        lines.setdefault(folder, {})[filename] = num_lines

    def bucket(table):
        rows = []
        for folder in sorted(table):
            row = {category: 0 for category in CATEGORIES}
            for filename, value in table[folder].items():
                category = CATEGORY_BY_FILENAME.get(filename)
                if category is not None:
                    row[category] += value
            rows.append([row[category] for category in CATEGORIES])
        return rows

    def format_matrix(rows):
        row_strs = ["[" + " ".join(str(v) for v in row) + "]" for row in rows]
        return "[" + ("\n ".join(row_strs)) + "]"

    ent_rows = bucket(entities)
    lin_rows = bucket(lines)

    ent_sum = [sum(row[i] for row in ent_rows) for i in range(len(CATEGORIES))]
    lin_sum = [sum(row[i] for row in lin_rows) for i in range(len(CATEGORIES))]

    print(", ".join(CATEGORIES))
    print("entities:", "[" + " ".join(str(v) for v in ent_sum) + "]")
    print(format_matrix(ent_rows))
    print("lines:", "[" + " ".join(str(v) for v in lin_sum) + "]")
    print(format_matrix(lin_rows))


if __name__ == "__main__":
    main()
