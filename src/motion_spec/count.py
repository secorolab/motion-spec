# SPDX-License-Identifier: MPL-2.0
"""Count entities and lines in motion specification model files."""

import sys
import os
import json
import numpy as np
import rdflib
from motion_spec.namespace import APP


class Table:
    def __init__(self):
        self.data = {}
        self.rows = set()
        self.columns = set()

    def _extend_rows(self, row):
        if row in self.rows:
            return

        self.data[row] = {}
        for column in self.columns:
            self.data[row][column] = None

        self.rows.add(row)

    def _extend_columns(self, column):
        if column in self.columns:
            return

        for row in self.rows:
            self.data[row][column] = None

        self.columns.add(column)

    def insert(self, row, column, entry):
        self._extend_rows(row)
        self._extend_columns(column)
        self.data[row][column] = entry

    def to_list(self):
        rows = list(self.rows)
        rows.sort()
        columns = list(self.columns)
        columns.sort()

        r = []
        for row in rows:
            c = []
            for column in columns:
                c.append(self.data[row][column])
            r.append(c)

        return r

    def row_names(self):
        rows = list(self.rows)
        rows.sort()
        return rows

    def column_names(self):
        columns = list(self.columns)
        columns.sort()
        return columns

    def __repr__(self):
        rows = list(self.rows)
        rows.sort()
        columns = list(self.columns)
        columns.sort()
        sep = "\t"
        lf = "\n"

        s = sep
        for column in columns:
            s += str(column) + sep
        s += lf

        for row in rows:
            s += str(row) + sep
            for column in columns:
                s += str(self.data[row][column]) + sep
            s += lf

        return s


def replace_by(arr, find, replace):
    arr[arr == find] = replace
    return arr


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
    entities = Table()
    lines = Table()

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

        entities.insert(folder, filename, num_entities)
        lines.insert(folder, filename, num_lines)

    #        world-model
    #        |  constraints
    #        |  |  controllers
    #        |  |  |  maps
    #        |  |  |  |  solver-specification
    #        |  |  |  |  |
    acc = np.array(
        [
            [0, 0, 0, 0, 0],  # 00-misc
            [1, 0, 0, 0, 0],  # 01-world-model
            [0, 0, 0, 1, 0],  # 02-map
            [0, 1, 0, 0, 0],  # 03-constraints
            [0, 1, 0, 0, 0],  # 04-motion-specification
            [0, 0, 1, 0, 0],  # 05-constraint-handler
            [0, 0, 0, 0, 1],  # 06-solver-specification
            [0, 0, 0, 0, 1],  # 07-scenario
        ]
    )

    ent_np = replace_by(np.array(entities.to_list()), None, 0)
    ent_np = ent_np @ acc

    lin_np = replace_by(np.array(lines.to_list()), None, 0)
    lin_np = lin_np @ acc

    print("world-model, constraints, controllers, map, solver-specification")
    print("entities:", np.sum(ent_np, axis=0))
    print(ent_np)
    print("lines:", np.sum(lin_np, axis=0))
    print(lin_np)


if __name__ == "__main__":
    main()
