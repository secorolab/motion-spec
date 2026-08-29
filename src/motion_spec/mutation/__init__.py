# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)

"""Mutate a motion model, run every mutant, and score which constraint the deviation points at.

One mutation per mutant, applied to the model's text; the pipeline then answers whether the
recorded run deviates from an unmutated reference envelope, and whether the constraint ranked
worst is the one the mutation touched.
"""
