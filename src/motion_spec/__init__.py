# SPDX-License-Identifier: MPL-2.0
"""motion_spec package."""

import warnings

# raised by rdflib calling itself, not avoidable from here
warnings.filterwarnings(
    "ignore", message="Dataset.default_context is deprecated", category=DeprecationWarning
)
