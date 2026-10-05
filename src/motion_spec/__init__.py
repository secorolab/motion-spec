# SPDX-License-Identifier: MPL-2.0
"""motion_spec package."""

import warnings

# rdflib's Dataset.parse() internally reads its own deprecated `default_context`
# property, emitting a DeprecationWarning we cannot avoid from our side (it is
# rdflib calling itself). Silence just that one message so it doesn't flood the
# CLI tools and test logs; all other warnings are left intact.
warnings.filterwarnings(
    "ignore", message="Dataset.default_context is deprecated", category=DeprecationWarning
)
