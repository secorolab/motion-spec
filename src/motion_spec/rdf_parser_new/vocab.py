# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
"""RDF terms motion-spec reads that no dependency binds.

Everything else comes from `rdf_utils.models.vocab`, `rdf_utils.namespace`,
`motion_spec_dsl.rdf_parser.vocab` or `scene_dsl.rdf_parser.vocab`. Only the behaviour FSM is
bound here, because the FSM metamodel has no Python package of its own; the event-loop namespace
it pairs with is `rdf_utils.namespace.NS_MM_EL`.
"""

from rdf_utils.namespace import URL_SECORO_MM
from rdflib import Namespace

NS_MM_FSM = Namespace(f"{URL_SECORO_MM}/behaviour/fsm#")

URI_FSM_TYPE_FSM = NS_MM_FSM["FiniteStateMachine"]
URI_FSM_PRED_NAME = NS_MM_FSM["name"]
URI_FSM_PRED_DESCRIPTION = NS_MM_FSM["description"]
URI_FSM_PRED_STATES = NS_MM_FSM["states"]
URI_FSM_PRED_START_STATE = NS_MM_FSM["start-state"]
URI_FSM_PRED_END_STATE = NS_MM_FSM["end-state"]
URI_FSM_PRED_TRANSITIONS = NS_MM_FSM["transitions"]
URI_FSM_PRED_TRANSITION_FROM = NS_MM_FSM["transition-from"]
URI_FSM_PRED_TRANSITION_TO = NS_MM_FSM["transition-to"]
URI_FSM_PRED_REACTIONS = NS_MM_FSM["reactions"]
URI_FSM_PRED_DO_TRANSITION = NS_MM_FSM["do-transition"]
URI_FSM_PRED_FIRES_EVENTS = NS_MM_FSM["fires-events"]
