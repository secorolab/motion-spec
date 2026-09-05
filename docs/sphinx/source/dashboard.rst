.. SPDX-License-Identifier: MPL-2.0

Dashboard
=========

A browser view of everything a generation produces: its sources and provenance, its runs as
they execute, and its archives afterwards. One process, standard-library HTTP, no database —
everything shown is read from the files the pipeline already writes.

.. code-block:: console

   $ motion-spec dashboard                  # serve on :8080, browse $MOTION_SPEC_GEN
   $ motion-spec dashboard --port 8090 --logs path/to/generations
   $ motion-spec dashboard -b               # detached; -k stops it, -r restarts

``--logs`` names the generation root to browse (default ``$MOTION_SPEC_GEN``, else the working
directory); ``--sources`` the model source root (default: the logs root's parent).

The pages
---------

**Generations.** Every generation under the root: its model, whether the sources have drifted,
whether it is built, its runs. A built generation can be run from here; a model can be edited
in place and regenerated.

**Run.** One run, live or finished, fed by the shared memory block while the runtime writes and
by the archived frame log afterwards. Constraint plots (``Open in Jupyter`` seeds a notebook
beside the run), a timeline of state entries and constraint edges, SPARQL over the model graph
and the run's occurrences with a live overlay, the run's console, the files the run wrote
(frame log, runtime graph, REC record, bag, videos — each opens read-only), its notes (dated,
tagged, kept beside the archive), and the camera — a recorded mp4, or a ROS image topic on a
real platform.

**Health.** ``motion-spec health`` on the page, with the installation's versions and roots
above it. Reached from the sidebar footer, alongside the repository and these docs. The checks
are slow, so they are remembered — **re-check** reruns them. See :doc:`setup`.

Driving a simulated run
-----------------------

A simulated, introspected run creates a control block the loop polls once per tick; the
dashboard writes it, the loop acks. Pause skips the whole tick — no FSM step, no frame, no
control against a frozen plant. Step runs a counted number of ticks while paused. Speed sets
the real-time factor, so it means nothing to an uncapped headless run. On a real platform the
block does not exist and the controls are not offered.

The block is named ``/motion_spec_ctrl_<schema_hash[:16]>``; ``motion_spec/dashboard/control.py``
is its authoritative layout.

Nothing else is a channel: live values ride the frame publisher the runtime already writes,
history is the frame log, and cameras are mp4 recordings or ROS topics. A generation run
without the dashboard behaves identically — a zeroed control block commands nothing.
