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

**Generations.** Every generation under the root: the model it came from, whether its sources
have drifted since, whether it is built, its runs. A built generation can be run from here —
simulated ones with headless/speed/camera-recording choices, real ones only after every declared
device answered a probe. A model can be edited in place and regenerated.

**Run.** One run, live or finished — the page is the same either way, fed by the shared memory
block while the runtime writes and by the archived frame log afterwards.

- *Plots*: any authored constraint plotted from its own log signals, live as the run writes;
  an empty plot accepts any signal the contract names. ``Open in Jupyter`` seeds a notebook
  beside the run that loads the same signals.
- *Transport*: play/scrub the whole run; the timeline carries markers for state entries, fired
  events, constraint satisfied/unsatisfied edges and monitor firings — the same edges the
  runtime graph projects as occurrences.
- *SPARQL*: the model graph, the run's occurrences, and a live overlay of the newest frame's
  values (``urn:model`` / ``urn:runtime`` / ``urn:live``), queried together. Queries are saved
  beside the run they belong to.
- *Console*: the run's terminal output, colors included.
- *Camera*: a simulated run that recorded shows its mp4 beside the timeline. A real platform
  records nothing — the pane subscribes to a ROS image topic instead, defaulting to the
  declared camera's own name (``/<camera_id>/color``, the same topic a simulated run
  publishes), editable for external drivers.

Driving a simulated run
-----------------------

A simulated, introspected run creates a 56-byte control block the loop polls once per tick;
the dashboard writes it, the loop acks. Pause skips the whole tick — no FSM step, no frame, no
control against a frozen plant. Step executes a counted number of ticks while paused. Speed
writes the viewer's real-time factor, the loop's single pacer; headless runs are uncapped, so
speed means nothing there. On a real platform the block does not exist and the controls are
not offered.

.. list-table::
   :header-rows: 1

   * - offset
     - field
     - writer
   * - 0
     - version = 1
     - runtime, at create
   * - 8
     - seq
     - dashboard, bumped after setting fields
   * - 16
     - pause (0/1)
     - dashboard
   * - 24
     - speed (clamped 0.1–10; 0 reads as 1.0)
     - dashboard
   * - 32
     - stop (0/1)
     - dashboard
   * - 40
     - ack_seq
     - runtime, after applying seq
   * - 48
     - steps (single-step budget while paused)
     - dashboard

The block is named ``/motion_spec_ctrl_<schema_hash[:16]>``; the code in
``motion_spec/dashboard/control.py`` is the authoritative layout.

Nothing else is a channel: live values ride the frame publisher the runtime already writes,
history is the frame log, and cameras are mp4 recordings or ROS topics. A generation run
without the dashboard behaves identically — a zeroed control block commands nothing.
