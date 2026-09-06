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

Organizing results
------------------

Generations and runs have editable labels, pins, tags, and dated notes. Labels leave the folder
name and provenance unchanged. Pinned generations appear first; pinned runs lead their run
list. Search includes model names, labels and tags. The generation browser remembers folded
groups, filters and sorting; use newest, last run or largest to order the list.

Choose **Use as model baseline** on a run to compare later runs against it, including runs from
other generations of the same model. Reports offer all of that model's runs and a direct
baseline action. Comparison aligns the existing activity timings; it does not overlay signals.

**Clean up old…** selects older unpinned generations for a preview. The preview counts bundles,
contained runs and size, and excludes active runs, pins and model baselines. The server checks
protection again before removal, including protected runs inside selected generations. Items
move to desktop Trash; space is freed only when Trash is emptied. **Trash** in the sidebar
lists this root's removed items and restores them without overwriting existing paths.

Labels, pins and explicit tags live in ``dashboard.json`` beside the generation or run; notes
live in ``notes.json``. A model's ``dashboard-model.json`` records its baseline. These ordinary
JSON annotation files are separate from the generated RDF and REC provenance.

Returning to an investigation
----------------------------

Archived replay views remember open plots, signals, zoom, panel, camera and cursor when leaving
or reloading. Named plot presets reuse signal choices across runs of the same model; unavailable
signals are reported and skipped, and a preset never reuses another run's time window. These
preferences are local to the browser and generation root.

**Copy link to this moment** includes the replay frame. Run notes can attach the current frame
or an optional frame range; their timestamps and timeline markers return to it. Generation
notes remain general notes. Report event times open the relevant replay position and constraint
plots; aggregate signal reports link to the motion rather than implying an exact peak time.

Verification
------------

Run the dashboard Python tests with the workspace environment and
``PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest tests/test_dashboard_*.py``. The optional browser
integration test requires Playwright and ``MOTION_SPEC_BROWSER_EXECUTABLE`` pointing to a
Chromium executable. Give pytest a fresh ``--basetemp`` directory under
``/home/batsy/work/ms/generations/``; the browser test moves and restores only its own fixtures.

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
