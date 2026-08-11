.. SPDX-License-Identifier: MPL-2.0

Control loop timing
===================

Why the generated loop lands close to its nominal period on a stock kernel, what it deliberately
does not do, and how to measure it. Established 2026-08-11.

The contract
------------

The authored control period is a **nominal target**, not a guarantee. Some iterations finish
early and some late. Controllers and integrators consume ``shared.dt_measured_s`` — the measured,
bounded elapsed time — never an assumed fixed step, so a late iteration changes the integration
step rather than corrupting the result. ``period_ns`` in the frame log is the observation of what
actually happened.

Nothing here turns observed jitter into a controller constant, and nothing here claims hard
real-time behaviour.

Where the loop waits
--------------------

``sleep_until_next`` advances an **absolute** deadline and sleeps to it:

.. code-block:: c++

   next.tv_nsec += period_ns;              // deadline advances by exactly one period
   clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

The absolute deadline is the important part: error does not accumulate, because each deadline is
computed from the previous deadline rather than from the current time. A relative sleep would
drift by the wakeup latency every single cycle.

Why a stock kernel still overshoots
-----------------------------------

The loop's own work is not the problem. Measured on the three maintained simulation models, the
control computation is **4.5–5.1 µs mean** against a 1000 µs budget — under 1% utilisation. The
period nonetheless landed at 1.062–1.093 ms before the mitigations below. Everything between the
deadline and the wakeup belongs to the kernel:

Timer slack
   Linux grants every non-realtime thread a default timer slack of **50 µs** and may fire the
   timer that late in order to batch wakeups. This alone accounts for most of the observed
   overshoot. A ``SCHED_FIFO`` thread gets zero slack, which is why realtime-scheduled loops do
   not show it.

C-state exit latency
   The loop sleeps for roughly 99% of every period, so the core idles into deep C-states
   continuously. Exit latency on mobile-class parts is tens to hundreds of microseconds.

Scheduler wakeup latency
   A ``SCHED_OTHER`` thread becomes runnable on time, then waits for a CPU.

What the generated program does about it
----------------------------------------

Two mitigations, both pure userspace: no capabilities, no scheduling policy change, no kernel
configuration, and therefore nothing that can fail to deploy.

1. Timer slack is set to one nanosecond
   ``prctl(PR_SET_TIMERSLACK, 1)`` at startup, before the loop. This removes the 50 µs grant
   described above and is the single largest improvement available without privileges.

2. The wait is hybrid: sleep, then spin
   The loop sleeps until ``deadline − margin`` and then busy-polls ``clock_gettime`` until the
   deadline. Sleeping covers the bulk of the period cheaply; the spin covers the part where
   wakeup latency would otherwise land. ``clock_gettime`` is a vDSO call of a few tens of
   nanoseconds, so the poll is not a syscall storm.

   The margin is a named constant. It bounds the spin: the loop never busy-waits longer than the
   margin, in any of the three cases below.

The three cases, and the overrun
--------------------------------

Let ``remaining = deadline − now`` at the moment the loop reaches the wait:

=============================  ==================================================================
``remaining > margin``         sleep to ``deadline − margin``, then spin to the deadline
``0 < remaining <= margin``    skip the sleep, spin the remainder — still bounded by the margin
``remaining <= 0``             the deadline was already missed
=============================  ==================================================================

The third case is an **overrun**: the cycle's work did not fit in the period. The loop re-phases
onto the current time and does *not* attempt to catch up, because a controller that runs several
compressed cycles back to back to repay lost time is worse than one that admits the period
slipped — ``dt_measured_s`` already tells the controllers what actually elapsed.

Overruns are counted, and the count is reported on shutdown as
``[run] N of M cycles overran the control period`` — printed only when ``N`` is non-zero. A
silently missed deadline is the failure mode worth preventing: without the counter, an overrun is
indistinguishable from a normal cycle after the fact. Per-cycle ``period_ns`` in the frame log
still shows *which* cycles were late; the counter exists so nobody has to go looking to find out
whether any were.

The margin must stay a fixed constant rather than something adaptive. If compute time ever grows
to sit just inside the period, the loop spins for most of every cycle and burns a core; a named
constant with a stated ceiling makes that visible in review instead of emergent at runtime.

Who paces the loop
------------------

Exactly one component may pace a loop. Before mj_kdl_wrapper 0.3.2, ``mj_kdl::step()`` also slept
until wall time caught up, so a windowed run had two pacers: MuJoCo's sleep consumed the period
and the control loop found its deadline already gone on **6506 of 9967 cycles**, against 0
headless. The measured period was 1.016 ms windowed versus 1.000 ms headless — the loop that won
was the less accurate one.

``step()`` no longer sleeps. The generated control loop is the only pacer, in simulation and on
hardware alike, and the real-time factor scales its period:

.. code-block:: text

   effective period = nominal period / rtf     (rtf > 0)
   uncapped                                    (rtf == 0)

The factor comes from ``mj_kdl::realtime_factor_of`` in simulation, so the viewer's ``,`` and
``.`` speed keys keep working mid-run, and is the literal ``1.0`` on hardware. This is a named
backend dispatch (``loop-rtf-source``), not an inline conditional, for the same reason
``clock-time-source`` is.

The four modes
--------------

=========================  =========================================  ==========================
invocation                 real-time factor                           measured period
=========================  =========================================  ==========================
GUI (default)              viewer's setting, initially 1.0            1.000 ms mean
GUI ``--rtf x``            viewer seeded to ``x``, keys still adjust  scales with ``x``
headless (default)         **0.0 — uncapped**                         0.101 ms mean
headless ``--rtf x``       ``x``                                      scales with ``x``
=========================  =========================================  ==========================

Uncapped headless is safe because the simulation clock, not the wall clock, drives
``dt_measured_s``: ``clock-time-source-mj_kdl`` reads ``robot->data->time``. A run that finishes
ten times sooner produces the identical frame count and the identical final state — only wall time
changes. On hardware the clock is monotonic wall time and there is nothing to uncap.

Deliberately not done
---------------------

The following are standard on stock kernels and are *not* applied here. They are listed so the
omissions read as decisions rather than oversights:

``SCHED_FIFO`` / ``SCHED_RR``
   The largest remaining jitter reduction, and **not** a PREEMPT_RT feature — it works on a stock
   kernel. It needs ``rtprio`` in ``limits.conf`` or ``CAP_SYS_NICE``, so it is a deployment
   requirement rather than a code change. ``ros2_control`` applies it through
   ``realtime_tools::configure_sched_fifo``.

``mlockall(MCL_CURRENT | MCL_FUTURE)``
   Prevents page faults in the loop. Cheap to add; pairs naturally with ``SCHED_FIFO``.

PM QoS via ``/dev/cpu_dma_latency``
   Writing ``0`` and holding the descriptor open caps C-state exit latency. Needs root. Standard
   in EtherCAT and CNC stacks.

CPU affinity, ``isolcpus``, ``nohz_full``
   Kernel command line territory; appropriate for a dedicated control host, not for a workstation
   that also renders a MuJoCo window.

PREEMPT_RT
   Bounds the worst case rather than improving the average. Orthogonal to everything above.

Measuring it
------------

Every change here is measurable with what the pipeline already records — do not accept a timing
claim without it. ``period_ns`` is a per-frame field of the frame log and ``nominal_period_ns`` is
in its header:

.. code-block:: console

   $ motion-spec run <model>.robmot -o <generations>
   $ python -c "from motion_spec.introspection.replay import summarize; print(summarize('<log>.pb'))"

The summary reports mean and maximum period alongside mean and maximum compute time. Compare the
two: if compute is a small fraction of the period and the period still overshoots, the cause is
scheduling, not the controller. Run the same model before and after any change, and prefer the
distribution over a single maximum — one outlier on a loaded workstation says more about the
workstation than about the loop.
