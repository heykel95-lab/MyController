# Case F, first disturbance timeline

Superseded on 2026-07-31, before Case F was complete. Kept because the mode-1
runs carry usable null-space traces and they are what sized the replacement
window.

Two faults, neither recoverable in analysis:

The three mode-0 runs contain no null-space data at all. Mode 0 returned before
the projector was built, so `nullspace_speed` and every `nullspace_dq` column
is exactly zero across 29968 samples per run. The mode commands no torque,
correctly, but it still has to observe the axis it is the baseline for. Fixed
in the controller; these runs predate the fix.

The timeline had no stop-moving cue, so the driven stretch and the statically
held one run together and the transition can only be inferred. Every archived
repetition kept moving 0.8 to 2.5 s past the release cue, and one started 3.2 s
before the push cue.

They did establish the recovery window. All null-space motion ceased within
2.6 s of the release cue and the arm then sat idle for 19.5 to 21.2 s, so the
22 s recovery was almost entirely dead time. The replacement uses 10 s.

Excursion across the eight moving runs ranged 0.65 to 3.01 rad, sd 0.86. That
spread is larger than the damping sweep is likely to produce and remains the
dominant uncontrolled variable in the case.
