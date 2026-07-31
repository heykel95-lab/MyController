# Superseded MAIN results

The first attempt at the `MAIN_*` campaign of [`../AXIS_STUDY.md`](../AXIS_STUDY.md),
set aside on 2026-07-31 when the campaign was restarted from Case A.

These runs are kept as exploratory data. They are **not** primary results and
must not be mixed into the restarted campaign's means.

```text
MAIN_A0_00deg                  3 repeats, no commanded spin
MAIN_A1_t1_05deg               3 repeats, no commanded spin
MAIN_A2_t1_10deg               1 repeat, incomplete
MAIN_A0_00deg_spin_gate_2deg   2 repeats, spin commanded, 2 deg spin gate
metrics_before_redo.csv        the metrics.csv rows as they stood at the move
```

## MAIN_A0_00deg_spin_gate_2deg

The first two restarted A0 trials, kept for comparison rather than discarded.
They command the spin correctly but gate it at 2 deg, the threshold the tool
axis needs, so both handed over to descend while the spin was still converging
-- r01 at 2.0 deg, r02 at 2.0 deg -- and the clearance capture froze that error
into the contact reference. From 2026-07-31 the spin has its own 0.5 deg gate,
so the rest of Case A converges before handover and is not comparable to these
two at that resolution.

r02 against the three superseded A0 runs above: the phenomenon is unchanged,
alignment worsening by about 0.8 deg with steady force at 49 N, and
`align_t1_before_deg` matches to two decimals at 0.91. The shifts sit in
`align_t2_before_deg` (+0.15) and `tip_final_deg` (-0.25), which is where a
different start posture and a different leading edge would appear.

## Why they were set aside

The spin about the tool axis was not commanded when these ran. The two
`tool_target_offset_*` tilts fix the tool-axis direction only, so the
remaining rotation about that axis came from whichever `q_init` the run
started in. For the 40 x 120 mm face that spin picks the leading contact
edge, and a 120 mm edge and a 40 mm edge are different contact conditions
under the same commanded angle.

`tool_target_offset_normal_deg` and `command_tool_twist` were added on
2026-07-31 and are now pinned in `MAIN_COMMON`, so every restarted run records
its spin instead of inheriting it. The runs here predate that and cannot be
compared against runs that have it.

`MAIN_A2_t1_10deg/r01` is separately unusable: it was driven in guiding mode
and stopped with `e` during `approach_descend`, so the set-up phase never ran
and `terminal.log` has no `=== Set-up result ===` block. `run_axis_study.sh`
refuses to continue past a partial trial directory, which is why it was moved
rather than left in place.

Raw controller CSVs are gitignored here exactly as under `results/`. The
provenance files, the terminal transcripts and the metrics snapshot are
tracked.
