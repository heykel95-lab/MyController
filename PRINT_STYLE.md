# Terminal output style

How the controller talks to the operator. Same spirit as
[COMMENT_STYLE.md](COMMENT_STYLE.md) and [README_STYLE.md](README_STYLE.md):
this is a living document — change the rule here first, then the code.

**The principle: the terminal answers "what is the robot commanding right
now".** Anything that does not help with that question belongs in `params/`,
in the CSV, or nowhere.

## 1. One block per phase, before that phase runs

Every phase, gate and mode introduces itself once, as it starts: a line naming
it, then the gains it commands.

```text
phase: approach_descend
  Kp=[150, 150, 150] N/m  KR=[90, 90, 90] Nm/rad  [t1,t2,n]
  Dp=[132.8, 94.2, 129.2] Ns/m  DR=[10.83, 17.71, 1.96] Nms/rad  auto (factor 1.90)
  descend to 20 mm clearance at 0.100 m/s
```

Never inside the phase's own debug stream. A value that appears between two
`descend:` lines is noise at the moment it is least readable.

## 2. Print a value once

If a number is in the phase block, it is not printed anywhere else. Two places
mean two versions to keep in step, and the reader has to work out which is
live.

## 3. Show only what this mode commands

Never print an alternative next to the value in use. `auto=[…] manual=[…]`
forces the reader to know which one wins; print the one that wins.

```text
Dp [x,y,z]    = [   255.2,    181.1,    103.9] Ns/m   auto     good
Dp auto=[255.2, …]  manual=[10.0, …]                           bad
```

The same rule picks the whole block: a hold prints hold gains, the set-up hold
prints set-up gains, a mode that never reads `alpha` does not mention `alpha`.

## 4. Wait for the real value

If a number is computed on the first control cycle — auto damping is — the
block waits for it rather than printing a placeholder and correcting itself.
One complete block beats two partial ones.

## 5. Units and frames, always

Every physical value carries its unit, and every vector says which frame it is
in: `[x,y,z]` for the base, `[t1,t2,n]` for the surface. A stiffness triple
without a frame is unreadable — the same three numbers mean different springs.

## 6. Say what can be typed, where it applies

Keys live in the block of the mode that accepts them, not in a startup banner
listing everything for every mode.

```text
  To command Kp, KR and the centre of compliance, type:
    kp1..kp3 <N/m>    kr1..kr3 <Nm/rad>    r1..r3 <mm>
```

The startup banner carries only what is always true:

```text
keys: e stop | m menu | g hand-guide
```

## 7. Silence is the normal case

Print the surprising state, not the expected one.

```text
Collision: Franka default thresholds (safety.txt is off).   only when off
```

No line means the configured thresholds are active. The same goes for
"running indefinitely" when that is the default.

## 8. No implementation details

`fitted Dp factor=0.545` describes how a number was reached; the number is
what gets commanded. Diagnostics that only make sense while debugging the
controller belong behind `print_*_debug` or in the CSV.

## 9. Periodic lines are rate-limited and narrow

The per-cycle debug lines (`orient:`, `descend:`, `hold:`) are gated by
`debug_period` and stay on one line, aligned so a column can be read down the
screen. They report state that changes; they never repeat configuration.

## 10. Nothing slow in the control loop

No file I/O inside the 1 kHz callback — CSV rows are buffered and written
after the run. One-shot blocks and rate-limited debug lines are the only
printing a control cycle may do.
