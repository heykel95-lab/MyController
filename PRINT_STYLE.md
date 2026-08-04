# Terminal output style

How the controller talks to the operator. Same spirit as
[COMMENT_STYLE.md](COMMENT_STYLE.md) and [README_STYLE.md](README_STYLE.md):
this is a living document — change the rule here first, then the code.

**The principle: the terminal answers "what is the robot commanding right
now".** Anything that does not help with that question belongs in `params/`,
in the CSV, or nowhere.

## 1. One block per phase, before that phase runs

Every phase, gate and mode introduces itself once, as it starts: a rule naming
it, then the gains it commands.

```text
-- phase: approach_descend -----------------------------------------
  Kp [t1,t2,n]     = [    150.0,     150.0,     150.0] N/m
  Dp [t1,t2,n]     = [    132.8,      94.2,     129.2] Ns/m     auto (factor 1.90)
  KR [t1,t2,n]     = [     90.0,      90.0,       8.0] Nm/rad
  DR [t1,t2,n]     = [     10.8,      17.7,       2.0] Nms/rad  auto (factor 1.90)
  descend to         20 mm clearance at 0.100 m/s
```

Never inside the phase's own debug stream. A value that appears between two
`descend:` lines is noise at the moment it is least readable.

## 1a. Three pieces of furniture, one width

Blocks are opened by `printBanner`, `printSection` or `printRule` — nothing
else draws a rule, so every block in the transcript lines up at 68 columns.

```text
====================================================================
  STARTUP MODE                      what the operator chose or got
====================================================================

-- set-up impedance ------------------------------------------------
                                    a part of it
--------------------------------------------------------------------
                                    closes a list
```

Inside a block there is one label column: two spaces, the label padded to 16,
then the value from column 22 — `= [` for a triple, three spaces for prose.
`printRow` writes the triple form. Vertical borders are never drawn: they
would break the log-reading scripts listed at the bottom of this file.

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

Keys live in the block of the mode that accepts them, on a `keys` row, not in
a banner listing everything for every mode.

```text
  keys               kp1..kp3 <N/m> | kr1..kr3 <Nm/rad> | r1..r3 <mm>
```

The startup menu carries the ones that hold in every run, as its last row:

```text
  in a run    e stop | m menu | g hand-guide | s sequence | t hold
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

## 11. Some lines are an interface: do not reword them

`experiments/` drives a trial by watching for its prompts and reads the saved
transcript back for the set-up report. These exact strings are what it finds;
changing one silently breaks a campaign, so change the reader in the same
commit or leave the line alone.

| String | Read by | Used for |
|--------|---------|----------|
| `Press Enter to recover and configure the robot.` | `lib/auto_drive.py` | answers the first prompt |
| `Choice [s/h/t/g/q/o/c/r/f/b/e]: ` | `lib/auto_drive.py` | answers the startup menu |
| `Choice [0/1/2/3` | `lib/auto_drive.py` | answers the nullspace selector |
| `[GATE] Reached` | `lib/auto_drive.py` | passes the set-up gate |
| `[GATE] Set up finished` | `lib/auto_drive.py` | ends the trial |
| `SET-UP RESULT` banner title | `run_axis_study.sh`, the two pilot runners | proof that a contact trial pressed |
| `RELEASE` disturbance cue | `run_axis_study.sh` | proof that a hold trial ran to its end |
| `stop:` line, `\|`-separated `t= tip= F= M=` | `analysis/sgc_log.py` | cross-checks the CSV metrics |
| `alignment:` line, `before= after= gain=` | `analysis/sgc_log.py` | alignment gain |
| `r_c [t1,t2,n]` row | `analysis/sgc_log.py` | commanded centre of compliance |

The three read from the transcript are matched on the stripped line, so they
may be indented — but they must stay at the start of it, and their `key=value`
parts must keep their separators.
