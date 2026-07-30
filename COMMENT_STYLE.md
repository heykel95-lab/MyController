# Comment style

How comments are written in this repository — parameter files, C++, Python and
shell alike.

**The principle: write for the person tuning the file.** A few words, or one
sentence. A file is understood from its header block; nothing below it should
cost more than a glance.

This is a living document: when a rule stops fitting the work, change the rule
here first, then the files.

## 1. Every file opens with a header block

The first thing in any file is a block that says what the whole file is for:
what it owns, and how it relates to its neighbours. Aim for 3–8 lines. It is
the only place where several lines of prose are welcome.

```text
# ====================================================================
# HOLD gains -- per axis of the base frame [x, y, z]
# ====================================================================
# Applies when the startup menu picks hold. Hold locks the pose it starts
# from: position with hold_Kp/hold_Dp, orientation with hold_KR/hold_DR.
```

```cpp
// ====================================================================
// Parameter loading
// ====================================================================
// Reads the params/ topic files into one Parameters struct. Keys are
// disjoint across files; later files win if that is ever broken.
```

A header block answers *what is this and when does it apply*. It does not list
the file's contents line by line — the file itself does that.

## 2. Below the header: one or two lines, never more

Every other comment is at most two lines. If something needs a paragraph, it
belongs in the header block, in `README.md`, or in the commit message.

```text
# Switches to set up when the contact point reaches this clearance [m].
descend_surface_clearance = 0.020
```

Not this:

```text
# Alignment starts from the actual contact-point height captured at the end
# of descend. End + speed define the ramp; ramp time is derived as
# abs(end - captured_start) / speed. setup_timeout is independent and may
# stop the phase before the configured end is reached. Check the active
# normal stiffness times setup_push_end against the collision thresholds.
setup_push_end = 0.180
```

## 3. Physical quantities carry their unit

Every parameter and every variable holding a physical quantity states its unit
in brackets: `[m]`, `[m/s]`, `[N/m]`, `[Ns/m]`, `[Nm/rad]`, `[Nms/rad]`,
`[rad]`, `[deg]`, `[s]`, `[kg]`, `[kg m^2]`, `[Nm]`. Counts, flags and file
names do not need one.

```text
# Stiffness [N/m].
hold_Kp_x = 2000.0
```

For a `0/1` switch, say what each value does instead:

```text
# 0 = manual damping only. 1 = manual damping is also a per-axis floor.
auto_damping_min_from_manual = 0
```

## 4. New topics get a title, not a longer comment

When a file moves on to a different subject, open a titled section. That is how
a reader skims; it is also what keeps individual comments short.

```text
# --------------------------------------------------------------------
# Auto damping
# --------------------------------------------------------------------
```

```cpp
// ---- descend step ----
```

If a file grows more than about three of these, it probably wants splitting
into two files instead.

## 5. Say why, not what

The code and the key name already say what. A comment earns its place by
explaining a choice, a unit, a range, or a consequence that is not visible.

```cpp
// Freeze the pole at first contact: tracking the live edge makes the
// commanded stiffness jump when the contact point switches corners.
```

Not `// set the pole`.

## 6. Do not keep history in comments

No "changed from 4.0", no "was setup_Kp_z before", no commented-out
alternatives left in place. Git holds that. A measured value may cite where it
came from in one line:

```text
# From the G2 run: force is flat from 4.6 s onward [s].
setup_min_time = 2.0
```

## Applies to

| Kind | Header marker | Section marker |
|------|---------------|----------------|
| `params/*.txt` | `# ===` banner | `# ---` banner |
| `*.cpp`, `*.h` | `// ===` banner | `// ---- title ----` |
| `*.py` | module docstring | `# ---- title ----` |
| `*.sh` | `#` block under the shebang | `# ---- title ----` |
