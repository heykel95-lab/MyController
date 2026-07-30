# How the README should be written

The README is for someone who wants to **run the controller**, not study it.
It answers three questions in order: what the files are, how to start it, and
what each mode does. Everything else belongs in the source comments or in a
separate note.

**The principle: clear in a few words.** One or two sentences per idea. If a
section needs a paragraph to make sense, the section is doing too much.

Same rules as [COMMENT_STYLE.md](COMMENT_STYLE.md): units on physical values,
no history, no restating what the file names already say.

## Required order

### 1. What this is

Two or three sentences. The controller runs a Franka arm through
approach → set up → grind, or holds a pose.

### 2. The files

Say the shape of the project in a short table, nothing more:

- `main.cpp` is **the** program: the control loop and the run state.
- The other `.cpp` files serve it — `control_math.cpp` the mathematics,
  `runtime_io.cpp` the printing and menus, `config.cpp` the parameter
  loading, `setup_report.cpp` the end-of-phase report.
- `params/*.txt` hold every tunable value, one topic per file. Nothing is
  compiled in.

The reader should finish this section knowing where to change a number and
where to change behaviour.

### 3. How to start it

The build and run commands, and nothing between them:

```text
make
./surface_grinding_controller
```

Then one line saying the startup menu appears and the robot does not move
until a choice is made.

### 4. The startup menu

One line per key. This is the most-read part of the README — keep it a list,
never prose.

```text
s  go to q_init, then run the sequence
h  go to q_init, then hold that pose
g  go to q_init, then hand-guide; pick s or h afterwards
q  go to q_init and look at the posture
o  open the hand    c  grasp the tool    m  home the hand
```

### 5. What each mode does, and which files tune it

One short block per mode. State what happens, then name the parameter files
that matter — file names only, not individual keys.

```text
Sequence (s)
  Orients the tool, descends to the clearance, presses, then grinds.
  Tunes: Approach_Phase, Clearance_Gate, SetUp_Phase, Grind_Phase.

Hold (h)
  Locks the pose it starts from and asks for the nullspace mode 0-3.
  Tunes: hold, Nullspace.

Guiding (g)
  Compliant hand-guidance; the pose you leave becomes the start pose.
  Tunes: guidance.
```

Always shared: `Run_Settings`, `Plane_Definition`, `Tool_Orientation`,
`Tool_Geometry`, `Q_Init`, `Gripper_Action`, `Auto_Damping`, `safety`.

### 6. Everything else

Anything deeper — the task frame, coupled stiffness, the sigma diagnostics —
goes after those five sections, or in its own document. A reader who stops at
the mode table must still be able to run an experiment.

## What to leave out

- Derivations and formulas. The one place they help is a variable's unit.
- Rationale for a chosen value. That is a comment next to the value.
- Anything already stated by a file name or a menu line.
- Changelogs. Git holds them.

## Updating

Change this document first when the rules stop fitting, then the README. When
a menu key or a parameter file is added or renamed, sections 4 and 5 are the
two places that must be updated with it.
