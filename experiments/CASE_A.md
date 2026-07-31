# Case A -- what it tests

**The question: when the tool is commanded at a known angle to the surface,
how much of that angle does contact remove?**

Everything else is held fixed -- gains, preload, plane, timing, start pose --
so the only thing changing across the five setups is the commanded angle.

## The five setups

| Setup | Commanded offset | Repeats |
|---|---|---|
| `MAIN_A0_00deg` | none | 3 |
| `MAIN_A1_t1_05deg` | +5 deg about t1 | 3 |
| `MAIN_A2_t1_10deg` | +10 deg about t1 | 3 |
| `MAIN_A3_t2_05deg` | +5 deg about t2 | 3 |
| `MAIN_A4_t2_10deg` | +10 deg about t2 | 3 |

15 trials.

## What each one answers

**A0 -- is the rig honest?** Command zero and the tool should already lie flat.
Whatever alignment error it still shows is the floor: no later result can claim
to resolve anything smaller. It is the calibration gate for the whole campaign.

**A1 vs A2, and A3 vs A4 -- does the correction scale?** Double the commanded
angle and see whether the correction doubles, saturates, or stays flat. That
distinguishes a spring-like response from a geometric limit.

**t1 versus t2 -- does the axis matter?** The face is 40 x 120 mm, so tipping
it about its long axis is not the same mechanical problem as tipping it about
its short one. If the two axes behave differently, the response is geometric,
not just a property of the impedance.

## The measurement

Per trial: alignment against the calibrated physical plane before and after
set-up, its signed t1 and t2 components, the fraction of error removed, and
how long it took. Plots use the **measured first-contact angle**, not the
nominal command -- the commanded angle and the real one differ.

## What Case A gives the rest of the campaign

A baseline at fixed gains. Cases B and C then vary rotational and
translational stiffness against it, and Case D moves the centre of compliance.
Without A, none of those have anything to be compared against.

## Caveat carried by every trial

The tool can rotate roughly 2 deg in the gripper. The controller is unaffected
-- it commands and regulates the gripper, and the loop never sees the tool --
but the alignment metric infers the tool's orientation from the gripper pose
through a transform assumed rigid. That slip is an unquantified contribution
to the reported angles. See the mount-status note in `AXIS_STUDY.md`.
