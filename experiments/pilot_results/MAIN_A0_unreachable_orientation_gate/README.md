# Rejected A0 diagnostic: unreachable orientation gate

This stopped zero-offset run used a 0.5 degree approach-orientation transition.
The mounted system reduced its free-space error from 5.7 degrees but settled
near 1.5 degrees. It consequently remained in `approach_orient` for about
24 seconds and never descended or entered set-up.

The run is not a contact experiment and must not enter the primary campaign.
It demonstrated that 0.5 degrees is an unreachable state-machine condition for
this hardware and gain set. The MAIN overlays therefore use the proven
2 degree transition. The controller continues regulating orientation during
descent, and the measured first-contact angle is retained as the experimental
initial condition.
