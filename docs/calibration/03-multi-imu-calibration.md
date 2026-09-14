# Calibrating more than one IMU

Some rigs carry more than one IMU — for redundancy, or because different
IMUs are being compared or fused for extra robustness. This calibration
finds how each *extra* IMU relates to a single **reference IMU**, reusing
almost everything from [02-camera-imu-calibration.md](02-camera-imu-calibration.md).
Read that document first; this one only covers what's different.

## Why a camera is still required

It's tempting to assume that calibrating IMU against IMU shouldn't need a
camera at all. It does, for a basic reason: an IMU only ever measures how
motion is *changing* — how fast it's turning, how hard it's accelerating.
It has no way to say where the rig actually is or was, only how its state is
evolving moment to moment. Left with only IMU data, the shared trajectory
curve described in the previous document would be free to drift into any
number of trajectories that all imply the same felt rotations and
accelerations, and there would be nothing to pin it to reality.

The camera is what pins the trajectory curve to a fixed frame in the world,
by tying it to a calibration target with known, fixed geometry. So even when
the goal is purely to relate IMUs to each other, at least one camera
watching a calibration target is still required, to give the shared
trajectory curve something absolute to be measured against.

## What's different from camera-IMU calibration

Everything from the previous document carries over unchanged for the
reference IMU. Each additional IMU then gets:

- **Its own mounting offset**, relative to the reference IMU rather than to
  a camera — same idea, same "rotation needs excitation" caveat from the
  previous document.
- **Its own timing offset**, found the same cross-correlation way described
  previously, except the gyroscope signal it's aligned against is the
  reference IMU's gyroscope rather than a camera-derived rotation estimate.
- **Its own bias curves**, found and regularised exactly as described
  previously.

All of it is fine-tuned together with everything else, against the same
single shared trajectory curve — there's still only one curve describing the
rig's motion; every sensor on the rig, camera or IMU, reference or not, is
just another set of predictions read off that one curve.

## What you get out

One mounting transform and one timing offset per non-reference IMU, plus
bias curves for every IMU including the reference one. The same guidance
from [05-reading-the-results.md](05-reading-the-results.md) applies: check
that residuals are small and consistent, and that the rig was actually
rotated enough during recording for each extra IMU's offset to be a
meaningful measurement rather than a guess dressed up as a number.
