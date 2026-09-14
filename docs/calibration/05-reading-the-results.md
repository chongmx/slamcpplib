# Reading the calibration outcome

Every calibration in this folder ends the same way: a small file full of
numbers, and a report of how well those numbers fit the data. This document
is a field guide to both — what the files contain, which numbers are worth
actually looking at, and how to tell a trustworthy calibration from one that
merely finished without complaint.

## The output files

- **`camchain.yaml`** — the result of [camera calibration](01-camera-calibration.md).
  One entry per camera: its lens numbers (intrinsics), its distortion
  numbers, its resolution, and — for every camera after the first — the
  transform chaining it back to camera zero.
- **`camchain-imucam.yaml`** — the result of
  [camera-IMU calibration](02-camera-imu-calibration.md). Everything in
  `camchain.yaml`, plus each camera's mounting transform to the IMU and its
  timing offset.
- **`imu.yaml`** — the IMU's own noise characteristics. This one is mostly
  an *input* rather than an output — it describes, in the manufacturer's or
  a prior calibration's terms, how jittery a single reading tends to be
  (**noise density**) and how fast the sensor's own bias wanders on its own
  (**random walk**). These numbers control how strongly the bias curves in
  camera-IMU calibration are allowed to bend; see
  [02-camera-imu-calibration.md](02-camera-imu-calibration.md#the-drifting-biases).
- **A report**, with plots and statistics for visual inspection — this is
  where you actually look at the shape of the reprojection error and the
  residuals, rather than just their averages.

## The one number to always check: reprojection error

Reprojection error is, on average, how many pixels off the fine-tuned
prediction was from where a target corner was actually seen. It's the
single most direct measure of whether a calibration succeeded, because
every calibration in this folder is, underneath, driven by shrinking exactly
this number (plus, for camera-IMU calibration, the accelerometer and
gyroscope residuals alongside it).

What to actually look for:

- **Small and consistent, not just small on average.** A low mean hiding one
  camera, or one cluster of views, with a much larger error is a sign that
  something specific went wrong there — a mis-synced camera, a section of
  the recording with more motion blur, or a genuinely different problem
  worth investigating rather than averaging away.
- **Stable after outlier rejection, not just before it.** Both camera
  calibration and camera-IMU calibration automatically discard the worst
  corner detections. If the error is still large *after* that cleanup, the
  fine-tuning process isn't failing to try hard enough — something
  structural is wrong: a mis-measured target, a rig that isn't actually
  rigid, wrong camera model chosen for the lens, or (for a scanning camera)
  a missing [rolling-shutter correction](04-rolling-shutter-calibration.md).
- **For camera-IMU calibration, check the accelerometer and gyroscope
  residuals too**, not just the reprojection error. They're in the sensor's
  own units rather than pixels, but the same logic applies: small and
  consistent is good, and a residual that stays large points at a specific,
  investigatable cause rather than "needs more iterations."

## Covariance: how confident the result actually is

Alongside the fitted numbers, the calibration reports a **covariance** — a
measure of how much uncertainty remains in each number once fine-tuning has
finished. Roughly: an axis of the mounting offset that the rig rotated
around freely and often ends up with tight, confident uncertainty; an axis
it barely moved around ends up wide and uncertain, because the data simply
never demonstrated it.

This is worth checking specifically for the case described in
[02-camera-imu-calibration.md](02-camera-imu-calibration.md#the-mounting-offset-extrinsics):
a lever-arm translation that the dataset never actually excited. If the
covariance along one direction is unusually large, that's the calibration
being honest about not having learned that direction — which is the
correct, useful outcome. It's a much better sign than the alternative below.

## The trap: confidently wrong

A fine-tuning process can converge, report small residuals, and report
tight uncertainty — and still have quietly gotten a number wrong, if the
recorded data never actually varied enough to expose the mistake. This
mostly shows up with the more detailed, optional IMU intrinsics: request
scale-and-misalignment or size-effect corrections without giving the rig
the strong-acceleration-while-rotating motion those specific corrections
need, and the fine-tuning can settle on plausible-looking numbers that
aren't actually reliable, without the report necessarily screaming about it.

The defence against this isn't a number to check after the fact — it's
making sure the recording had the motion variety each calibration document's
"Getting a good dataset" section describes, before you rely on the result.
A second, independent recording that reproduces close to the same numbers is
the most convincing check available: the true mounting geometry doesn't
change between sessions, even if the fitted numbers wiggle slightly from
one recording's noise to the next.

## Outcomes that are correct, not broken

A few results look like failures but are actually the calibration behaving
correctly by refusing to manufacture an answer it has no basis for:

- **A disconnected camera graph** ([01-camera-calibration.md](01-camera-calibration.md)) —
  cameras on the rig that never shared a view of the target with each other,
  directly or through a chain. There is genuinely no measurement to be made
  here; the fix is to recapture data with more overlap, not to force a
  number out of the existing data.
- **An unmeasurable lever arm** ([02-camera-imu-calibration.md](02-camera-imu-calibration.md#the-mounting-offset-extrinsics)) —
  a dataset with no rotational motion simply contains no information about
  the camera-to-IMU translation, however long it runs.
- **A run that doesn't converge** — this usually means the starting guess
  was too far off, the dataset is too sparse, or (for camera-IMU
  calibration) the rough time-offset or orientation prior landed somewhere
  the fine-tuning couldn't recover from. Recapturing with the "Getting a
  good dataset" guidance from the relevant document is the usual fix.

Treat all three as diagnostic information about the recorded data, not as
crashes to route around.
