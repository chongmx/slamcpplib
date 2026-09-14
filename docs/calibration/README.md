# Calibration, explained without the mathematics

This folder explains what "calibrating" a camera rig and its IMU actually
means, how the calibration process figures out its answers, and how to read
the numbers it hands back. It is written for anyone who needs to understand
*what calibration does and why it matters*, not for anyone who needs to
implement it. This library's calibration module is a port of
[Kalibr](https://github.com/ethz-asl/kalibr) (ethz-asl); if this repository
is checked out inside a superproject that also vendors a `docs/kalibr/`
engineering specification for that port, that is the place to look for
implementation-level detail — design-variable groups, gauge fixing, solver
internals, and so on. Nothing here depends on it.

## Why bother calibrating anything?

Every sensor lies a little. A camera's lens bends light slightly differently
near the edges of the image than in the middle. An IMU's accelerometer reads
a number that is close to, but not exactly, the acceleration it actually
felt. If two cameras are bolted to the same rig, nobody hand-measured the
half-a-millimetre gap and the fraction-of-a-degree tilt between them to
engineering precision.

None of that matters if you only ever look at pictures. It matters enormously
if you're trying to reconstruct where a robot is and what it saw, because a
SLAM system takes all of these small lies at face value unless it is told
otherwise. A lens that's slightly misjudged bends the entire map. An
IMU-to-camera offset that's off by a centimetre puts a permanent, systematic
wobble into every estimate that uses both sensors together. Calibration is
the process of measuring these small, fixed imperfections *once*, carefully,
so that everything built afterwards can correct for them automatically.

It is exactly the same idea as zeroing a kitchen scale before you weigh
anything on it — except here there are a dozen different "zero points" to
find, some of them changing slowly over time, and finding them requires
solving a fairly large puzzle rather than pressing a button.

## The four things that get calibrated

This library (following Kalibr) breaks calibration into four pipelines. Each
one is documented separately, and each builds on the one before it:

| Document | What it figures out |
| --- | --- |
| [01-camera-calibration.md](01-camera-calibration.md) | Each camera's lens shape (intrinsics), and how multiple cameras are positioned relative to each other (extrinsics) |
| [02-camera-imu-calibration.md](02-camera-imu-calibration.md) | How the IMU is mounted relative to the camera, the timing offset between the two, and how the IMU's own errors drift over time |
| [03-multi-imu-calibration.md](03-multi-imu-calibration.md) | The same, when there is more than one IMU on the rig |
| [04-rolling-shutter-calibration.md](04-rolling-shutter-calibration.md) | An extra correction needed for cameras that scan an image row by row instead of capturing it all at once |

Read them in that order — each one leans on ideas introduced in the previous
document, particularly the continuous-motion idea introduced in
[02-camera-imu-calibration.md](02-camera-imu-calibration.md).

Once you've run a calibration, [05-reading-the-results.md](05-reading-the-results.md)
explains what comes out the other end: the output files, the handful of
numbers worth actually looking at, and how to tell a good calibration from a
quietly broken one.

## The shape of every calibration in this library

Underneath the four pipelines above, every calibration in this library
follows the same two-step recipe:

1. **A rough guess.** Some quick, closed-form arithmetic gets every unknown
   number into roughly the right neighbourhood — close enough that the next
   step won't get lost.
2. **Fine-tuning against the data.** All the rough guesses are thrown into one
   big optimisation problem together, alongside every photo and every sensor
   reading that was recorded. The numbers are then nudged, repeatedly, in
   whatever direction makes their predictions line up better with what was
   actually observed, until nudging further stops helping.

That second step is doing essentially the same thing every time: comparing a
*prediction* (where should this corner appear on the sensor, given the
current guess for the lens shape and rig geometry?) against a *measurement*
(where did the corner actually appear?), and reducing the gap. The rest of
these documents are mostly about what gets predicted, what gets measured, and
why each calibration needs the particular kind of data it needs.

## Status of the port in this repository

These documents describe what the calibration pipelines compute — the
target behaviour this library is being built to reproduce, faithfully
ported from Kalibr. They are accurate regardless of how much of the port is
finished at any given moment; see [`include/slamcpp/calibration/Calibration.h`](../../include/slamcpp/calibration/Calibration.h)
for the current layer-by-layer build status and the phased plan it's
following.
