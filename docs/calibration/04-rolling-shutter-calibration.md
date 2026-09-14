# Rolling-shutter calibration: correcting for cameras that scan

Most of the calibrations in this folder quietly assume a camera captures an
entire image at a single instant, the way a flashbulb photograph freezes
everything at once. Many real cameras — most consumer and machine-vision
cameras that don't specifically advertise "global shutter" — don't actually
work that way. This document explains the correction needed when they don't,
and only applies to that kind of camera.

## Global shutter versus rolling shutter

A **global shutter** camera exposes every pixel at the same instant, like a
flashbulb: the whole image is one simultaneous snapshot.

A **rolling shutter** camera instead scans the image one row at a time, from
top to bottom, very quickly but not instantaneously. The top row of a photo
was genuinely captured a tiny fraction of a second before the bottom row.
Ordinarily this is invisible. It stops being invisible when the camera or
the scene is moving quickly during that scan: a fast-moving edge can appear
subtly slanted or skewed, because "the same photo" is secretly a stack of
thin strips, each taken at a very slightly different moment.

For calibration, this matters because every other document in this folder
treats "the instant a photo was taken" as one single, well-defined number.
For a rolling-shutter camera, that's not quite true — it depends on *which
row of the image* a particular target corner happened to land on.

## The one extra number: line delay

The correction needed is a single number per camera: the **line delay**, the
extra sliver of time between when the scan starts on one row and when it
starts on the next. A camera's known frame rate and row count give a
reasonable starting guess for it (the interval between rows is roughly the
time to capture one whole frame, divided by the number of rows), and
fine-tuning refines it from there.

<img src="../img/line-delay-complete-system-integration.jpg" alt="Rolling shutter line delay: a corner detected on a given image row samples the continuous motion trajectory at a time offset by that row's line delay, rather than at one single instant for the whole frame" width="900">

The left half of the diagram above shows how this plays out. Instead of
every corner in a photo being compared against the shared trajectory curve
(introduced in [02-camera-imu-calibration.md](02-camera-imu-calibration.md))
at one shared instant, each corner's comparison instant is nudged by its own
row number times the line delay — a corner near the top of the frame samples
the curve slightly earlier than a corner near the bottom of the same photo.
Once that row-dependent shift is folded in, the line delay becomes just one
more number in the same fine-tuning process as everything else: adjust it,
along with the lens shape, the mounting offset, and so on, until every
corner's predicted position lines up with where it was actually detected —
row-by-row shift included.

This calibration also lets the trajectory curve add a little extra
flexibility exactly where it's needed, rather than everywhere at once: after
an initial fine-tuning pass, wherever the leftover error is concentrated,
the curve is given a bit more detail right there and the process repeats.
This keeps the curve simple where the motion was simple, and only lets it
get more intricate where the data actually demands it.

## What you get out, and what "good" looks like

The result adds one small number to the calibrated camera's entry: its line
delay, typically a small fraction of a millisecond per row. As with every
other calibration here, the reprojection error is the number to watch — see
[05-reading-the-results.md](05-reading-the-results.md).

A telling sign that a camera actually needs this calibration in the first
place: if a camera is run through the ordinary calibration assuming a global
shutter, but the rig was moved quickly during capture, the reprojection
error tends to stay stubbornly elevated in a way that no amount of
re-fitting the lens shape fixes. That pattern — otherwise sound data, but an
error that won't go away — is a reasonable hint to check whether the camera
is actually rolling-shutter and re-run this calibration instead.

## Getting a good dataset

- **This step is unnecessary for global-shutter cameras.** Their line delay
  is simply zero, and forcing this calibration on them adds nothing but
  noise.
- **It matters most when the rig moves quickly** during capture — a slow,
  careful pan gives the row-dependent skew very little to show itself with.
  Include some genuinely fast motion in the recording.
- The same general advice from [01-camera-calibration.md](01-camera-calibration.md)
  about varied distance, angle, and image coverage still applies underneath
  this correction.
