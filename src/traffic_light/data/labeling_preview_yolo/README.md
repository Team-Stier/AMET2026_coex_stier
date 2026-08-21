# Traffic-light YOLO labeling preview

This directory contains ten copied preview images. The original images under
`data/raw/sim` are unchanged.

Class IDs:

- `0`: red
- `1`: yellow
- `2`: green

Bounding boxes cover the full visible traffic-light housing. Coordinates use
the YOLO format: `class_id x_center y_center width height`, normalized to the
480 x 360 image size.

`10_unclear_medium_run03.txt` is intentionally empty so that its paired image
is treated as a negative/background sample.

This is a labeling preview only. The same ten images are referenced as both
`train` and `val` in `data.yaml` so that dataset tooling can open the preview;
they must be split properly before model training.
