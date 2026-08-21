# Traffic-light YOLO dataset

This dataset contains all 2019 JPEG images from `data/raw/sim`.
The source images were copied and were not modified.

## Split policy

- `train`: run_01 through run_03 (1216 images)
- `val`: run_04 (401 images)
- `test`: run_05 (402 images)

Sequential frames stay in the same split. Exact duplicate hashes were checked
and do not cross split boundaries.

## Classes

- `0`: red (622 labels)
- `1`: yellow (652 labels)
- `2`: green (473 labels)
- empty/background: 272 images with zero-byte label files

Bounding boxes cover the full visible traffic-light housing. Images that are
visually unclear are represented by empty label files. One file placed in the
`unclear` source folder is an exact duplicate of a clearly illuminated green
frame; it is labeled green to prevent contradictory targets for identical
pixels. `manifest.csv` records the source path, hash, split, class, pixel box,
normalized YOLO values, and the decision source for every image.

The `review/contact_sheets` directory is for quality assurance only and is not
used by YOLO training.
