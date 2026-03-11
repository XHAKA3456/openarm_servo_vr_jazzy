#!/bin/bash
source ~/miniconda3/etc/profile.d/conda.sh
conda run -n openarm_lerobot python3 /home/sst/openarm/src/openarm_lerobot/scripts/add_depth_v2.py \
  --dataset /home/sst/openarm/src/openarm_lerobot/datasets/openarm_pick_strawberry_and_place_on_white_plate \
  --model small --device cuda
