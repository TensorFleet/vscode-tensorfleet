#!/bin/bash
# train_act_overfit.sh - Train ACT policy on a small dataset (overfitting)
#
# Usage:
#   ./scripts/train_act_overfit.sh <dataset_root> <repo_id> [options]
#
# Example:
#   ./scripts/train_act_overfit.sh ./datasets/so_arm101_sim_20260206 local/so_arm101_sim_20260206

set -e

# Configuration for overfitting on small dataset (e.g. 5 episodes)
export TRAIN_STEPS=5000         # 5000 steps is enough to overfit 5 episodes
export SAVE_FREQ=1000           # Save frequently
export BATCH_SIZE=5             # Full batch (if 5 episodes)
export WANDB_ENABLE=false       # Disable wandb for quick local test

echo "=== Starting Overfit Training ==="
echo "Steps:      $TRAIN_STEPS"
echo "Batch Size: $BATCH_SIZE"
echo "Save Freq:  $SAVE_FREQ"
echo "================================="

# Pass arguments to the main training script with small model size
./scripts/train_act.sh "$@" \
    --policy.dim_model=256 \
    --policy.n_encoder_layers=2 \
    --policy.n_decoder_layers=2 \
    --policy.n_heads=4
