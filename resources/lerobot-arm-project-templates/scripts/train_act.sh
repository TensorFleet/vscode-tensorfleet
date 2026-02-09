#!/bin/bash
# train_act.sh - Train or resume ACT policy on SO-ARM101 dataset
#
# Usage:
#   ./scripts/train_act.sh <dataset_root> <repo_id> [options]
#   ./scripts/train_act.sh --resume <output_dir>
#
# Examples:
#   # New training
#   ./scripts/train_act.sh ./datasets/so_arm101_sim_20260206 local/so_arm101_sim_20260206
#   ./scripts/train_act.sh ./datasets/so_arm101_sim_20260206 local/so_arm101_sim_20260206 --wandb.enable=true
#
#   # Resume training
#   ./scripts/train_act.sh --resume outputs/train/act_local_so_arm101_sim_20260206
#   ./scripts/train_act.sh --resume outputs/train/act_local_so_arm101_sim_20260206 --steps=50000

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Check for resume mode
if [ "$1" == "--resume" ]; then
    OUTPUT_DIR="${2:-}"
    shift 2 2>/dev/null || true
    EXTRA_ARGS="$@"
    
    if [ -z "$OUTPUT_DIR" ]; then
        echo -e "${RED}Error: --resume requires an output directory${NC}"
        echo ""
        echo "Usage: $0 --resume <output_dir> [extra_args...]"
        echo ""
        echo "Example:"
        echo "  $0 --resume outputs/train/act_local_so_arm101_sim_20260206"
        echo "  $0 --resume outputs/train/act_local_so_arm101_sim_20260206 --steps=50000"
        exit 1
    fi
    
    # Expand tilde in path
    OUTPUT_DIR="${OUTPUT_DIR/#\~/$HOME}"
    
    # Make path absolute if relative
    if [[ "$OUTPUT_DIR" != /* ]]; then
        OUTPUT_DIR="$PROJECT_ROOT/$OUTPUT_DIR"
    fi
    
    CONFIG_PATH="$OUTPUT_DIR/checkpoints/last/pretrained_model/train_config.json"
    
    if [ ! -f "$CONFIG_PATH" ]; then
        echo -e "${RED}Error: Checkpoint not found${NC}"
        echo "Expected: $CONFIG_PATH"
        exit 1
    fi
    
    echo ""
    echo -e "${GREEN}=== Resuming ACT Training ===${NC}"
    echo "Output dir:   $OUTPUT_DIR"
    echo "Config:       $CONFIG_PATH"
    echo "Extra args:   $EXTRA_ARGS"
    echo ""
    
    cd "$PROJECT_ROOT"
    uv run lerobot-train \
        --config_path="$CONFIG_PATH" \
        --resume=true \
        $EXTRA_ARGS
    
    echo ""
    echo -e "${GREEN}=== Training Complete ===${NC}"
    echo "Checkpoints saved to: $OUTPUT_DIR/checkpoints/"
    exit 0
fi

# Regular training mode
DATASET_ROOT="${1:-}"
REPO_ID="${2:-}"
DEVICE="cuda"
WANDB_ENABLE="false"
PUSH_TO_HUB="false"

# Shift positional args to get extra options
shift 2 2>/dev/null || true
EXTRA_ARGS="$@"

# Validate arguments
if [ -z "$DATASET_ROOT" ] || [ -z "$REPO_ID" ]; then
    echo -e "${RED}Error: Missing required arguments${NC}"
    echo ""
    echo "Usage:"
    echo "  $0 <dataset_root> <repo_id> [extra_args...]     # New training"
    echo "  $0 --resume <output_dir> [extra_args...]        # Resume training"
    echo ""
    echo "Arguments:"
    echo "  dataset_root   Path to the dataset folder (contains meta/info.json)"
    echo "  repo_id        Dataset identifier (e.g., local/so_arm101_sim_20260206)"
    echo ""
    echo "Examples:"
    echo "  $0 ./datasets/so_arm101_sim_20260206 local/so_arm101_sim_20260206"
    echo "  $0 --resume outputs/train/act_local_so_arm101_sim_20260206"
    exit 1
fi

# Expand tilde in path
DATASET_ROOT="${DATASET_ROOT/#\~/$HOME}"

# Make path absolute if relative
if [[ "$DATASET_ROOT" != /* ]]; then
    DATASET_ROOT="$PROJECT_ROOT/$DATASET_ROOT"
fi

# Validate dataset exists
if [ ! -f "$DATASET_ROOT/meta/info.json" ]; then
    echo -e "${RED}Error: Dataset not found at $DATASET_ROOT${NC}"
    echo "Expected to find: $DATASET_ROOT/meta/info.json"
    exit 1
fi

# Extract job name from repo_id (replace / with _)
JOB_NAME="act_${REPO_ID//\//_}"
OUTPUT_DIR="$PROJECT_ROOT/outputs/train/${JOB_NAME}"

# Detect device
if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
    DEVICE="cuda"
    echo -e "${GREEN}Using CUDA GPU${NC}"
elif [ "$(uname)" == "Darwin" ]; then
    DEVICE="mps"
    echo -e "${GREEN}Using Apple Silicon (MPS)${NC}"
else
    DEVICE="cpu"
    echo -e "${YELLOW}Warning: No GPU detected, using CPU (training will be slow)${NC}"
fi

# Print configuration
echo ""
echo -e "${GREEN}=== ACT Training Configuration ===${NC}"
echo "Dataset root: $DATASET_ROOT"
echo "Repo ID:      $REPO_ID"
echo "Job name:     $JOB_NAME"
echo "Output dir:   $OUTPUT_DIR"
echo "Device:       $DEVICE"
echo "Extra args:   $EXTRA_ARGS"
echo ""

# Validate dataset
echo -e "${GREEN}Validating dataset...${NC}"
cd "$PROJECT_ROOT"
uv run python - <<PY
from lerobot.datasets.lerobot_dataset import LeRobotDataset

root = "$DATASET_ROOT"
repo = "$REPO_ID"

try:
    ds = LeRobotDataset(repo_id=repo, root=root)
    print(f"  Episodes: {ds.num_episodes}")
    print(f"  Frames:   {ds.num_frames}")
    print(f"  Features: {list(ds.features.keys())}")
    
    if ds.num_episodes < 10:
        print(f"\n  ⚠️  Warning: Only {ds.num_episodes} episode(s). Recommend 50+ for good results.")
except Exception as e:
    print(f"  ❌ Error loading dataset: {e}")
    exit(1)
PY

if [ $? -ne 0 ]; then
    echo -e "${RED}Dataset validation failed${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}Starting training...${NC}"
echo ""

# Default training parameters (can be overridden by environment variables)
TRAIN_STEPS="${TRAIN_STEPS:-100000}"
SAVE_FREQ="${SAVE_FREQ:-10000}"
BATCH_SIZE="${BATCH_SIZE:-8}"
NUM_WORKERS="${NUM_WORKERS:-4}"

# Run training
uv run lerobot-train \
    --dataset.repo_id="$REPO_ID" \
    --dataset.root="$DATASET_ROOT" \
    --policy.type=act \
    --output_dir="$OUTPUT_DIR" \
    --job_name="$JOB_NAME" \
    --policy.device="$DEVICE" \
    --wandb.enable="$WANDB_ENABLE" \
    --policy.push_to_hub="$PUSH_TO_HUB" \
    --steps=$TRAIN_STEPS \
    --save_freq=$SAVE_FREQ \
    --batch_size=$BATCH_SIZE \
    --num_workers=$NUM_WORKERS \
    $EXTRA_ARGS

echo ""
echo -e "${GREEN}=== Training Complete ===${NC}"
echo "Checkpoints saved to: $OUTPUT_DIR/checkpoints/"
echo ""
echo "To resume training:"
echo "  $0 --resume $OUTPUT_DIR"
echo "  $0 --resume $OUTPUT_DIR --steps=50000"
echo ""
echo "To run inference:"
echo "  uv run lerobot-eval --policy.path=$OUTPUT_DIR/checkpoints/last/pretrained_model --robot.type=so101_follower"
