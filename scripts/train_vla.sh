#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ "$#" -lt 3 ]; then
  echo "Usage: $0 BASE_CHECKPOINT LEROBOT_DATASET LOCAL_VLM_ASSETS [extra lerobot arguments]" >&2
  exit 2
fi
BASE="$1"; DATA="$2"; VLM="$3"; shift 3
export PYTHONPATH="$ROOT/vendor/lerobot-smolvla-jetson/src${PYTHONPATH:+:$PYTHONPATH}"
export TOKENIZERS_PARALLELISM=false
exec "$ROOT/.venv-vla/bin/python" -m lerobot.scripts.train \
  --policy.path="$BASE" --policy.device=cuda --policy.vlm_model_name="$VLM" \
  --policy.chunk_size=10 --policy.n_action_steps=1 --policy.push_to_hub=false \
  --dataset.repo_id=local/edgepick --dataset.root="$DATA" --dataset.use_imagenet_stats=false \
  --batch_size=2 --steps=360000 --num_workers=0 --log_freq=50 --save_freq=1000 \
  --output_dir="$ROOT/models/vla_training" --job_name=edgepick_vla --policy.use_amp=false \
  --wandb.enable=false "$@"
