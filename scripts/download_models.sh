#!/usr/bin/env bash
# Downloads the model weights that are not versioned in this repository.
# Usage: scripts/download_models.sh [graspgen] [sam3] [--all-graspgen]     (default: graspgen sam3)
#   graspgen       Franka Panda checkpoints (about 1 GB) into GraspGen/GraspGenModels/  (public, no account needed)
#   --all-graspgen every gripper's checkpoints + sample data (about 8 GB)
#   sam3           SAM 3 weights into the Hugging Face cache (~/.cache/huggingface). GATED: request access at
#                  https://huggingface.co/facebook/sam3 and run `hf auth login` (or `huggingface-cli login`) first.
# Variables: ENV_SUFFIX (conda env suffix, default none), GRASPGEN_MODELS_DIR (destination override).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SUFFIX="${ENV_SUFFIX:-}"
DEST="${GRASPGEN_MODELS_DIR:-$ROOT/GraspGen/GraspGenModels}"

find_conda() {
  for c in "${CONDA_EXE:-}" "$(command -v conda 2>/dev/null || true)" "$HOME/miniconda3/bin/conda" "$HOME/anaconda3/bin/conda" \
           "$HOME/miniforge3/bin/conda" "$HOME/mambaforge/bin/conda" /opt/conda/bin/conda; do
    [ -n "$c" ] && [ -x "$c" ] && { echo "$c"; return; }
  done
  echo "conda not found: run scripts/install_conda.sh first." >&2; exit 1
}
eval "$("$(find_conda)" shell.bash hook)"

ALL=0; TARGETS=()
for a in "$@"; do [ "$a" = "--all-graspgen" ] && ALL=1 || TARGETS+=("$a"); done
[ ${#TARGETS[@]} -eq 0 ] && TARGETS=(graspgen sam3)

for t in "${TARGETS[@]}"; do
  case "$t" in
    graspgen)
      echo "==> GraspGen checkpoints -> $DEST"
      conda activate "SAM3$SUFFIX"          # any env with huggingface_hub works
      python - "$DEST" "$ALL" <<'PY'
import sys
from huggingface_hub import snapshot_download
dest, everything = sys.argv[1], sys.argv[2] == "1"
patterns = None if everything else ["checkpoints/graspgen_franka_panda*", "sample_data/meshes/*", "README.md", "LICENSE"]
snapshot_download(repo_id="adithyamurali/GraspGenModels", local_dir=dest, allow_patterns=patterns)
print("done:", dest)
PY
      conda deactivate ;;
    sam3)
      echo "==> SAM 3 weights -> Hugging Face cache"
      conda activate "SAM3$SUFFIX"
      python - <<'PY'
import sys
from huggingface_hub import whoami
try:
    print("Hugging Face user:", whoami()["name"])
except Exception:
    sys.exit("Not logged in to Hugging Face: request access to facebook/sam3, then run `hf auth login` and retry.")
PY
      (cd "$ROOT/SAM3" && python -c "from sam3.model_builder import build_sam3_image_model; build_sam3_image_model(); print('SAM3 weights downloaded')")
      conda deactivate ;;
    *) echo "unknown target: $t" >&2; exit 1 ;;
  esac
done
