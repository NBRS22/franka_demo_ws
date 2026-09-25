#!/usr/bin/env bash
# Creates the conda environments of the project: ER, SAM3, GraspGen.
# Usage: scripts/setup_envs.sh [ER] [SAM3] [GraspGen]        (default: all three)
#   ENV_SUFFIX=_test scripts/setup_envs.sh   -> creates ER_test, SAM3_test, ... (for trials; the launch files expect no suffix)
# Needs: conda (scripts/install_conda.sh), an NVIDIA GPU with CUDA for SAM3/GraspGen,
#        and for GraspGen: nvcc, gcc-12 and g++-12 (sudo apt install gcc-12 g++-12 nvidia-cuda-toolkit).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SUFFIX="${ENV_SUFFIX:-}"

find_conda() {
  for c in "${CONDA_EXE:-}" "$(command -v conda 2>/dev/null || true)" "$HOME/miniconda3/bin/conda" "$HOME/anaconda3/bin/conda" \
           "$HOME/miniforge3/bin/conda" "$HOME/mambaforge/bin/conda" /opt/conda/bin/conda; do
    [ -n "$c" ] && [ -x "$c" ] && { echo "$c"; return; }
  done
  echo "conda not found: run scripts/install_conda.sh first." >&2; exit 1
}
CONDA="$(find_conda)"
eval "$("$CONDA" shell.bash hook)"

create_env() {  # <name> <python version>
  local name="$1$SUFFIX"
  if conda env list | awk '{print $1}' | grep -qx "$name"; then
    echo "==> env $name already exists (skipping creation; remove it with 'conda env remove -n $name' to rebuild)"; return 1
  fi
  # conda-forge only: avoids the Anaconda default-channel terms-of-service prompt
  conda create -y -n "$name" -c conda-forge --override-channels "python=$2"
  return 0
}

setup_ER() {
  create_env ER 3.12 || return 0
  conda activate "ER$SUFFIX"
  pip install opencv-python==4.9.0.80 numpy==1.26.4 pyzmq msgpack
}

setup_SAM3() {
  create_env SAM3 3.12 || return 0
  conda activate "SAM3$SUFFIX"
  pip install torch==2.10.0 torchvision --index-url https://download.pytorch.org/whl/cu128
  (cd "$ROOT/SAM3" && pip install -e ".[inference]")
  pip install "numpy==1.26.4" pyzmq msgpack matplotlib huggingface_hub    # numpy pinned: sam3 needs numpy<2
}

setup_GraspGen() {
  for tool in nvcc gcc-12 g++-12; do
    command -v "$tool" >/dev/null || { echo "GraspGen needs '$tool' (sudo apt install gcc-12 g++-12 nvidia-cuda-toolkit)" >&2; return 1; }
  done
  create_env GraspGen 3.10 || return 0
  conda activate "GraspGen$SUFFIX"
  pip install torch==2.1.0 torchvision==0.16.0 torch-cluster torch-scatter -f https://data.pyg.org/whl/torch-2.1.0+cu121.html
  (cd "$ROOT/GraspGen" && pip install -e . && ./install_pointnet.sh)
  pip install pyzmq msgpack msgpack-numpy
}

TARGETS=("$@"); [ ${#TARGETS[@]} -eq 0 ] && TARGETS=(ER SAM3 GraspGen)
for t in "${TARGETS[@]}"; do
  echo; echo "################ $t ################"
  "setup_$t"
  conda deactivate 2>/dev/null || true
done
echo; echo "Done. Environments: $(conda env list | awk '{print $1}' | grep -E "^(ER|SAM3|GraspGen)$SUFFIX\$" | tr '\n' ' ')"
echo "Next: scripts/download_models.sh"
