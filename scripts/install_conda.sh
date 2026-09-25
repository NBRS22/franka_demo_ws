#!/usr/bin/env bash
# Installs Miniconda (no sudo, no shell modification) if conda is not already available.
# Usage: scripts/install_conda.sh            (installs into ~/miniconda3, or $CONDA_PREFIX_DIR)
set -euo pipefail

PREFIX="${CONDA_PREFIX_DIR:-$HOME/miniconda3}"

if [ -x "$PREFIX/bin/conda" ]; then
  echo "conda already installed: $("$PREFIX/bin/conda" --version) ($PREFIX)"; exit 0
fi
if command -v conda >/dev/null 2>&1; then
  echo "conda already on PATH: $(conda --version) ($(command -v conda))"; exit 0
fi

echo "==> Downloading the Miniconda installer"
curl -fsSL -o /tmp/miniconda_installer.sh https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
echo "==> Installing into $PREFIX"
bash /tmp/miniconda_installer.sh -b -p "$PREFIX"
rm -f /tmp/miniconda_installer.sh
echo "Installed: $("$PREFIX/bin/conda" --version)"
echo "Nothing was added to your shell configuration. To use 'conda activate' in a terminal, run once:"
echo "    $PREFIX/bin/conda init bash      # then open a new terminal"
echo "and keep the 'base' environment deactivated when running ROS 2 (conda config --set auto_activate_base false)."
