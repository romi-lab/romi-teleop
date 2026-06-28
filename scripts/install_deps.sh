#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ ! -d third_party/lerobot ]]; then
  echo "Missing third_party/lerobot. Initialize submodules first." >&2
  echo "Run: git submodule update --init --recursive" >&2
  exit 1
fi

git submodule update --init --recursive

python -m pip install --upgrade pip setuptools wheel

if command -v conda >/dev/null 2>&1; then
  conda install -y -c conda-forge ffmpeg
else
  echo "conda not found; skipping conda ffmpeg install." >&2
  echo "Install ffmpeg manually if LeRobot video decoding fails." >&2
fi

python -m pip install -r requirements.txt

echo "Installation complete."
echo "Sanity checks:"
echo "  python -c 'import lerobot; print(lerobot.__file__)'"
echo "  lerobot-info"
echo "  lerobot-find-port"
