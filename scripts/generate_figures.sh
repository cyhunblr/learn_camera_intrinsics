#!/usr/bin/env bash
# Regenerate every figure in data/generated/ from the Python side.
#
#   uv sync && ./scripts/generate_figures.sh
#
# The C++ binaries produce identical images; this script uses Python only
# because it needs no build step.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p data/generated

uv run python/figures/render_2d.py --out data/generated/app2d.png \
        --preset "action cam (strong barrel)" --alpha 0.0
uv run python/figures/render_3d.py --out data/generated/app3d.png \
        --preset "action cam (strong barrel)"
uv run python/examples/02_k_matrix_anatomy.py    --save data/generated/ex02_k_anatomy.png  > /dev/null
uv run python/examples/03_distortion_anatomy.py  --save data/generated/ex03_distortion.png > /dev/null
uv run python/examples/05_undistort_and_alpha.py --save data/generated/ex05_alpha.png      > /dev/null
uv run python/quiz/export_markdown.py

# Palette-quantise the screenshots: they are flat-coloured UI, so this cuts the
# repository weight by ~4x with no visible change. Pillow is in the dev group,
# so this is skipped after `uv sync --no-dev`.
# The viewer screenshots the README shows. Needs a browser, so it is skipped
# without one -- the rest of the repository stays runnable headless.
BROWSER=$(command -v google-chrome || command -v chromium || true)
if [ -n "$BROWSER" ]; then
  for tab in 3d 2d; do
    "$BROWSER" --headless=new --disable-gpu --no-sandbox \
      --user-data-dir="$(mktemp -d)" --window-size=1680,1000 \
      --force-device-scale-factor=1 --virtual-time-budget=5000 --hide-scrollbars \
      --screenshot="data/generated/viewer_$tab.png" \
      "file://$PWD/web/index.html#$tab" >/dev/null 2>&1 \
      && echo "wrote data/generated/viewer_$tab.png"
  done
else
  echo "(no chrome/chromium found -- viewer_*.png left as committed)"
fi

uv run python - <<'PY' || echo "(Pillow unavailable -- figures left uncompressed)"
import os
from PIL import Image
for f in sorted(os.listdir("data/generated")):
    if not f.endswith(".png"):
        continue
    p = os.path.join("data/generated", f)
    Image.open(p).convert("RGB").quantize(colors=192).save(p, optimize=True)
PY

echo
ls -1 data/generated
