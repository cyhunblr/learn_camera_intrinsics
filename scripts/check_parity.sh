#!/usr/bin/env bash
# Prove the three ports agree, instead of asserting it in the README.
#
#   ./scripts/check_parity.sh
#
# Dumps the same quantities from Python, C++ and the viewer's JavaScript and
# diffs them byte for byte. Needs uv, a built cpp/build, and node.
set -euo pipefail
cd "$(dirname "$0")/.."
tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT

cat > "$tmp/cases.txt" <<'CASES'
812 809.5 639.1 361.4 -0.27 0.10 0.0012 -0.0009 0.004 1280 720
332.8 332.8 314 244 -0.34 0.11 0.0008 -0.0006 0 640 480
198 198 220 165 -0.35 0.12 0 0 0 440 330
500 500 320 240 0.22 -0.06 0 0 0 640 480
400 400 320 240 -0.5 0 0 0 0 640 480
CASES

uv run python scripts/parity_dump.py < "$tmp/cases.txt" > "$tmp/py.txt"
node scripts/parity_dump.js          < "$tmp/cases.txt" > "$tmp/js.txt"
cpp/build/bin/parity_dump            < "$tmp/cases.txt" > "$tmp/cpp.txt"

fail=0
for other in js cpp; do
  if diff -u "$tmp/py.txt" "$tmp/$other.txt" > "$tmp/d.$other"; then
    echo "  python == $other"
  else
    echo "  python != $other"; sed -n '1,25p' "$tmp/d.$other"; fail=1
  fi
done
[ "$fail" -eq 0 ] && echo "all three ports agree" || exit 1
