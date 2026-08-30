#!/usr/bin/env bash
# Lint every Markdown file in the repository.
#
#   ./scripts/check_markdown.sh          report problems
#   ./scripts/check_markdown.sh --fix    fix the ones that can be fixed
#
# Rules live in .markdownlint-cli2.jsonc, next to this repository's other
# configuration. Needs node; markdownlint is fetched on demand rather than
# committed as a package.json, the same way commitlint is.
set -euo pipefail
cd "$(dirname "$0")/.."

VERSION="0.22.1"           # pinned, so a new release cannot fail CI on its own

args=()
[ "${1:-}" = "--fix" ] && args+=(--fix)

npx --yes "markdownlint-cli2@${VERSION}" "${args[@]}"
