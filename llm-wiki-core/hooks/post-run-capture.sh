#!/usr/bin/env bash
set -euo pipefail
ROOT="${1:-.}"
TITLE="${2:-Agent run capture}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
exec "$SCRIPT_DIR/../scripts/llm-wiki-core" --root "$ROOT" capture-run --title "$TITLE"
