#!/usr/bin/env bash
set -euo pipefail
PACKAGE="${1:?usage: promotion-submit-validate.sh <promotion-package.yaml>}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
exec "$SCRIPT_DIR/../scripts/llm-wiki-core" validate-promotion "$PACKAGE"
