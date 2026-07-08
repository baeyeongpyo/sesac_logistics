#!/usr/bin/env bash
set -euo pipefail
ROOT="${1:-.}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BIN="$SCRIPT_DIR/../scripts/llm-wiki-core"

RESULT=$("$BIN" --root "$ROOT" validate 2>&1 || true)

python3 - "$RESULT" <<'PY' || true
import json, sys
raw = sys.argv[1]
try:
    result = json.loads(raw)
except json.JSONDecodeError:
    print("wiki_stack validation: COULD NOT RUN")
    print("Cause: the validate command did not produce valid JSON output. Raw output:")
    for line in (raw.splitlines() or ["(empty output)"]):
        print(f"  {line}")
    print("Fix: run 'llm-wiki-core/scripts/llm-wiki-core --root . validate' directly to see the full error,")
    print("then check llm-wiki-core/skills/research/llm-wiki-core-environment-setup/SKILL.md for the setup workflow.")
    sys.exit(0)
if result.get("ok"):
    print("wiki_stack validation: ok")
else:
    print("wiki_stack validation: FAILED")
    for err in result.get("errors", []):
        print(f"  - {err}")
    print("Fix: edit wiki_stack.yaml to resolve the errors above, then re-run:")
    print("  llm-wiki-core/scripts/llm-wiki-core --root . validate")
    print("See llm-wiki-core/skills/research/llm-wiki-core-environment-setup/SKILL.md for the full workflow.")
PY

exit 0
