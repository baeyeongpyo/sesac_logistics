#!/usr/bin/env bash
set -euo pipefail

AGENT="generic"
ROOT=""

while [ "$#" -gt 0 ]; do
  case "$1" in
    --agent)
      AGENT="${2:-generic}"
      shift 2
      ;;
    --root)
      ROOT="${2:-}"
      shift 2
      ;;
    *)
      shift
      ;;
  esac
done

if [ -z "$ROOT" ]; then
  ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BIN="$SCRIPT_DIR/../scripts/llm-wiki-core"

RESULT=$("$BIN" --root "$ROOT" get-context-bundle 2>&1 || true)

python3 - "$RESULT" "$AGENT" <<'PY'
import json
import sys
from pathlib import Path

raw = sys.argv[1]
agent = sys.argv[2]

def emit(text: str) -> None:
    text = text[:9000]
    if agent == "claude":
        print(json.dumps({
            "hookSpecificOutput": {
                "hookEventName": "UserPromptSubmit",
                "additionalContext": text,
            },
            "suppressOutput": True,
        }, ensure_ascii=False))
    else:
        print(text)

try:
    result = json.loads(raw)
except json.JSONDecodeError:
    emit(
        "llm-wiki-core context reminder:\n"
        "- Bundle generation failed before this prompt.\n"
        "- Do not answer project-specific questions from general knowledge.\n"
        "- Run `llm-wiki-core/scripts/llm-wiki-core --root . get-context-bundle` and inspect the error.\n"
        "- Raw hook output:\n"
        + "\n".join(f"  {line}" for line in (raw.splitlines() or ["(empty output)"]))
    )
    raise SystemExit(0)

output_dir = result.get("output_dir")
if not output_dir:
    emit(
        "llm-wiki-core context reminder:\n"
        "- No context bundle path was returned.\n"
        "- Do not answer project-specific questions from general knowledge.\n"
        "- Run `llm-wiki-core/scripts/llm-wiki-core --root . get-context-bundle` and inspect the result."
    )
    raise SystemExit(0)

bundle_dir = Path(output_dir)
required = ["context_bundle.md", "warnings.yaml", "selected_pages.yaml", "source_lineage.yaml"]
lines = [
    "llm-wiki-core context reminder:",
    "- Before answering project-specific questions, inspect the newest bundle first.",
    "- Do not answer from general knowledge before checking whether selected wiki pages contain relevant project knowledge.",
    f"- Bundle: {bundle_dir}",
    "- Required context files:",
]
for name in required:
    lines.append(f"  - {bundle_dir / name}")

selected_path = bundle_dir / "selected_pages.yaml"
try:
    import yaml
    selected_pages = yaml.safe_load(selected_path.read_text(encoding="utf-8"))
    selected_pages = selected_pages if isinstance(selected_pages, list) else []
except Exception as exc:
    selected_pages = []
    lines.append(f"- Selected pages unavailable: {exc}")

if selected_pages:
    lines.append("- Selected pages:")
    seen = set()
    shown = 0
    for page in selected_pages:
        if not isinstance(page, dict):
            continue
        path = page.get("path")
        if not path or path in seen:
            continue
        seen.add(path)
        title = page.get("title") or path
        scope = page.get("effective_scope") or page.get("scope") or "unknown"
        authority = page.get("authority_level") or page.get("authority") or "unknown"
        lines.append(f"  - {title} -> {path} ({scope}/{authority})")
        shown += 1
        if shown >= 12:
            remaining = len([p for p in selected_pages if isinstance(p, dict)]) - shown
            if remaining > 0:
                lines.append(f"  - ... {remaining} more selected page record(s)")
            break

warnings = result.get("warnings", [])
if warnings:
    lines.append("- Warnings:")
    for warning in warnings[:8]:
        lines.append(f"  - {warning}")

emit("\n".join(lines))
PY
