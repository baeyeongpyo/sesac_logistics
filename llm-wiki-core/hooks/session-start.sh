#!/usr/bin/env bash
set -euo pipefail
ROOT="${1:-.}"
OUT="${2:-}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BIN="$SCRIPT_DIR/../scripts/llm-wiki-core"

if [ -n "$OUT" ]; then
  RESULT=$("$BIN" --root "$ROOT" get-context-bundle --output "$OUT" 2>&1 || true)
else
  RESULT=$("$BIN" --root "$ROOT" get-context-bundle 2>&1 || true)
fi

python3 - "$RESULT" <<'PY' || true
import json, sys
from pathlib import Path

raw = sys.argv[1]
try:
    result = json.loads(raw)
except json.JSONDecodeError:
    print("Bundle generation: COULD NOT RUN")
    print("Cause: the get-context-bundle command did not produce valid JSON output. Raw output:")
    for line in (raw.splitlines() or ["(empty output)"]):
        print(f"  {line}")
    print("Fix: run 'llm-wiki-core/scripts/llm-wiki-core --root . get-context-bundle' directly to see the full error,")
    print("then check llm-wiki-core/skills/research/llm-wiki-core-environment-setup/SKILL.md for the setup workflow.")
    sys.exit(0)
print(f"Bundle ready: {result.get('output_dir')}")

output_dir = result.get("output_dir")
if output_dir:
    bundle_dir = Path(output_dir)
    print("Wiki context injection summary:")
    print("  Instruction: Before answering project questions, inspect relevant selected wiki pages.")
    print(f"  Bundle: {bundle_dir}")
    print("  Required context files:")
    for name in ("context_bundle.md", "warnings.yaml", "selected_pages.yaml", "source_lineage.yaml"):
        print(f"    - {bundle_dir / name}")
    try:
        import yaml
        selected_path = bundle_dir / "selected_pages.yaml"
        selected_pages = yaml.safe_load(selected_path.read_text(encoding="utf-8"))
        selected_pages = selected_pages if isinstance(selected_pages, list) else []
    except Exception as e:
        selected_pages = []
        print(f"  Selected pages: unavailable ({e})")
    if selected_pages:
        print("  Selected pages:")
        seen_paths = set()
        shown = 0
        for page in selected_pages:
            if not isinstance(page, dict):
                continue
            path = page.get("path")
            if not path or path in seen_paths:
                continue
            seen_paths.add(path)
            title = page.get("title") or path
            scope = page.get("effective_scope") or page.get("scope") or "unknown"
            authority = page.get("authority_level") or page.get("authority") or "unknown"
            print(f"    - {title} -> {path} ({scope}/{authority})")
            shown += 1
            if shown >= 12:
                remaining = max(0, len(selected_pages) - shown)
                if remaining:
                    print(f"    - ... {remaining} more selected page record(s)")
                break
warnings_list = result.get("warnings", [])
for w in warnings_list:
    if w.startswith("orphaned_source:") or w.startswith("conflict_auto_resolved:"):
        continue
    print(f"  warning: {w}")
orphaned = [w for w in warnings_list if w.startswith("orphaned_source:")]
if orphaned:
    print(f"Fix: {len(orphaned)} item(s) may be from an interrupted ingest run "
          "(source file exists, log.md entry missing). Follow the 'Interrupted "
          "Run Recovery' procedure in "
          "llm-wiki-core/skills/software-development/llm-wiki-ingest/SKILL.md - ask the user "
          "whether to resume before writing anything.")
resolved_conflicts = result.get("resolved_conflicts", [])
if resolved_conflicts:
    for r in resolved_conflicts:
        print(f"  resolved_conflict: {r.get('title')} -> chose {r.get('chosen')} "
              f"over {', '.join(r.get('rejected', []))} (rule: {r.get('rule')})")
        if r.get("note"):
            print(f"    note: {r['note']}")
    print(f"Fix: {len(resolved_conflicts)} conflict(s) were auto-resolved by priority "
          "(no action required to keep working). Review each by running:")
    for r in resolved_conflicts:
        print(f"  {r.get('command')}")
    print("If priority picked the wrong page, record an override with:")
    print('  llm-wiki-core/scripts/llm-wiki-core --root . resolve-conflict "<title>" --choose <path>')
conflicts = result.get("conflicts", [])
if conflicts:
    for c in conflicts:
        print(f"  conflict: {c.get('type')} title={c.get('title')}")
    print("Fix: these conflicts have equal source priority, so they could not be "
          "auto-resolved. Review conflict_warnings.yaml in the bundle output dir "
          "above and decide with the user/curator, or use show-conflict / "
          "resolve-conflict.")
PY

exit 0
