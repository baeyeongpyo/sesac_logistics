#!/usr/bin/env bash
set -euo pipefail

DEST="."
INSTALL_AGENTS=true
FORCE=false

usage() {
  cat <<'USAGE'
Usage: init-agent-os.sh [options]

Create or refresh Agent OS overlay files for an llm-wiki-core project.

Options:
  --dest PATH          Project root. Default: .
  --agents             Install thin AGENTS.md template when missing. Default.
                       Skips AGENTS.md creation if an existing lowercase agents.md is present.
  --no-agents          Do not install AGENTS.md.
  --force              Overwrite existing target files. Default is skip existing.
  -h, --help           Show help.

Examples:
  llm-wiki-core/scripts/init-agent-os.sh --dest ~/project
  llm-wiki-core/scripts/init-agent-os.sh --dest ~/project --force
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --dest)
      DEST="${2:?--dest requires PATH}"
      shift 2
      ;;
    --agents)
      INSTALL_AGENTS=true
      shift
      ;;
    --no-agents)
      INSTALL_AGENTS=false
      shift
      ;;
    --force)
      FORCE=true
      shift
      ;;
    --claude|--codex)
      echo "Note: The flags --claude and --codex have been moved to llm-wiki-core/scripts/init-ide-agents.sh." >&2
      echo "Please run: llm-wiki-core/scripts/init-ide-agents.sh $1 --dest $DEST" >&2
      exit 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

CORE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$(mkdir -p "$DEST" && cd "$DEST" && pwd)"

log() { printf '[agent-os-init] %s\n' "$*"; }
warn() { printf '[agent-os-init][warn] %s\n' "$*" >&2; }

copy_file() {
  local src="$1"
  local dst="$2"
  if [ ! -e "$src" ]; then
    warn "missing source template: $src"
    return 0
  fi
  mkdir -p "$(dirname "$dst")"
  if [ -e "$dst" ] && [ "$FORCE" != "true" ]; then
    log "skip existing: $dst"
    return 0
  fi
  cp "$src" "$dst"
  log "installed: $dst"
}

copy_agents_entrypoint() {
  if exact_file_exists "$DEST" "agents.md" && ! exact_file_exists "$DEST" "AGENTS.md"; then
    warn "existing agents.md detected; not creating AGENTS.md. Merge llm-wiki-core/templates/snippet-agent-os-agents.txt manually if this agent entrypoint should reference llm-wiki-core/templates/agent-os-agents.md"
    return 0
  fi
  copy_file "$CORE_ROOT/templates/snippet-agent-os-agents.txt" "$DEST/AGENTS.md"
}

exact_file_exists() {
  local dir="$1"
  local name="$2"
  [ -n "$(find "$dir" -maxdepth 1 -type f -name "$name" -print -quit 2>/dev/null)" ]
}

mkdir -p \
  "$DEST/.agent-os/roles" \
  "$DEST/.agent-os/tasks" \
  "$DEST/.agent-harness/bundles" \
  "$DEST/.agent-harness/pending-personal-captures"

copy_file "$CORE_ROOT/agent-os/README.md" "$DEST/.agent-os/README.md"
copy_file "$CORE_ROOT/agent-os/roles/default.md" "$DEST/.agent-os/roles/default.md"
copy_file "$CORE_ROOT/agent-os/tasks/llm-wiki-task.md" "$DEST/.agent-os/tasks/llm-wiki-task.md"
copy_file "$CORE_ROOT/templates/wiki-core-agents.md" "$DEST/llm-wiki-core/templates/wiki-core-agents.md"
copy_file "$CORE_ROOT/templates/agent-os-agents.md" "$DEST/llm-wiki-core/templates/agent-os-agents.md"
copy_file "$CORE_ROOT/templates/snippet-wiki-core-agents.txt" "$DEST/llm-wiki-core/templates/snippet-wiki-core-agents.txt"
copy_file "$CORE_ROOT/templates/snippet-agent-os-agents.txt" "$DEST/llm-wiki-core/templates/snippet-agent-os-agents.txt"

if [ "$INSTALL_AGENTS" = "true" ]; then
  copy_agents_entrypoint
fi

log "done"
