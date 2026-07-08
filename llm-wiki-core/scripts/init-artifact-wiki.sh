#!/usr/bin/env bash
set -euo pipefail

DEST=""
VERSION="0.1.0"
TITLE=""
FORCE=false

usage() {
  cat <<'USAGE'
Usage: init-artifact-wiki.sh [options]

Create an empty folder artifact wiki.

Options:
  --dest PATH          Artifact wiki directory to create. Required.
  --version VERSION    Artifact version. Default: 0.1.0
  --title TITLE        Human-readable wiki title. Default: derived from directory name.
  --force              Overwrite existing generated files. Default is preserve.
  -h, --help           Show help.

Example:
  llm-wiki-core/scripts/init-artifact-wiki.sh \
    --dest ../wiki-artifacts/new-team-wiki \
    --version 1.0.0 \
    --title "New Team Wiki"
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --dest)
      DEST="${2:?--dest requires PATH}"
      shift 2
      ;;
    --version)
      VERSION="${2:?--version requires a value}"
      shift 2
      ;;
    --title)
      TITLE="${2:?--title requires a value}"
      shift 2
      ;;
    --force)
      FORCE=true
      shift
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

if [ -z "$DEST" ]; then
  echo "Error: --dest is required." >&2
  usage >&2
  exit 1
fi

DEST="$(mkdir -p "$DEST" && cd "$DEST" && pwd)"
ARTIFACT_ID="$(basename "$DEST")"

if [ -z "$TITLE" ]; then
  TITLE="$(printf '%s' "$ARTIFACT_ID" | tr '_-' '  ')"
fi

write_file() {
  local path="$1"
  local content="$2"
  if [ -e "$path" ] && [ "$FORCE" != "true" ]; then
    return 0
  fi
  printf '%s\n' "$content" > "$path"
}

mkdir -p \
  "$DEST/raw" \
  "$DEST/sources" \
  "$DEST/concepts" \
  "$DEST/decisions" \
  "$DEST/comparisons" \
  "$DEST/queries" \
  "$DEST/metadata"

write_file "$DEST/artifact.yaml" "version: \"$VERSION\"
created_at: \"$(date -u +%Y-%m-%dT%H:%M:%SZ)\""

write_file "$DEST/index.md" "# $TITLE

> Folder artifact wiki. Add durable pages to this catalog when the artifact is published.

## Sources

## Concepts

## Decisions

## Comparisons

## Queries"

write_file "$DEST/log.md" "# Wiki Log

> Chronological record of artifact wiki changes. Append-only.
> Format: ## [YYYY-MM-DD] action | subject

## [$(date +%Y-%m-%d)] create | Artifact wiki scaffold initialized
- Version: $VERSION"

echo "created artifact wiki: $DEST"
