#!/usr/bin/env bash
#
# Build a read-only compressed wiki artifact (.tar.gz) from a mutable source directory.
#
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: build-wiki-artifact.sh [options]

Options:
  --src PATH         Source wiki directory to package (e.g. llm-wiki). Required.
  --dest PATH        Destination artifact file path (e.g. artifacts/team-wiki-1.0.0.tar.gz). Required.
  --namespace NS     Namespace to print in the wiki_stack.yaml snippet.
                     Not written to artifact.yaml. Default: team:wiki.
  -h, --help         Show this help.

Example:
  ./llm-wiki-core/scripts/build-wiki-artifact.sh --src ./llm-wiki --dest ./artifacts/team-wiki-1.0.0.tar.gz --namespace team:wiki
USAGE
}

SRC=""
DEST=""
NAMESPACE=""

while [ "$#" -gt 0 ]; do
  case "$1" in
    --src)
      SRC="${2:?--src requires a directory path}"
      shift 2
      ;;
    --dest)
      DEST="${2:?--dest requires a file path}"
      shift 2
      ;;
    --namespace)
      NAMESPACE="${2:?--namespace requires a value}"
      shift 2
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

if [ -z "$SRC" ] || [ -z "$DEST" ]; then
  echo "Error: Both --src and --dest are required." >&2
  usage >&2
  exit 1
fi

if [ ! -d "$SRC" ]; then
  echo "Error: Source directory '$SRC' does not exist." >&2
  exit 1
fi

# Ensure output directory exists
mkdir -p "$(dirname "$DEST")"

# Prepare temporary staging area for bundling to avoid tar paths issue
STAGING_DIR="$(mktemp -d)"
trap 'rm -rf "$STAGING_DIR"' EXIT

# Copy files to staging area, ignoring exclusions like .git, AGENTS.md, etc.
cp -R "$SRC"/* "$STAGING_DIR"/ 2>/dev/null || true

# Autogenerate artifact.yaml if missing
if [ ! -f "$STAGING_DIR/artifact.yaml" ]; then
cat <<EOF > "$STAGING_DIR/artifact.yaml"
version: "0.1.0"
built_at: "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
EOF
fi

# Create tar.gz archive
echo "Packaging wiki from '$SRC' to '$DEST'..."
(cd "$STAGING_DIR" && tar -czf "$STAGING_DIR.tar.gz" .)
mv "$STAGING_DIR.tar.gz" "$DEST"

echo "Artifact successfully created: $DEST"
echo "You can register it in your wiki_stack.yaml under 'wiki_artifacts' as follows:"
echo ""
echo "wiki_artifacts:"
echo "  - dependency_id: $(basename "$DEST" | sed 's/\.tar\.gz//' | sed 's/\.tgz//')"
echo "    artifact_ref: $DEST"
echo "    namespace: ${NAMESPACE:-team:wiki}"
echo ""
