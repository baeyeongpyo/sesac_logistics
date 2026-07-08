#!/usr/bin/env bash
set -euo pipefail

FORCE=false
ROLE_NAME=""
ROLE_DESC=""

usage() {
  cat <<'USAGE'
Usage: create-role.sh [options] <role-name> [description]

Create a new Agent OS role template under .agent-os/roles/.

Options:
  --force           Overwrite the existing role file if it already exists.
  -h, --help        Show this help message.

Examples:
  ./create-role.sh qa-engineer "Validates core behaviors"
  ./create-role.sh --force tech-writer "Writes high quality docs"
USAGE
}

# Parse options
while [ "$#" -gt 0 ]; do
  case "$1" in
    --force)
      FORCE=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      if [ -z "$ROLE_NAME" ]; then
        ROLE_NAME="$1"
      elif [ -z "$ROLE_DESC" ]; then
        ROLE_DESC="$1"
      else
        echo "Too many arguments: $1" >&2
        usage >&2
        exit 2
      fi
      shift
      ;;
  esac
done

if [ -z "$ROLE_NAME" ]; then
  echo "Error: <role-name> is required." >&2
  usage >&2
  exit 2
fi

# Ensure role name is kebab-case or plain string suited for filename
# Replace spaces with hyphens, lowercase
ROLE_FILENAME=$(echo "$ROLE_NAME" | tr '[:upper:]' '[:lower:]' | tr ' ' '-')
TARGET_DIR=".agent-os/roles"
TARGET_FILE="$TARGET_DIR/$ROLE_FILENAME.md"

# Format display name (e.g. qa-engineer -> Qa Engineer)
DISPLAY_NAME=$(echo "$ROLE_FILENAME" | awk 'BEGIN{FS=OFS="-"} {for(i=1;i<=NF;i++) $i=toupper(substr($i,1,1))tolower(substr($i,2))} 1' | tr '-' ' ')

mkdir -p "$TARGET_DIR"

if [ -f "$TARGET_FILE" ] && [ "$FORCE" != "true" ]; then
  echo "Error: Role file '$TARGET_FILE' already exists. Use --force to overwrite." >&2
  exit 1
fi

ROLE_DESC_CONTENT=""
if [ -n "$ROLE_DESC" ]; then
  ROLE_DESC_CONTENT="$ROLE_DESC"
else
  ROLE_DESC_CONTENT="[이 역할이 수행하는 핵심 책임과 에이전트가 준수해야 할 정체성에 대해 명세합니다.]"
fi

cat <<EOF > "$TARGET_FILE"
# $DISPLAY_NAME Agent OS Role

## Role

$ROLE_DESC_CONTENT

## Before Work

[작업을 시작하기 전 확인해야 할 번들 검증 작업]
\`llm-wiki-core/hooks/pre-bundle-validate.sh\`와 \`llm-wiki-core/hooks/session-start.sh\`는 \`SessionStart\` hook이 자동 실행하므로 다시 실행하지 않는다.
1. 생성된 번들과 \`warnings.yaml\`의 충돌 사항 및 검토 결과를 확인.
2. bundle은 이전 대화 transcript가 아니라 wiki-context snapshot이므로, 이전 작업 상태가 필요하면 handoff/task/capture 문서를 확인.

## During Work

[작업 수행 중 에이전트가 위키 및 아키텍처 규칙을 준수하는 방법 가이드]
- Canonical Truth와 Personal/Reference 내용이 충돌할 경우 조용히 덮어쓰지 않고 curator 피드백 획득.
- Project/Team Wiki의 소스 파일을 직접 편집하는 것을 금지하며, 컨텍스트 번들과 lineage를 근거로 사용.

## After Work

[작업 완료 후 산출물 기록 및 승격 절차]
- 에이전트 작업 실행 중 획득한 재사용 가치 있는 지식이나 교훈은 \`llm-wiki-core/hooks/post-run-capture.sh\`를 통해 명시적으로 Personal Capture 후보로 기록.
- Canonical Truth에 반영해야 하는 변경사항은 Promotion Package(\`.yaml\`)로 작성하여 \`llm-wiki-core/hooks/promotion-submit-validate.sh\` 검증 후 승격 절차 진행.
EOF

echo "Successfully created role lens at: $TARGET_FILE"
