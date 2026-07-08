---
name: agent-os-role-generator
description: Use to create, provision, and validate custom Agent OS roles within the .agent-os/roles/ directory to guide agent behavior.
version: 1.0.0
author: Antigravity
license: MIT
metadata:
  tags: [agent-os, role-provision, tools, configuration]
  related_skills: [agent-os-wiki-bundle]
---

# Agent OS Role Generator

## Overview

Agent OS는 wiki truth store가 아닌 task/role lens입니다.
특정 목적을 가진 에이전트의 동작과 훅 실행 규칙을 제어하기 위해 `.agent-os/roles/<role-name>.md` 포맷으로 역할을 정의하고 프로비저닝하는 스킬입니다.

## When to Use

- 새로운 에이전트 역할(예: `software-engineer.md`, `tech-writer.md`, `qa-engineer.md`)을 설계하고 배치할 때.
- 에이전트 세션의 사전/사후 훅 동작 규칙을 역할별로 세분화할 때.
- 프로젝트 루트의 에이전트 진입점(`AGENTS.md`)에 새로운 역할을 명시할 때.

## Required Role Schema (`.agent-os/roles/<role-name>.md`)

모든 Agent OS 역할 파일은 아래의 표준 마크다운 헤더 구조를 준수해야 합니다.

```markdown
# [Role Name] Agent OS Role

## Role

[이 역할이 수행하는 핵심 책임과 에이전트가 준수해야 할 정체성에 대해 명세합니다.]

## Before Work

[작업을 시작하기 전 확인해야 할 번들 검증 작업]
`llm-wiki-core/hooks/pre-bundle-validate.sh`와 `llm-wiki-core/hooks/session-start.sh`는 `SessionStart` hook이 자동 실행하므로 다시 실행하지 않는다.
1. 생성된 번들과 `warnings.yaml`의 충돌 사항 및 검토 결과를 확인.
2. bundle은 이전 대화 transcript가 아니라 wiki-context snapshot이므로, 이전 작업 상태가 필요하면 handoff/task/capture 문서를 확인.

## During Work

[작업 수행 중 에이전트가 위키 및 아키텍처 규칙을 준수하는 방법 가이드]
- Canonical Truth와 Personal/Reference 내용이 충돌할 경우 조용히 덮어쓰지 않고 curator 피드백 획득.
- Project/Team Wiki의 소스 파일을 직접 편집하는 것을 금지하며, 컨텍스트 번들과 lineage를 근거로 사용.

## After Work

[작업 완료 후 산출물 기록 및 승격 절차]
- 에이전트 작업 실행 중 획득한 재사용 가치 있는 지식이나 교훈은 `llm-wiki-core/hooks/post-run-capture.sh`를 통해 명시적으로 Personal Capture 후보로 기록.
- Canonical Truth에 반영해야 하는 변경사항은 Promotion Package(`.yaml`)로 작성하여 `llm-wiki-core/hooks/promotion-submit-validate.sh` 검증 후 승격 절차 진행.
```

## Creating a Role

이 스킬은 멱등적으로 역할을 생성할 수 있는 자동화 스크립트 `scripts/create-role.sh`를 제공합니다.

### Automated Way (Script)

이 스킬의 helper 스크립트를 사용해 새 역할을 생성합니다:

```bash
# Usage:
# llm-wiki-core/skills/software-development/agent-os-role-generator/scripts/create-role.sh [options] <role-name> [description]
#
# Options:
#   --force: 기존에 존재하는 역할 파일을 강제로 덮어씁니다.
#
# Example:
llm-wiki-core/skills/software-development/agent-os-role-generator/scripts/create-role.sh "qa-engineer" "Responsible for validating and testing all wiki features and runtime configurations."
```

### Manual Way

1. `.agent-os/roles/` 디렉토리가 존재하는지 확인합니다. (`mkdir -p .agent-os/roles`)
2. `.agent-os/roles/<role-name>.md` 파일을 생성합니다.
3. 표준 Schema를 기반으로 역할 책임, Before, During, After Work를 작성합니다.

## Verification Checklist

- [ ] `.agent-os/roles/<role-name>.md` 파일이 존재하고 적절한 마크다운 문법을 만족함.
- [ ] 파일의 제목 헤더가 `# [Role Name] Agent OS Role` 형식임.
- [ ] `## Role`, `## Before Work`, `## During Work`, `## After Work` 섹션이 누락 없이 작성됨.
- [ ] 역할 정의 내에 금지된 stale 키워드(`trust_level`, `binding_id` 등)가 남발되지 않았는지 검토 완료.
