# llm-wiki-core .gitignore 가이드

이 문서는 `llm-wiki-core`를 사용하는 프로젝트에서 한 번에 적용할 `.gitignore` 기준을 정리한다.

## 핵심 원칙

`.gitignore`는 llm-wiki만 쓰는 단계와 Agent OS를 쓰는 단계를 나누지 않는다. 처음부터 통합 블록을 적용한다.

이유:

```text
처음에는 llm-wiki만 사용할 수 있다.
나중에 Agent OS를 적용하면 .agent-os/, AGENTS.md, hook 설정이 추가된다.
이때 .gitignore를 다시 설계하지 않아도 되어야 한다.
```

따라서 기본 정책은 다음과 같다.

```text
ignore한다:
  llm-wiki/
  .agent-harness/의 재생성 가능한 실행 산출물
  local cache/tmp/log
  runtime noise

ignore하지 않는다:
  wiki_stack.yaml
  llm-wiki-core/
  AGENTS.md
  .agent-os/
  llm-wiki-promotion-queue/
  공유하기로 결정한 artifacts/
  팀 공통 agent hook 설정
```

## 바로 붙여넣을 통합 설정

아래 블록을 프로젝트 `.gitignore`에 한 번에 적용한다. Agent OS를 아직 쓰지 않아도 그대로 넣는다.

```gitignore
# llm-wiki-core integrated project ignore
# Apply this once even if Agent OS is added later.

# local mutable wiki
# Project/Team artifact로 승격하기 전의 개인/프로젝트 작업 영역이다.
llm-wiki/

# generated llm-wiki-core runtime outputs
# context bundle은 현재 wiki 상태에서 재생성되는 산출물이다.
.agent-harness/bundles/
.agent-harness/pending-personal-captures/
.agent-harness/cache/
.agent-harness/tmp/
.agent-harness/*.log

# runtime noise
__pycache__/
*.py[cod]
.pytest_cache/
.DS_Store
Thumbs.db

# keep shared llm-wiki-core configuration tracked
!wiki_stack.yaml
!llm-wiki-core/
!llm-wiki-core/**
!AGENTS.md
!.agent-os/
!.agent-os/**
!llm-wiki-promotion-queue/
!llm-wiki-promotion-queue/**

# keep shared agent hook configs tracked when used
!.codex/
!.codex/hooks.json
!.claude/
!.claude/settings.json
!.agents/
!.agents/hooks.json
```

## 현재 프로젝트 구조 기준

`init-llm-wiki.sh`를 적용하면 다음 구조가 생긴다.

```text
wiki_stack.yaml
llm-wiki/
llm-wiki-core/
  hooks/
  scripts/
  templates/
  agent-os/
  skills/
```

Git 기준:

```text
track:
  wiki_stack.yaml
  llm-wiki-core/

ignore:
  llm-wiki/
```

`init-agent-os.sh`를 나중에 적용하면 다음 구조가 추가된다.

```text
AGENTS.md
.agent-os/
  README.md
  roles/default.md
  tasks/llm-wiki-task.md
.agent-harness/
  bundles/
  pending-personal-captures/
```

위 통합 `.gitignore`를 이미 적용해두면 추가 작업 없이 다음 기준이 유지된다.

```text
track:
  AGENTS.md
  .agent-os/

ignore:
  .agent-harness/bundles/
  .agent-harness/pending-personal-captures/
  .agent-harness/cache/
  .agent-harness/tmp/
  .agent-harness/*.log
```

## 경로별 판단 기준

`llm-wiki/`:

```text
현재 프로젝트의 local mutable wiki다.
Project/Team artifact로 승격하기 전의 작업 영역이므로 일반 프로젝트 repo에서는 Git에 올리지 않는다.
```

`wiki_stack.yaml`:

```text
프로젝트가 어떤 artifact를 연결하는지 나타내는 dependency 선언이다.
팀원이 같은 설정을 사용해야 하므로 Git에 올린다.
```

`llm-wiki-core/`:

```text
runtime, CLI, hooks, templates, docs, skills를 담는 실행 자산이다.
현재 방식처럼 프로젝트에 vendoring해서 쓰는 경우 Git에 올린다.
```

`AGENTS.md`:

```text
agent entrypoint다.
Agent OS를 나중에 적용하더라도 공유되어야 하므로 ignore하지 않는다.
```

`.agent-os/`:

```text
Agent OS role/task 운영 규칙이다.
캐시가 아니므로 Git에 올린다.
```

`.agent-harness/`:

```text
bundle, cache, tmp, log는 실행 중 재생성되는 산출물이다.
Git에 올리지 않는다.
```

`llm-wiki-promotion-queue/`:

```text
target-free promotion package를 다른 repo나 별도 promotion processor로 넘기기 위한 queue다.
submit 결과를 팀 리뷰 대상으로 남길 수 있어야 하므로 기본적으로 Git에 올린다.
```

## 에이전트별 hook 설정

에이전트별 hook 설정 파일은 팀 공통으로 같은 hook을 쓰면 Git에 올린다.

```text
.codex/hooks.json
.claude/settings.json
.agents/hooks.json
```

이 파일들이 `llm-wiki-core/hooks/*.sh`를 호출하는 프로젝트 설정이면 공유 대상이다.

개인 토큰, 개인 로컬 경로, 비공개 모델 설정이 섞인다면 같은 파일에 넣지 말고 개인 전용 local 설정으로 분리한다. 프로젝트에 커밋하는 설정에는 secret을 넣지 않는다.

## artifacts/ 운영 방식별 선택

`artifacts/`는 프로젝트 운영 방식에 따라 결정한다.

```text
repo가 artifact snapshot을 소유한다:
  artifacts/를 Git에 올린다.

repo가 artifact를 외부 cache나 별도 repo에서 받는다:
  artifacts/를 ignore한다.
```

artifact를 현재 repo에 vendoring하지 않고 외부 cache나 별도 repo에서만 관리한다면 통합 블록 아래에 이 optional 블록을 추가한다.

```gitignore
# Optional: artifact cache is managed outside this repo.
# 이 블록을 켜면 artifacts/ 아래 artifact snapshot은 Git에 올라가지 않는다.
artifacts/*

# But keep a placeholder or local README if the project uses one.
!artifacts/.gitkeep
!artifacts/README.md
```

반대로 `artifacts/`를 이 repo에서 공유 artifact snapshot으로 관리한다면 optional 블록은 넣지 않는다.

## local-only queue가 필요할 때

기본 `llm-wiki-promotion-queue/`는 공유 queue다. local 실험 queue가 필요하면 이름을 분리한다.

```gitignore
llm-wiki-promotion-queue-local/
```
