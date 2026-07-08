---
name: llm-wiki-query
description: Use when querying compiled pages, searching wiki contents, or verifying page lineage and source integrity using the llm-wiki-core reference runtime.
version: 1.0.0
author: Antigravity Agent
license: MIT
metadata:
  hermes:
    tags: [llm-wiki, query, context-bundle, page-search, lineage, multi-agent, token-budget]
    related_skills: [agent-os-wiki-bundle, dispatching-parallel-agents]
---

# LLM Wiki Query

## Overview

에이전트가 로컬 또는 배포된 위키 지식을 조회할 때, 파일시스템을 무작위로 검색하는 대신 `llm-wiki-core` 쿼리 레이어를 사용하여 일관되고 검증된 지식만 조회해야 합니다. 
특히, 프로젝트 내 **Agent OS 오버레이가 활성화되어 있는지 여부**에 따라 쿼리 및 소비 방식이 달라집니다.

## When to Use

- 프로젝트 내 특정 아키텍처 규칙, 의사결정 기록(ADR) 또는 가이드를 검색할 때
- 현재 활성화된 지식의 출처(lineage) 및 신뢰도(confidence), 최신 여부(stale)를 확인할 때
- 작업 세션을 위해 여러 결합된 위키들의 통합 wiki-context snapshot인 Context Bundle을 활용할 때

## Required Flow

프로젝트 루트 내 `.agent-os/` 디렉터리 및 Agent OS 훅이 구성되어 있는지 확인한 후, 다음 두 흐름 중 하나를 선택합니다.

### 1. Agent OS가 활성화된 경우 (With Agent OS)
- **원칙**: 에이전트는 직접 위키 소스 파일을 무작위로 뒤지거나 직접 쿼리하지 않고, Agent OS가 세션 시작 시 구성한 **Context Bundle 스냅샷**을 우선적으로 소비해야 합니다. 이 bundle은 이전 대화 transcript가 아니라 selected wiki pages, warnings/conflicts, source binding, lineage를 담는 derived artifact입니다.
- **워크플로우**: `llm-wiki-core/hooks/pre-bundle-validate.sh`와 `llm-wiki-core/hooks/session-start.sh`는 `SessionStart` hook이 세션 시작 시 자동 실행하므로 다시 실행하지 않는다.
  1. 생성된 번들 파일인 `.agent-harness/bundles/<run-id>/context_bundle.md`를 읽어 필요한 위키 지식 정보를 한 번에 획득합니다.
  2. 번들 내 `conflict_warnings.yaml` 또는 `warnings.yaml`이 존재하는지 확인하여 충돌이나 만료 데이터 여부를 진단합니다.
  3. 이전 작업 상태가 필요하면 bundle이 아니라 handoff/task/capture 문서를 확인합니다.

### 2. Agent OS가 없는 경우 (Without Agent OS)
- **원칙**: Agent OS의 훅 흐름이 없을 때는 `llm-wiki-core` CLI 또는 쿼리 MCP 서버를 직접 사용하여 실시간으로 지식을 검색하고 조회합니다.
- **워크플로우**:
  1. `search-pages` 커맨드를 사용하여 키워드나 태그 기반으로 지식 페이지 목록을 검색합니다.
  2. 필요한 경우 `get-lineage` 커맨드를 실행하여 특정 페이지의 원본 출처 해시와 만료 여부를 검사합니다.
  3. 필요 시 `get-context-bundle` 커맨드를 호출해 임시 아웃풋 경로에 번들을 빌드하여 조회할 수 있습니다.

## Budgeted Multi-Agent Query Context Fill

이 섹션은 query 시 context 품질을 높이기 위한 선택적 multi-agent workflow를 정의합니다. 기본 원칙은 **single search first, gated multi-agent second**입니다. 단순 조회는 기존 단일 검색 흐름을 유지하고, 넓거나 애매한 질문에서만 token budget에 맞춰 병렬 query agent를 사용합니다.

### Budget modes

| Mode | Agent count | Use case | Behavior |
|---|---:|---|---|
| `low` | `0` | 좁은 lookup, token 최소화 요청 | 기존 `search-pages`/bundle 중심 단일 조회만 수행 |
| `normal` | 최초 `0`, gate 통과 시 `2`-`3` | 기본 추천값, architecture/implementation 질문 | 단일 검색 후 부족할 때만 role-specific query agent 실행 |
| `deep` | 최대 `4` | 사용자가 명시적으로 깊은 조사/교차검증을 요청 | query variants와 perspective agents를 함께 사용 가능 |

`deep`은 routine query에 자동 적용하지 않습니다. 사용자가 `deep`을 명시적으로 요청하지 않으면 실행하지 않습니다. 정확도 위험이 높아 비용을 사용할 필요가 있다면, 해당 비용을 감수하는지 사용자에게 설명하고 동의가 있어야만 실행할 수 있습니다.

### Gate conditions

초기 단일 검색 이후 아래 조건 중 하나 이상이 참이면 multi-agent query를 사용할 수 있습니다.

- broad question인데 plausible match가 `3`개 미만입니다.
- 상위 match가 한 페이지에 몰려 있지만 질문은 architecture, 구현 근거, lineage 등 여러 관심사를 포함합니다.
- `warnings.yaml`, `conflict_warnings.yaml`, `resolved_conflicts.yaml`, `review_needed`, `stale` 정보가 답변 신뢰도에 영향을 줄 수 있습니다.
- 사용자가 architecture-level 답변, trade-off 분석, 구현 가이드를 요청했습니다.
- 사용자가 명시적으로 deep query 또는 context fill을 요청했습니다.

해당 조건이 없으면 multi-agent를 실행하지 않고 기존 단일 query 결과만 사용합니다.

### Default roles for `normal`

- `architecture`: architecture rules, concepts, design docs, decisions를 찾습니다.
- `implementation_evidence`: 구현 파일, 명령, runtime behavior, tests 관련 근거를 찾습니다.
- `lineage_risk`: source lineage, stale/review flags, conflicts, warnings를 확인합니다.

### Extra role for `deep`

- `cross_check`: contradiction, duplicate title, rejected page, confidence gap을 찾습니다.

### Query agent contract

각 subagent는 하나의 role과 하나의 query intent만 받습니다. Subagent는 read-only로 동작해야 하며 아래 입력만 사용할 수 있습니다.

- 최신 `.agent-harness/bundles/<run-id>/context_bundle.md`
- 같은 bundle의 `selected_pages.yaml`, `warnings.yaml`, `conflict_warnings.yaml`, `resolved_conflicts.yaml`, `source_lineage.yaml`
- `llm-wiki-core/scripts/llm-wiki-core --root . search-pages "<query>"`
- `llm-wiki-core/scripts/llm-wiki-core --root . get-lineage <page>`
- `llm-wiki-core/scripts/llm-wiki-core --root . get-context-bundle`는 멀티에이전트 실행 전, 활성/현재 번들이 없거나 stale일 때 먼저 실행/요청해야 합니다. active/current bundle이 없거나 오래된 상태에서는 missing/stale 파일로 멀티 에이전트를 dispatch해서는 안 됩니다.

Subagent는 다음을 절대 하지 않습니다.

- `llm-wiki/raw/` 직접 조회
- 파일 수정
- ingest, promotion, capture artifact 생성
- 긴 page body 복사 반환
- 이전 세션 대화 전체를 bundle에서 복원하려고 시도

Return exactly this shape:

```text
ok: <role>
top_matches:
- path: <path>
  title: <title>
  reason: <one sentence>
  snippet: <short excerpt or paraphrase>
  confidence: <high|medium|low>
lineage_notes:
- <short note, or none>
warnings:
- <short warning, or none>
missing_context:
- <what was not found, or none>
```

On failure:

```text
failed: <role>: <reason>
```

### Orchestrator merge

Orchestrator가 최종 context assembly를 소유합니다.

1. 성공한 agent 결과를 파싱합니다.
2. `path` 기준으로 중복 page를 제거하고, 가장 높은 confidence reason을 보존하며 role labels를 병합합니다.
3. higher authority와 `accepted` status page를 우선하지만, 충돌하지 않는 advisory page는 보조 근거로 유지할 수 있습니다.
4. warning, conflict, stale flag, missing context를 답변에 사용하기 전에 먼저 노출합니다.
5. snippet이 부족해 full page가 필요하면 subagent마다 읽지 않고 orchestrator가 deduplicated page를 한 번만 읽습니다.
6. worker output 전체 transcript가 아니라 compact context set만 최종 답변에 사용합니다.

### Token controls

- 좁은 lookup은 `low`를 기본으로 사용합니다.
- architecture 또는 implementation 질문은 `normal`을 기본으로 사용하되 gate를 먼저 평가합니다.
- `normal`은 최대 `3` agents, `deep`은 최대 `4` agents입니다.
- 각 agent는 최대 `5` matches와 짧은 snippet만 반환합니다.
- page content 복사보다 path, metadata, short paraphrase를 선호합니다.
- full-page read는 orchestrator가 필요할 때만 중복 제거 후 한 번 수행합니다.

### Dispatch fallback

병렬 subagent dispatch가 가능하면 batch 안의 role agents를 한 번의 응답/턴에서 동시에 실행합니다. 병렬 dispatch가 불가능하면 동일한 role contract를 deterministic order(`architecture`, `implementation_evidence`, `lineage_risk`, `cross_check`)로 순차 실행합니다. 한 role이 실패해도 성공한 role 결과로 계속 진행하고 실패 role을 최종 답변에 보고합니다.

## Commands

```bash
# 1. 키워드로 위키 페이지 검색
llm-wiki-core/scripts/llm-wiki-core --root . search-pages "<검색어>"

# 2. 특정 페이지의 출처 메타데이터(Lineage) 및 신뢰도 정보 확인
llm-wiki-core/scripts/llm-wiki-core --root . get-lineage llm-wiki/concepts/example.md

# 3. 전체 위키 컴파일 정보 스냅샷(Context Bundle) 확인
llm-wiki-core/scripts/llm-wiki-core --root . get-context-bundle
```

기본 generated bundle retention은 최신 `run-YYYYMMDD-HHMMSS` 10개 유지,
최소 3개 보호다. 명시적 `--output` 경로는 호출자가 관리한다.

## Common Pitfalls

1. **직접 탐색**: `llm-wiki` 하위 폴더나 배포된 아티팩트 내 파일을 임의로 직접 파싱하여 캐싱하는 것은 버그나 검증 오류를 유발할 수 있습니다. 쿼리 인터페이스 또는 빌드된 `context_bundle.md`를 사용해야 합니다.
2. **경고 무시**: `review_needed: true` 또는 `stale: true`로 표시된 문서를 핵심 의사결정의 근거(Canonical Truth)로 맹신하고 적용하는 것은 금지됩니다.
3. **대화 복원 오해**: Context Bundle은 이전 세션 대화 기록이 아닙니다. 작업 재개 상태는 handoff/task/capture 문서에서 확인해야 합니다.

## Verification Checklist

- [ ] 검색 전 `validate` 명령어로 스택 설정에 문법적 오류가 없는지 확인하였는가?
- [ ] 참조한 위키 지식의 `confidence`와 `stale` 상태를 체크했는가?
- [ ] 소스의 `lineage` 메타데이터가 보존되어 있는가?
- [ ] `deep`은 명시적 요청 또는 비용 동의 후에만 활성화되었고, active/current bundle이 없거나 stale일 때는 `get-context-bundle`으로 갱신 후 dispatch했는지 기록했는가?
