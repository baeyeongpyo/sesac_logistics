---
name: llm-wiki-ingest
description: Use when proposing, requesting, or running raw data ingestion into personal or local wiki stores while respecting Team/Project raw isolation and curator-approval boundaries.
version: 1.0.0
author: Antigravity Agent
license: MIT
metadata:
  hermes:
    tags: [llm-wiki, ingest, sync-point, permission-boundary, raw-isolation, multi-agent, parallel-dispatch]
    related_skills: [llm-wiki-core-environment-setup, agent-os-wiki-bundle, dispatching-parallel-agents]
---

# LLM Wiki Ingest

## Overview

Team/Project Wiki는 공유 지식 저장소이므로 외부/개인의 raw 데이터가 무단으로 유입되거나 오염되어서는 안 됩니다. 원본 raw 데이터는 외부 **Source Vault**에 격리하며, Wiki에는 가공된(curated/compiled) 지식과 `lineage` 메타데이터만 포함합니다.

이 스킬은 외부/개인 원본 데이터를 동기화하고 Ingest 요청을 제출하는 안전한 워크플로우를 정의합니다.

현재 프로젝트의 직접 편집 가능한 wiki는 `local-mutable-wiki` binding으로 읽힙니다. 프로젝트 루트 `wiki_stack.yaml`은 `personal_wikis: []`와 `source_binding_id: local-mutable-wiki`를 사용하며, `llm-wiki/raw/`와 `llm-wiki/sources/`에 쓰는 작업은 Project/Team artifact 승격이 아니라 local wiki ingest입니다.

`llm-wiki/raw/`는 원문 계층으로 유지한다. 대형/복합 raw item은 원문을 직접
worker에게 넘기기 전에 `raw-derived` 계획을 먼저 만든다. `raw-derived`는
manifest/divider/inventory/prepared unit/lineage 같은 ingest metadata이며,
정제된 wiki 지식이 아니고 `sources:` frontmatter에 직접 citation으로 넣지
않는다.

## When to Use

- 외부 원본 문서, 데이터시트, 로그 파일, 타사 기술 스펙 등을 로컬/개인 위키에 Ingest하려고 할 때
- Project 또는 Team Wiki에 Ingest할 수 있는 **Sync Point**를 검색하거나 요청서를 작성할 때
- Ingest 승인 게이트(Approval Gate) 절차를 거치고자 할 때

## Required Flow

1. **Source Visibility & Selection**:
   - Ingest 하려는 대상 파일들이 Ingest 허용 범위(vault, allowlist)에 있는지 확인합니다.
   - 대용량 바이너리, 비밀번호(secrets), 미검증 원본 덤프를 직접 프로젝트/팀 레포에 업로드하지 마십시오.

2. **Create Ingest Request**:
   - 직접 파일 병합을 하지 않고, `wiki_create_ingest_request` 또는 수동 등록 방식을 사용하여 Ingest 요청을 생성합니다.
   - 요청 시 소스 URI, Checksum/Version ID, Ingest 대상 범위 등을 정의하여 큐(`pending_review` 상태)에 추가합니다.

3. **Curator Approval & Ingest Run**:
   - 큐에 제출된 Sync Point는 큐레이터(Curator) 또는 관리자가 승인(`ready_for_ingest`)해야 Ingest Worker에 의해 실제 컴파일(Curated Page 생성)이 이루어집니다.

## Parallel Multi-Agent Ingest Run

이 섹션은 위 "Curator Approval & Ingest Run" 단계의 실행 방법을 정의합니다. 이 워크플로우를 직접 실행하는 것 자체가 승인(curator approval)으로 간주됩니다.

### 1. Discover pending raw items

먼저 raw item을 분류한다.

```bash
llm-wiki-core/scripts/llm-wiki-core --root . list-raw-items
```

작고 독립적인 raw 파일은 기존 방식 그대로 처리한다. 예:
`llm-wiki/raw/robot_spec.md` -> `llm-wiki/sources/robot_spec_summary.md`.

대형/복합 raw 디렉터리는 raw-derived 계획을 확인한다.

```bash
llm-wiki-core/scripts/llm-wiki-core --root . get-raw-derived-manifest ros2_ws
```

이 경우 pending 단위는 top-level raw item 하나가 아니라 divider의 prepared
unit이다. worker는 prepared unit을 읽고, 필요한 직접 원문 파일만 열어 source
summary를 작성한다.

### 2. Batch

대상 목록을 4~5개씩 배치로 나눕니다.

### Prepared-unit dispatch for complex raw items

복합 raw item은 divider의 unit별로 subagent를 보낸다. 병렬 실행 가능 시 서로
다른 output file을 소유하는 unit들을 한 번에 dispatch한다.

Subagent prompt contract:

```
Refine exactly one raw-derived prepared unit into a wiki source summary.

Raw item: llm-wiki/raw/<raw-id>
Prepared unit: llm-wiki/raw-derived/<raw-id>/prepared/<unit-id>/index.yaml
Output file: llm-wiki/sources/<output-name>.md

1. Read the prepared unit index first.
2. Open only the direct raw source files listed in that prepared unit unless
   more evidence is necessary.
3. Write the source page using the existing source frontmatter convention.
4. In `sources:`, list direct `llm-wiki/raw/...` evidence paths only. Do not
   list `raw-derived` files in `sources:`.
5. Do NOT write shared files (`llm-wiki/log.md`, raw-derived lineage, registry,
   manifest, divider). Return the log entry and lineage fragment to the
   orchestrator.
6. Do NOT touch `concepts/`, `decisions/`, `comparisons/`, or Project/Team
   wiki artifacts.

Return:
- On success: `ok: <unit-id>`, then the log entry, then a YAML lineage fragment.
- On failure: `failed: <unit-id>: <reason>`.
```

The orchestrator writes `llm-wiki/log.md` and raw-derived lineage in stable
divider order after collecting worker results.

### 3. Detect dispatch capability

병렬 subagent 디스패치가 가능한지 확인합니다.

- Claude Code: 항상 가능 (`Agent` 도구).
- Codex: `~/.codex/config.toml`에 `[features] multi_agent = true`가 설정된 경우에만 가능 (`spawn_agent`/`wait_agent`/`close_agent`).
- Antigravity: 항상 가능 (`invoke_subagent`).

가능하면 4단계를 배치 내 모든 항목에 대해 한 번의 응답/턴에서 동시에 디스패치합니다. 불가능하면 같은 4단계 계약을 항목별로 하나씩 순차 실행합니다 (출력/파일 계약은 동일, 동시성만 다릅니다).

### 4. Dispatch one subagent per small raw item

배치 안의 각 작고 독립적인 raw 항목마다 아래 prompt로 subagent를 디스패치합니다 (병렬 가능 시 한 번의 응답/턴에서 동시에; `superpowers:dispatching-parallel-agents` 참고). 대형/복합 raw item은 위 prepared-unit 계약을 사용합니다.

```
Refine exactly one raw item into a wiki source summary.

Raw item: llm-wiki/raw/<name>
Output file: llm-wiki/sources/<name>_summary.md

1. Read the raw item.
2. Write llm-wiki/sources/<name>_summary.md following the existing frontmatter
   convention used by other files in llm-wiki/sources/ (title, created,
   updated, type: source, status: accepted, tags, sources: [llm-wiki/raw/<name>]).
3. Do NOT write to llm-wiki/log.md yourself. Other agents in this batch may be
   running at the same time and concurrent edits to that one file can corrupt
   it or silently drop entries. Instead, compose the log entry (see Return
   format below) and return it - the orchestrator appends it after every
   agent in this batch has finished.
4. Do NOT touch llm-wiki/concepts/, llm-wiki/decisions/, llm-wiki/comparisons/,
   or any Project/Team wiki artifact.
5. Do NOT copy large binaries, secrets, or unreviewed raw dumps verbatim into
   the summary - summarize/extract, per this skill's Common Pitfalls.

Return exactly this, nothing else:
- On success: "ok: <name>" on the first line, then the exact llm-wiki/log.md
  entry to append on the following lines, in its existing format: a
  "## [YYYY-MM-DD] ingest | raw/<name>" header line, then a "- Source: ..."
  line, then a "- Details: ..." line.
- On failure: one line - "failed: <name>: <reason>".
```

### 5. Collect results (orchestrator writes log.md, never the subagent)

언제 쓰는지는 디스패치 모드에 따라 다릅니다 — 둘 다 최종적으로 discovery
순서가 되도록 보장하되, 순차 모드는 굳이 배치 전체를 기다릴 필요가 없습니다.

- **병렬 디스패치 모드**: 배치 안의 모든 subagent가 결과를 반환할 때까지
  기다립니다. 모두 받으면, **orchestrator 자신이** 성공한 항목들의 log
  entry를 discovery 순서(완료된 순서가 아니라 1단계에서 나열된 순서)대로
  하나씩 `llm-wiki/log.md`에 추가합니다. 완료 순서가 뒤섞여 도착하므로,
  전부 모은 뒤 정렬해서 써야만 discovery 순서가 보장됩니다.
- **순차 fallback 모드**: 3번에서 이미 항목을 discovery 순서대로 하나씩
  처리하므로, 배치 전체를 기다릴 필요가 없습니다. **orchestrator는 각
  subagent가 끝나는 즉시 그 자리에서 바로 해당 항목의 log entry를
  `llm-wiki/log.md`에 추가**한 뒤 다음 항목으로 넘어갑니다. 처리 순서
  자체가 discovery 순서와 같으므로 결과는 병렬 모드와 동일합니다.

두 모드 모두, subagent는 이 파일을 절대 직접 쓰지 않으므로 동시 쓰기 자체가
발생할 수 없습니다. 어떤 성공 응답에서 log entry 텍스트(`## [YYYY-MM-DD]
ingest | raw/<name>` 헤더 줄과 Source/Details 줄)를 알아볼 수 없으면, 그
항목은 source 파일은 이미 정상적으로 작성되었더라도 "log entry parse
failed: <name>"으로 최종 요약에 남깁니다.

### 6. Terminate / free the slot

- Claude Code: 추가 동작 불필요 (foreground 호출이 반환되면 이미 종료된 것입니다).
- Codex: 각 `spawn_agent`에 대해 `wait_agent`로 결과를 받은 뒤 `close_agent`를 호출해 명시적으로 종료합니다.
- Antigravity: `invoke_subagent` 반환으로 종료됩니다. `manage_subagents`에 아직 남아 있으면 kill합니다.

### 7. Handle per-item failure

항목 하나가 실패해도 그 항목만 실패로 기록하고 배치를 계속 진행합니다. 실패는 `log.md`에 쓰지 않고 (이 파일은 완료된 ingest만 기록하는 기존 컨벤션을 따릅니다) 최종 요약에만 보고합니다.

### 8. Repeat and summarize

모든 배치가 끝날 때까지 2~7단계를 반복한 뒤, 전체 성공/실패 개수와 실패 이유를 요약합니다.

### Platform tool mapping

| Action | Claude Code | Codex | Antigravity |
|---|---|---|---|
| Dispatch subagent | `Agent` | `spawn_agent` | `invoke_subagent` |
| Parallel | 한 응답에 여러 `Agent` 호출 | 한 응답에 여러 `spawn_agent` 호출 | 하나의 `Subagents` 배열에 여러 항목 |
| Wait for result | (동기 반환) | `wait_agent` | (동기 반환) |
| Terminate/free | 불필요 (이미 종료) | `close_agent` | 남아 있으면 `manage_subagents` kill |

## Interrupted Run Recovery

세션 시작 시 `orphaned_source` warning이 보고되면(`llm-wiki-core/hooks/session-start.sh`
출력 참고), 다음 절차를 따릅니다. 이 절차는 "source 파일은 작성됐지만
log.md 기록이 없는" 경우만 다룹니다 — Parallel Multi-Agent Ingest Run의
동시쓰기 문제와는 무관하며, 여기서는 사람이 응답한 뒤 에이전트 혼자
순차적으로 처리하므로 동시쓰기 위험이 없습니다.

1. 사용자에게 직접 묻습니다: "이전 ingest 작업이 완료되지 않았습니다
   (<N>개 항목: <목록>). 이어서 처리할까요?"
2. **"예"인 경우**, orphan 항목마다:
   - `llm-wiki/sources/<name>_summary.md`의 frontmatter에서 `created`와
     `sources:`를 읽습니다.
   - `## [<created의 날짜>] ingest | raw/<name>` 헤더 줄과, `- Source:
     <sources: 필드의 경로>`, `- Details: [복구됨] 이전 세션이 중단되어
     source는 작성되었으나 log 기록이 누락된 항목을 복구했습니다.` 줄로
     구성된 entry를 만듭니다.
   - 이 entry들을 `llm-wiki/log.md`에 직접 추가합니다.
   - 몇 개를 복구했는지 요약해서 보고합니다.
3. **"아니요"인 경우**, 아무것도 쓰지 않고 그대로 둡니다. 다음 세션
   시작 시 같은 항목이 다시 보고됩니다 (상태가 바뀌지 않았으므로).

## Commands

Ingest 요청 생성 및 관리 명령어 예시:
```bash
# Ingest가 허용된 Sync Point 목록 확인
llm-wiki-core/scripts/llm-wiki-core --root . list-sync-points (또는 구현된 MCP 도구 활용)

# Ingest 요청 제출 (개발자 에이전트 전용)
# 직접적인 파일 복사 대신 소스 메타데이터를 request 큐에 제출합니다.
```

## Common Pitfalls

1. **프로젝트 레포에 raw 파일 직접 복사**: `llm-wiki/` 하위에 대용량 로그나 PDF, 검증되지 않은 원시 데이터를 직접 복사하거나 깃에 커밋하는 행위는 금지됩니다.
2. **동기화 포인트 없는 자율 Ingest**: 큐레이터의 승인(`ready_for_ingest`) 없이 임의의 경로를 라이브로 Ingest하려고 시도하는 것은 아키텍처에 위배됩니다.

## Verification Checklist

- [ ] 원본 파일이 외부 Source Vault 또는 격리된 디렉터리에 위치해 있는가?
- [ ] 대용량 바이너리, 민감한 개인정보, Secrets가 제외되었는가?
- [ ] Ingest 요청 메타데이터에 `source_uri` 및 `sha256`가 명시되었는가?
- [ ] 큐레이터 승인 이전에는 canonical wiki 데이터를 수정하지 않고 대기하는가?
- [ ] 병렬 디스패치가 가능한 런타임에서는 배치 내 모든 항목을 한 번의 응답/턴에서 동시에 디스패치했는가?
- [ ] 병렬 디스패치가 불가능한 런타임(예: `multi_agent` 비활성 Codex)에서는 동일한 계약으로 순차 fallback했는가?
- [ ] 배치 종료 후 플랫폼별 종료 동작(Codex `close_agent`, Antigravity `manage_subagents` kill)을 수행했는가?
- [ ] 항목 하나의 실패가 배치 전체를 중단시키지 않았는가?
- [ ] `concepts/`, `decisions/`, `comparisons/`, Project/Team 영역을 건드리지 않았는가?
- [ ] log.md는 subagent가 아니라 orchestrator만 직접 쓰는가?
