---
name: agent-os-wiki-bundle
description: Use when running Agent OS tasks in a project that uses llm-wiki-core so the agent consumes context bundles instead of directly merging wiki files or mutating Project/Team truth.
version: 1.0.0
author: Hermes Agent
license: MIT
metadata:
  hermes:
    tags: [agent-os, llm-wiki-core, context-bundle, hooks]
    related_skills: [llm-wiki-core-environment-setup]
---

# Agent OS Wiki Bundle

## Overview

Agent OS는 task/role lens이지 wiki truth store가 아니다. 작업자는 `llm-wiki-core` bundle snapshot을 읽고 실행한다.
이 bundle은 이전 세션 대화 transcript가 아니라 selected wiki pages,
warnings/conflicts, source binding, lineage를 담는 읽기용 derived artifact다.
이전 작업 상태는 Agent OS task, handoff 문서, capture 후보로 이어받는다.

## When to Use

- Agent OS task를 시작할 때
- 작업 context에 Team/Project/Personal Wiki가 섞일 수 있을 때
- conflict나 stale page가 있는지 확인해야 할 때
- 작업 결과를 Personal capture 또는 promotion package로 남길 때

## Required Flow

`llm-wiki-core/hooks/pre-bundle-validate.sh`와 `llm-wiki-core/hooks/session-start.sh`는 `SessionStart` hook이 세션 시작 시 자동 실행하므로 다시 실행하지 않는다. `.agent-harness/bundles/<run-id>/context_bundle.md`를 읽고 작업한다.

Generated bundle retention은 기본적으로 최신 `run-YYYYMMDD-HHMMSS` 10개를
유지하고 최소 3개를 보호한다. 명시적 `get-context-bundle --output <path>`
산출물은 호출자가 직접 관리한다.

Pending personal capture retention은 기본적으로
`.agent-harness/pending-personal-captures/`의 timestamp 형식 markdown 후보
최신 50개를 유지하고 최소 10개를 보호한다. 수동 파일명은 pruning 대상이
아니다.

Stop hook은 플랫폼별 boilerplate stop event를 자동 capture하지 않는다.
작업자가 실패, 교훈, 실험 결과처럼 재사용 가치가 있다고 판단한 내용만
명시적으로 capture한다.

## Promotion

Project/Team truth에 반영하려면 직접 파일 수정이 아니라 promotion package를 작성한다.

```bash
llm-wiki-core/hooks/promotion-submit-validate.sh promotion-package.yaml
```

## Capture

실패/교훈/실험 결과처럼 재사용 가치가 있는 내용은 Personal Wiki 후보로
명시적으로 남긴다.

```bash
printf 'what happened...' | llm-wiki-core/hooks/post-run-capture.sh . "Short title"
```

## Common Pitfalls

1. Agent OS가 Team/Project artifact 파일을 직접 뒤지는 것.
2. Personal Wiki page를 Project Wiki로 직접 복사하는 것.
3. conflict warning을 무시하고 autonomous decision처럼 처리하는 것.
4. generated/bundle artifact를 canonical wiki data로 취급하는 것.
5. bundle을 이전 대화 전체를 복원하는 세션 transcript로 취급하는 것.

## Verification Checklist

- [ ] pre-bundle validation 통과.
- [ ] context bundle 생성.
- [ ] warnings/conflicts 검토.
- [ ] 이전 작업 상태가 필요하면 handoff/task/capture 문서를 별도로 확인.
- [ ] Project/Team 변경은 promotion package로만 submit.
