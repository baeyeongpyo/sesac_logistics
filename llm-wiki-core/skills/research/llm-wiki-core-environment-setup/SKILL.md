---
name: llm-wiki-core-environment-setup
description: Use when adding or validating Team, Project, Reference, or Personal wiki sources in a project-local llm-wiki-core runtime while preserving read-only artifact and promotion boundaries.
version: 1.0.0
author: Hermes Agent
license: MIT
metadata:
  hermes:
    tags: [llm-wiki-core, wiki, source-binding, artifact, personal-wiki]
    related_skills: [raw-source-governance]
---

# llm-wiki-core Environment Setup

## Overview

이 project-local skill은 `wiki_stack.yaml` 또는 `wiki_stack.example.yaml`을 안전하게 수정/검증할 때 사용한다.

핵심 규칙은 다음이다.

```text
read-time composition, write-time separation
```

`wiki_artifacts`와 `personal_wikis`는 외부 dependency registry이고, 실제 사용 의미는 `mutable_source_wiki_policy.source_binding_order`에 둔다.

현재 프로젝트의 직접 편집 가능한 `llm-wiki/`는 registry에 등록하지 않는다. 대신 예약 binding인 `local-mutable-wiki`를 `source_binding_order`에 명시한다. `dependency_type`, `effective_scope`, `authority_level`은 생략 가능하며, 생략 시 각각 `local_wiki`, `local`, `working`으로 해석된다.

기본 local-only stack 예시:

```yaml
wiki_artifacts: []
personal_wikis: []

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: local-mutable-wiki
```

## Workflow

1. `llm-wiki-core/scripts/llm-wiki-core --root . validate`로 현재 config를 검증한다.
2. Team/Project artifact나 외부 Personal Wiki가 필요할 때만 dependency를 registry에 추가한다.
3. `source_binding_order`에 runtime binding을 추가한다. 현재 프로젝트의 local wiki는 `local-mutable-wiki`를 사용한다.
4. Team/Project artifact를 직접 수정하지 않는다.
5. 다시 validate를 실행한다.
6. context bundle을 생성해 source order와 warnings를 확인한다. 이 bundle은
   이전 대화 transcript가 아니라 selected wiki pages, warnings/conflicts,
   source binding, lineage를 담는 derived artifact다.

## Commands

```bash
llm-wiki-core/scripts/llm-wiki-core --root . validate
llm-wiki-core/scripts/llm-wiki-core --root . get-context-bundle
```

기본 generated bundle retention은 최신 `run-YYYYMMDD-HHMMSS` 10개 유지,
최소 3개 보호다. 명시적 `--output` 경로는 호출자가 관리한다.

## Common Pitfalls

1. `effective_scope`를 `wiki_artifacts[]`에 넣지 않는다.
2. numeric `priority`를 만들지 않는다. YAML order가 priority다.
3. `group_id`/`parallel` 대신 literal `group:`을 사용한다.
4. 현재 프로젝트의 `llm-wiki/`를 `personal_wikis: path: llm-wiki`로 중복 등록하지 않는다. local wiki는 `local-mutable-wiki`로 binding한다.
5. Personal Wiki는 advisory이며 Project/Team을 자동 override하지 않는다.

## Verification Checklist

- [ ] Forbidden stale keys가 없다.
- [ ] 모든 non-local source binding이 존재하는 dependency를 참조한다.
- [ ] `local-mutable-wiki`는 `personal_wikis`가 아니라 `source_binding_order`에만 있다.
- [ ] `group:`은 같은 tier가 필요할 때만 쓴다.
- [ ] Project/Team artifact는 read-only snapshot으로 남는다.
