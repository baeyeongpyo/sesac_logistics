---
name: llm-wiki-pack
description: Use when packaging a personal/local llm-wiki into a folder artifact or optional .wikipkg artifact to share with teammates, distinct from formal Project/Team promotion.
version: 1.0.0
author: Antigravity Agent
license: MIT
metadata:
  hermes:
    tags: [llm-wiki, pack, artifact, folder-artifact, wikipkg, sharing]
    related_skills: [llm-wiki-ingest, llm-wiki-query, agent-os-wiki-bundle]
---

# LLM Wiki Pack

## Overview

개인/로컬 `llm-wiki`를 다른 팀원에게 공유하고 싶을 때, 기본은 기존 wiki 폴더 구조를 유지한 folder artifact로 만듭니다. folder artifact는 압축 해제가 필요 없고 Git diff/review가 쉽습니다. 단일 파일 배포나 외부 캐시가 필요할 때만 `.wikipkg`(zip 기반)를 선택합니다.

Promotion(공식 Project/Team canonical truth 반영)과는 다릅니다. artifact 패키징은 "내용을 읽기용 dependency로 공유"하는 것이고, Project/Team 정식 반영은 `llm-wiki-submit`/`llm-wiki-promotion` 스킬의 영역입니다.

## When to Use

- 개인 `llm-wiki`의 현재 상태를 다른 팀원에게 folder artifact로 공유하고 싶을 때
- 외부 배포나 캐시를 위해 선택적으로 `.wikipkg` 파일이 필요할 때
- 받는 쪽이 `search-pages`/`get-lineage`/`get-context-bundle`로 바로 query할 수 있는 형태가 필요할 때
- Project/Team canonical truth로 정식 승격하는 것이 아니라, 가벼운 공유/배포가 목적일 때

## Required Flow

1. **Ingest/정제 상태 확인 (Pre-flight)**:
   - `raw/`의 모든 원본이 `sources/`(또는 `concepts/`)로 이미 ingest·정제되어 있는지 확인합니다 (`llm-wiki-ingest` 참고).
   - `index.md`/`log.md`가 현재 상태와 일치하는지 확인합니다. 불일치하면 먼저 갱신합니다.
   - 페이지 frontmatter에 `stale: true`/`review_needed: true`가 남아있지 않은지 확인합니다.

2. **패킹 (Pack)**:
   - 기본은 `llm-wiki/` 내용을 `artifacts/<artifact-id>/` 폴더로 복사합니다.
   - `raw/`는 folder artifact에서 제거하지 않습니다 — 정제 문서의 근거(lineage/evidence)로 아티팩트 안에 그대로 남아야 합니다.
   - `raw/`가 검색 대상에서 빠지는 것은 패킹 시점이 아니라 **쿼리 시점**입니다 — `llm_wiki_core.core`의 `DEFAULT_EXCLUDE_GLOBS`에 `raw/**`가 있어서, 어떤 binding 타입(personal_wiki든 artifact든)으로 읽히든 `raw/`는 항상 "selected pages"에서 제외됩니다.
   - 단일 파일 배포가 필요할 때만 `pack-artifact` 명령으로 `.wikipkg`를 만듭니다.

3. **전달 및 등록 안내 (Share & Register)**:
   - 생성된 folder artifact 또는 `.wikipkg` 파일을 팀원에게 전달합니다.
   - 받는 사람은 자신의 `wiki_stack.yaml`에 `wiki_artifacts` + `source_binding_order` 항목으로 등록해야만 query에 인식됩니다. **등록하지 않으면 그 파일은 완전히 무시됩니다** (중복도 아니고 그냥 0건).
   - 등록 후 `get-context-bundle`을 한 번 실행하면 바로 `search-pages`/`get-lineage`가 동작합니다. 별도로 bundle 파일을 아티팩트 안에 미리 넣어둘 필요는 없습니다 — bundle은 binding ID/경로가 받는 쪽의 `wiki_stack.yaml` 구성에 종속적이라, dev 환경에서 만든 bundle을 그대로 끼워넣으면 오히려 안 맞을 수 있습니다.

## Commands

```bash
# 1. 기본 folder artifact 생성
mkdir -p artifacts/team-wiki
cp -R llm-wiki/. artifacts/team-wiki/
cat > artifacts/team-wiki/artifact.yaml <<'YAML'
version: "0.1.0"
namespace: team:wiki
format: folder
YAML

# 2. (선택) 단일 파일 배포용 .wikipkg 생성
llm-wiki-core/scripts/llm-wiki-core --root . pack-artifact \
  --source llm-wiki \
  --output artifacts/team-wiki.wikipkg
```

받는 사람 쪽 `wiki_stack.yaml` 등록 예시:
```yaml
wiki_artifacts:
  - dependency_id: team-wiki
    artifact_ref: artifacts/team-wiki
    namespace: team:wiki

personal_wikis: []

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: team-wiki
      dependency_id: team-wiki
      dependency_type: artifact
      effective_scope: team
      authority_level: policy
    - source_binding_id: local-mutable-wiki
```

`local-mutable-wiki`는 현재 프로젝트의 직접 편집 가능한 `llm-wiki/`를 뜻하는 예약 binding입니다. `dependency_type`, `effective_scope`, `authority_level`을 생략하면 각각 `local_wiki`, `local`, `working`으로 적용됩니다. 팀 artifact를 local보다 우선하려면 위 순서를 유지하고, local 작업본을 우선하려면 `local-mutable-wiki`를 먼저 둡니다.

```bash
# 등록 후 바로 query 가능
llm-wiki-core/scripts/llm-wiki-core --root . validate
llm-wiki-core/scripts/llm-wiki-core --root . get-context-bundle
llm-wiki-core/scripts/llm-wiki-core --root . search-pages "<검색어>"
```

## Common Pitfalls

1. **folder artifact와 배포 archive 혼동**: 기본 개발/리뷰/실행은 folder artifact를 사용합니다. `.wikipkg`는 단일 파일 배포나 외부 캐시가 필요할 때만 만듭니다.
2. **raw를 패킹에서 빼려고 시도**: raw를 패킹 단계에서 제거하면 정제 문서의 lineage/evidence가 끊깁니다. raw는 그대로 두고, 쿼리 단계의 자동 제외(`DEFAULT_EXCLUDE_GLOBS`)에 맡깁니다.
3. **등록 없이 그냥 전달**: artifact만 전달하고 `wiki_stack.yaml` 등록 스니펫을 같이 안 주면 받는 사람 쪽에서 영구히 query가 안 됩니다 (조용히 0건으로 무시됨, 에러도 안 남).
4. **dev 환경 bundle을 그대로 끼워 보내기**: bundle은 binding ID와 페이지 경로가 패킹한 쪽의 `wiki_stack.yaml`에 종속적입니다. 받는 쪽 환경에서 직접 `get-context-bundle`을 한 번 실행하게 하는 것이 맞습니다.

## Verification Checklist

- [ ] `raw/`의 모든 원본이 `sources/`(또는 다른 정제 폴더)로 ingest·정제되어 있는가
- [ ] `index.md`/`log.md`가 현재 wiki 상태와 일치하는가
- [ ] `artifacts/<artifact-id>/artifact.yaml`에 namespace와 format이 기록되어 있는가
- [ ] `.wikipkg`를 선택했다면 `pack-artifact` 결과 JSON의 `ok`가 `true`이고 `file_count`가 예상과 맞는가
- [ ] 받는 사람에게 `wiki_artifacts`/`source_binding_order` 등록 스니펫을 함께 전달했는가
