# llm-wiki-core Workflow 사용 가이드

이 문서는 실제 작업할 때 따라가는 명령과 파일 위치를 정리한다.

artifact 연결, 외부 경로, missing artifact 동작, promotion queue의 세부 구조는
`llm-wiki-core/docs/artifact_usage.md`를 먼저 확인한다.

```text
1. raw 파일 넣기
2. raw 목록 확인
3. ingest 결과 작성
4. local wiki 검증
5. promotion package 작성 및 submit 검증
6. promotion review 후 반영
7. artifact 빌드 또는 pack
```

## 1. raw 파일 넣기

원본 자료를 `llm-wiki/raw/`에 넣는다.

```text
llm-wiki/raw/
  example_source.md
  meeting_notes.md
  external_reference.md
```

예시:

```bash
cp ~/Downloads/example_source.md llm-wiki/raw/example_source.md
```

넣은 뒤 목록을 확인한다.

```bash
llm-wiki-core/scripts/llm-wiki-core --root . list-raw-items
```

복합 raw item이면 prepared unit이 있는지 확인한다.

```bash
llm-wiki-core/scripts/llm-wiki-core --root . get-raw-derived-manifest <raw-id>
```

## 2. ingest 결과 작성

작은 raw 파일은 `llm-wiki/sources/<name>_summary.md`로 요약한다.

```text
llm-wiki/raw/example_source.md
  -> llm-wiki/sources/example_source_summary.md
```

source summary 예시:

```markdown
---
title: Example Source Summary
status: accepted
confidence: medium
sources:
  - llm-wiki/raw/example_source.md
---

# Example Source Summary

## Key Points

- ...
```

작성 후 `llm-wiki/log.md`에 ingest 기록을 추가한다.

```markdown
## [2026-06-22] ingest | raw/example_source.md
- Source: llm-wiki/raw/example_source.md
- Details: Created llm-wiki/sources/example_source_summary.md.
```

복합 raw item은 prepared unit별로 결과 파일을 만든다.

```text
llm-wiki/raw-derived/<raw-id>/prepared/<unit-id>/index.yaml
  -> llm-wiki/sources/<output-name>.md
```

## 3. local wiki 검증

ingest 결과를 만든 뒤 검증한다.

```bash
llm-wiki-core/scripts/llm-wiki-core --root . validate
llm-wiki-core/scripts/llm-wiki-core --root . get-context-bundle
```

검색으로 문서가 잡히는지 확인한다.

```bash
llm-wiki-core/scripts/llm-wiki-core --root . search-pages "example source"
```

lineage가 필요하면 확인한다.

```bash
llm-wiki-core/scripts/llm-wiki-core --root . get-lineage llm-wiki/sources/example_source_summary.md
```

## 4. promotion package 작성

local wiki 내용을 Project/Team wiki로 올리고 싶으면 promotion package YAML을 만든다.

예시 파일:

```text
promotion-package.yaml
```

예시 내용:

```yaml
promotion_package:
  source_owner: "owner-name"
  claims:
    - claim_id: "claim-001"
      content: "Example source summary should be added as project reference."
      target_section: "sources/example_source_summary"
  evidence_digest:
    - "llm-wiki/raw/example_source.md was summarized into llm-wiki/sources/example_source_summary.md."
  lineage:
    - page_path: "llm-wiki/sources/example_source_summary.md"
      sha256: "replace-with-page-hash"
  confidence: medium
  requested_target_pages:
    - "sources/example_source_summary.md"
  raw_transfer_policy: raw_copy
  raw_items:
    - pack_path: "files/raw/example_source.md"
      target_path: "raw/example_source.md"
      sha256: "replace-with-raw-hash"
  refined_pages:
    - pack_path: "files/sources/example_source_summary.md"
      target_path: "sources/example_source_summary.md"
      sha256: "replace-with-page-hash"
  reviewer_required: true
```

`promotion_package`에는 target artifact를 적지 않는다. submit 단계도 target artifact를 알지 않고, queue 처리자가 별도 프로세스에서 외부 artifact wiki로 반영한다.
`raw_transfer_policy: raw_copy`이면 `raw_items`가 필수이며 raw 파일이 queue에 복사된다. `none`, `excerpt`, `source_vault_ref`이면 raw 파일 복사본 없이 refined page만 staging될 수 있고, raw 근거는 policy에 맞게 evidence digest, excerpt, 외부 source reference로 설명한다.

검증한다.

```bash
llm-wiki-core/scripts/llm-wiki-core validate-promotion promotion-package.yaml
```

hook으로도 같은 검증을 실행할 수 있다.

```bash
llm-wiki-core/hooks/promotion-submit-validate.sh promotion-package.yaml
```

## 5. submit

검증이 통과한 package를 `llm-wiki-promotion-queue/` 아래 review queue에 raw/refined 파일과 함께 staging한다. submit은 현재 repo의 `wiki_stack.yaml`이나 `llm-wiki/`를 읽지 않고, promotion package 폴더 안의 `files/**`만 검증한다.

```bash
llm-wiki-core/scripts/llm-wiki-core --root . submit-promotion promotion-package.yaml
```

기본 출력 구조:

```text
llm-wiki-promotion-queue/
  20260708-153000-promotion-package/
    promotion-package.yaml
    submission.yaml
    files/
      raw/example_source.md
      sources/example_source_summary.md
```

queue에 복사된 `promotion-package.yaml`에는 `promotion_package.submitted_at`이 자동으로 추가되어 언제 올린 package인지 확인할 수 있다. 같은 시각은 `submission.yaml`의 `submitted_at`에도 기록된다.

커밋 예시:

```bash
git add llm-wiki-promotion-queue/20260708-153000-promotion-package
git commit -m "docs: submit example source summary promotion"
```

프로젝트에서 별도 incoming queue 디렉터리를 쓰면 그 경로에 둔다.

## 6. promotion review 후 반영

reviewer가 package를 승인하면 target wiki page를 생성하거나 수정한다.

예시 반영 대상:

```text
llm-wiki/sources/example_source_summary.md
llm-wiki/index.md
llm-wiki/log.md
```

반영 후 다시 검증한다.

```bash
llm-wiki-core/scripts/llm-wiki-core --root . validate
llm-wiki-core/scripts/llm-wiki-core --root . get-context-bundle
```

## 7. folder artifact 만들기

기본 artifact는 기존 wiki 폴더 구조를 그대로 사용하는 folder artifact다.
압축 해제 없이 읽을 수 있고, Git diff/review가 쉽다.

빈 artifact wiki를 새로 시작할 때:

```bash
llm-wiki-core/scripts/init-artifact-wiki.sh \
  --dest artifacts/team-wiki \
  --version 0.1.0 \
  --title "Team Wiki"
```

이 명령은 artifact 생성만 담당한다. `wiki_stack.yaml`은 사용하는 프로젝트에서 직접 작성한다. artifact 자체의 `artifact.yaml`에는 namespace나 format을 넣지 않는다.

기존 local wiki를 artifact로 만들 때:

```bash
mkdir -p artifacts
cp -R llm-wiki artifacts/team-wiki
```

artifact metadata가 필요하면 `artifact.yaml`을 추가한다.

```yaml
version: "0.1.0"
```

받는 쪽 `wiki_stack.yaml` 등록 예시:

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

등록 후 확인:

```bash
llm-wiki-core/scripts/llm-wiki-core --root . validate
llm-wiki-core/scripts/llm-wiki-core --root . get-context-bundle
llm-wiki-core/scripts/llm-wiki-core --root . search-pages "example source"
```

## 8. .wikipkg artifact 만들기

외부 다운로드, 캐시, 단일 파일 배포가 필요할 때만 `.wikipkg`를 만든다.

```bash
mkdir -p artifacts
llm-wiki-core/scripts/llm-wiki-core --root . pack-artifact \
  --source llm-wiki \
  --output artifacts/team-wiki.wikipkg
```

받는 쪽 `wiki_stack.yaml` 등록 예시:

```yaml
wiki_artifacts:
  - dependency_id: team-wiki
    artifact_ref: artifacts/team-wiki.wikipkg
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

## 9. tar.gz artifact 만들기

tarball artifact가 필요하면 helper script를 사용한다.

```bash
mkdir -p artifacts
llm-wiki-core/scripts/build-wiki-artifact.sh \
  --src ./llm-wiki \
  --dest ./artifacts/team-wiki-1.0.0.tar.gz \
  --namespace team:wiki
```

## 10. 자주 쓰는 명령 모음

```bash
# raw 목록
llm-wiki-core/scripts/llm-wiki-core --root . list-raw-items

# raw-derived manifest
llm-wiki-core/scripts/llm-wiki-core --root . get-raw-derived-manifest <raw-id>

# 설정 검증
llm-wiki-core/scripts/llm-wiki-core --root . validate

# bundle 생성
llm-wiki-core/scripts/llm-wiki-core --root . get-context-bundle

# 검색
llm-wiki-core/scripts/llm-wiki-core --root . search-pages "<query>"

# lineage 확인
llm-wiki-core/scripts/llm-wiki-core --root . get-lineage <page-path>

# promotion package 검증
llm-wiki-core/scripts/llm-wiki-core validate-promotion <package.yaml>

# promotion package submit staging
llm-wiki-core/scripts/llm-wiki-core --root . submit-promotion <package.yaml>

# .wikipkg 생성
llm-wiki-core/scripts/llm-wiki-core --root . pack-artifact --output artifacts/wiki.wikipkg
```
