# Artifact 사용 가이드

이 문서는 현재 `llm-wiki-core` 구조에서 `wiki_stack.yaml`로 artifact를 연결하고 사용하는 방법을 정리한다.

## 1. 현재 구조

`llm-wiki-core`는 runtime이고, 실제 wiki dependency는 프로젝트 루트의 `wiki_stack.yaml`에서 선언한다.

```text
project-root/
  wiki_stack.yaml
  llm-wiki/
    raw/
    sources/
    concepts/
    decisions/
  llm-wiki-core/
    scripts/
    hooks/
    docs/
  artifacts/
    team-wiki/
      raw/
      sources/
      concepts/
```

역할은 다음처럼 나눈다.

```text
llm-wiki-core/
  core runtime, CLI, hooks, docs, skills

wiki_stack.yaml
  현재 프로젝트가 사용할 wiki dependency 선언

llm-wiki/
  현재 프로젝트의 local mutable wiki

artifact
  외부에서 관리되는 read-only wiki snapshot 또는 shared wiki 폴더
```

`llm-wiki-core`는 artifact를 소유하지 않는다. `wiki_stack.yaml`이 어떤 artifact를 읽을지 선언하고, core는 그 선언을 따라 context bundle, search, lineage를 만든다.

## 2. artifact 기본 포맷

기본 artifact 포맷은 기존 wiki 폴더 구조다.

```text
artifacts/programming-architecture/
  artifact.yaml
  index.md
  log.md
  raw/
  sources/
  concepts/
  decisions/
```

folder artifact를 기본으로 쓰는 이유:

* 읽을 때 압축 해제가 필요 없다.
* Git diff와 review가 쉽다.
* raw 파일과 정제된 wiki page를 같은 구조 안에 보존할 수 있다.
* 외부 repo, shared directory, artifact cache에 그대로 연결하기 쉽다.

`.wikipkg`, `.tar.gz` 같은 압축 artifact도 여전히 지원하지만, 단일 파일 배포나 캐시가 필요할 때만 선택적으로 사용한다.

## 3. wiki_stack.yaml에 artifact 연결

프로젝트 루트의 `wiki_stack.yaml`에 artifact dependency를 선언한다.

```yaml
wiki_artifacts:
  - dependency_id: programming-architecture
    artifact_ref: artifacts/programming-architecture
    namespace: programming-architecture:wiki

personal_wikis: []

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: programming-architecture
      dependency_id: programming-architecture
      dependency_type: artifact
      effective_scope: team
      authority_level: policy
    - source_binding_id: local-mutable-wiki
```

필드 의미:

```text
dependency_id
  source_binding_order에서 참조할 artifact id

artifact_ref
  artifact 폴더 또는 artifact 파일 경로

namespace
  사람이 구분하기 위한 wiki namespace

source_binding_id
  context bundle과 lineage에 기록되는 source id

dependency_type: artifact
  이 binding이 artifact dependency를 읽는다는 선언

effective_scope
  local, personal, project, team 같은 적용 범위

authority_level
  working, reference, policy 같은 권위 수준
```

실제 읽기 우선순위는 `mutable_source_wiki_policy.source_binding_order`의 YAML 순서가 결정한다.

## 4. 경로 해석 규칙

`artifact_ref`는 절대경로와 상대경로를 모두 사용할 수 있다.

상대경로:

```yaml
artifact_ref: artifacts/programming-architecture
```

상대경로는 `--root`로 지정한 프로젝트 루트 기준으로 해석된다.

절대경로:

```yaml
artifact_ref: /Users/shared/wiki-artifacts/programming-architecture
```

절대경로는 그대로 사용된다. 외부 repo나 팀 공용 artifact cache에 있는 artifact도 연결할 수 있다.

팀 공유용으로는 절대경로보다 프로젝트 루트 기준 상대경로를 권장한다.

```text
../wiki-artifacts/programming-architecture
```

절대경로를 `wiki_stack.yaml`에 넣으면 팀원마다 경로가 달라질 수 있다. 이런 경우에는 artifact를 repo 옆의 공통 상대 위치에 두거나, 환경별 파일로 `wiki_stack.yaml`을 생성하는 방식을 사용한다.

## 5. artifact가 없을 때 동작

현재 정책은 missing artifact를 hard fail로 처리하지 않는다.

```text
wiki_stack.yaml에 dependency 선언은 있음
artifact_ref 경로에는 실제 artifact가 없음
```

이 경우 동작:

```text
validate
  dependency_id와 binding 구조만 검사한다.
  artifact 파일 존재 여부는 error로 보지 않는다.

get-context-bundle
  missing artifact warning을 남기고 해당 artifact를 제외한다.
  다른 artifact나 local wiki가 있으면 그 source만으로 bundle을 만든다.

search-pages
  최신 bundle에 포함된 page만 검색한다.
  빠진 artifact의 page는 검색되지 않는다.

get-lineage
  빠진 artifact의 page는 찾을 수 없다.
```

예상 warning:

```text
artifact ref missing for programming-architecture: /path/to/artifact
```

즉 현재 동작은 degraded mode다. 필수 artifact가 누락되어도 `validate`는 통과할 수 있으므로, 운영에서는 bundle 생성 결과의 `warnings`를 반드시 확인한다.

## 6. context bundle 생성과 검색

설정 후 기본 검증:

```bash
llm-wiki-core/scripts/llm-wiki-core --root . validate
llm-wiki-core/scripts/llm-wiki-core --root . get-context-bundle
```

검색:

```bash
llm-wiki-core/scripts/llm-wiki-core --root . search-pages "clean architecture"
```

lineage 확인:

```bash
llm-wiki-core/scripts/llm-wiki-core --root . get-lineage artifacts/programming-architecture/concepts/clean-architecture-core.md
```

artifact가 프로젝트 루트 밖에 있으면 page path가 절대경로로 기록될 수 있다.

```text
/Users/shared/wiki-artifacts/programming-architecture/concepts/clean-architecture-core.md
```

공유성과 재현성이 중요하면 artifact를 프로젝트 루트 기준 상대 위치로 연결하는 편이 낫다.

## 7. folder artifact 만들기

아무 artifact도 없는 환경에서 새 artifact wiki를 시작할 때는 빈 folder artifact를 먼저 만든다.

```bash
llm-wiki-core/scripts/init-artifact-wiki.sh \
  --dest ../wiki-artifacts/team-wiki \
  --version 1.0.0 \
  --title "Team Wiki"
```

이 명령은 artifact 생성만 담당한다. `wiki_stack.yaml`은 사용하는 프로젝트에서 직접 작성한다. artifact 자체의 `artifact.yaml`에는 namespace나 format을 기록하지 않는다.

생성되는 구조:

```text
../wiki-artifacts/team-wiki/
  artifact.yaml
  index.md
  log.md
  raw/
  sources/
  concepts/
  decisions/
  comparisons/
  queries/
  metadata/
```

기존 local wiki를 folder artifact로 만들 때는 기존 폴더 구조를 복사한다.

```bash
mkdir -p artifacts
cp -R llm-wiki artifacts/team-wiki
```

metadata가 필요하면 `artifact.yaml`을 둔다.

```yaml
version: "1.0.0"
```

folder/archive 여부는 `artifact.yaml`에 고정하지 않는다. 읽는 쪽은 `wiki_stack.yaml`의 `artifact_ref`가 폴더를 가리키는지, `.wikipkg`/tar archive 파일을 가리키는지로 판단한다.

이 artifact를 다른 프로젝트에서 사용할 때는 해당 프로젝트의 `wiki_stack.yaml`에 연결한다.

```yaml
wiki_artifacts:
  - dependency_id: team-wiki
    artifact_ref: ../wiki-artifacts/team-wiki
    namespace: team:wiki
```

## 8. .wikipkg와 archive 사용

`.wikipkg`는 기본 포맷이 아니다. 다음 상황에서만 사용한다.

```text
단일 파일로 배포해야 할 때
외부 다운로드/cache 처리가 필요할 때
압축 artifact로 version pinning을 하고 싶을 때
```

생성:

```bash
llm-wiki-core/scripts/llm-wiki-core --root . pack-artifact \
  --source llm-wiki \
  --output artifacts/team-wiki.wikipkg
```

연결:

```yaml
wiki_artifacts:
  - dependency_id: team-wiki
    artifact_ref: artifacts/team-wiki.wikipkg
    namespace: team:wiki
```

압축 artifact도 읽을 수 있지만, 매번 archive member를 읽어야 하므로 일반 운영에서는 folder artifact가 더 단순하다.

## 9. promotion과 submit

현재 프로젝트의 `llm-wiki/`에서 외부 artifact wiki로 지식을 올릴 때는 직접 artifact를 수정하지 않고 promotion package를 만든다.

promotion package는 target artifact를 모른다.

```text
알아야 하는 것:
  어떤 raw와 refined page를 승격 요청할지
  어떤 claim/evidence/lineage를 포함할지
  raw를 어떤 정책으로 전달할지

알면 안 되는 것:
  target artifact id
  target artifact path
  target binding id
  target scope
```

promotion package 예시:

```yaml
promotion_package:
  source_owner: "owner-name"
  claims:
    - claim_id: "claim-001"
      content: "Example source summary should be promoted."
      target_section: "sources/example_source_summary"
  evidence_digest:
    - "files/raw/example_source.md was summarized into files/sources/example_source_summary.md."
  lineage:
    - page_path: "files/sources/example_source_summary.md"
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

package 폴더 구조:

```text
promotion-pack/
  promotion.yaml
  files/
    raw/example_source.md
    sources/example_source_summary.md
```

검증:

```bash
llm-wiki-core/scripts/llm-wiki-core validate-promotion promotion-pack/promotion.yaml
```

submit:

```bash
llm-wiki-core/scripts/llm-wiki-core --root . submit-promotion promotion-pack/promotion.yaml
```

submit 결과는 target-free queue에 남는다.

```text
llm-wiki-promotion-queue/
  20260708-153000-promotion/
    promotion.yaml
    submission.yaml
    files/
      raw/example_source.md
      sources/example_source_summary.md
```

submit은 현재 repo의 `llm-wiki/`나 target artifact를 읽지 않는다. package 안의 `pack_path`, `sha256`, `target_path`, `raw_transfer_policy`만 검증하고 queue에 복사한다.

외부 repo나 별도 promotion processor가 이 queue를 받아 실제 target artifact wiki에 반영한다.

## 10. raw_transfer_policy

`raw_transfer_policy`는 raw evidence를 promotion package에 어떻게 포함할지 정한다.

```text
raw_copy
  raw_items가 필수다.
  raw 파일을 promotion queue에 같이 복사한다.

none
  raw 파일을 package에 포함하지 않는다.
  refined_pages만 submit할 수 있다.

excerpt
  raw 전체 대신 필요한 excerpt나 evidence_digest로 근거를 설명한다.

source_vault_ref
  raw 파일은 별도 vault/storage에 있고, package에는 참조 정보만 둔다.
```

submit은 policy에 맞게 적용한다.

```text
raw_copy인데 raw_items가 없으면 실패
none/excerpt/source_vault_ref이면 raw_items 없이도 submit 가능
non-copy policy에서 raw_items가 있으면 파일과 sha256은 검증 후 queue에 복사
```

## 11. 권장 운영 방식

팀에서 공유할 때 권장 구조:

```text
workspace/
  project-a/
    wiki_stack.yaml
    llm-wiki-core/
    llm-wiki/
  wiki-artifacts/
    programming-architecture/
    tracking-vehicle/
```

`project-a/wiki_stack.yaml`:

```yaml
wiki_artifacts:
  - dependency_id: programming-architecture
    artifact_ref: ../wiki-artifacts/programming-architecture
    namespace: programming-architecture:wiki
```

이 방식은 팀원이 같은 repo 배치를 사용하면 YAML을 그대로 공유할 수 있다. 팀원마다 artifact 위치가 다르면 `wiki_stack.yaml`을 직접 수정하지 말고 프로젝트별 bootstrap script나 로컬 설정으로 생성하는 편이 안전하다.

## 12. .gitignore 설정

`.gitignore`에 바로 붙여넣을 기본 블록과 `artifacts/` 운영 방식별 선택지는 별도 문서를 따른다.

```text
llm-wiki-core/docs/gitignore.md
```

핵심 기준:

```text
llm-wiki/
  일반 프로젝트 repo에서는 ignore한다.

wiki_stack.yaml
  프로젝트 dependency 선언이므로 공유한다.

llm-wiki-promotion-queue/
  target-free promotion package review queue이므로 기본적으로 공유한다.

artifacts/
  현재 repo가 artifact snapshot을 소유하면 공유하고,
  외부 cache나 별도 repo에서 관리하면 ignore한다.
```

## 13. 자주 쓰는 명령

```bash
# stack 구조 확인
llm-wiki-core/scripts/llm-wiki-core --root . validate

# artifact/local wiki를 읽어 context bundle 생성
llm-wiki-core/scripts/llm-wiki-core --root . get-context-bundle

# 최신 bundle 검색
llm-wiki-core/scripts/llm-wiki-core --root . search-pages "<query>"

# page lineage 확인
llm-wiki-core/scripts/llm-wiki-core --root . get-lineage <page-path>

# raw 목록 확인
llm-wiki-core/scripts/llm-wiki-core --root . list-raw-items

# promotion package 검증
llm-wiki-core/scripts/llm-wiki-core validate-promotion <promotion.yaml>

# promotion queue에 submit
llm-wiki-core/scripts/llm-wiki-core --root . submit-promotion <promotion.yaml>

# 선택적으로 .wikipkg 생성
llm-wiki-core/scripts/llm-wiki-core --root . pack-artifact --source llm-wiki --output artifacts/wiki.wikipkg
```
