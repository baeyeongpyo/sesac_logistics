# llm-wiki 설정 가이드

이 문서는 `llm-wiki-core` 기반 프로젝트에서 local wiki runtime을 만드는 방법만 다룬다. Agent OS, AGENTS.md, Codex/Claude/Antigravity hook 설정은 `llm-wiki-core/docs/agent_setup.md`에서 다룬다.

## 1. 기본 wiki runtime 생성

새 프로젝트 루트에서 실행한다.

```bash
llm-wiki-core/scripts/init-llm-wiki.sh --dest . --domain "Project knowledge base"
```

옵션:

```text
--dest PATH      생성 대상 프로젝트 루트. 기본값: .
--domain TEXT    llm-wiki/SCHEMA.md에 기록할 wiki domain. 기본값: Project knowledge base
--force          기존 파일을 덮어쓴다. 기본값은 기존 파일 보존
```

생성/복사되는 주요 구조:

```text
llm-wiki/
  SCHEMA.md
  index.md
  log.md
  concepts/
  decisions/
  comparisons/
  queries/
  sources/
  metadata/
  raw/

wiki_stack.yaml

llm-wiki-core/
  hooks/
  scripts/
  templates/
  agent-os/
  tests/
  llm_wiki_core/
```

`llm-wiki/`는 현재 프로젝트에서 직접 편집 가능한 local wiki다. Team/Project artifact는 직접 수정하지 않고 read-only snapshot으로 소비한다.
`wiki_stack.yaml`은 프로젝트별 dependency 선언 파일이며 `llm-wiki-core/` 안에 두지 않는다.
artifact 연결과 운영 방식은 `llm-wiki-core/docs/artifact_usage.md`에서 별도로 다룬다.

## 2. 기본 source stack

`init-llm-wiki.sh`는 local-only stack을 만든다.

```yaml
wiki_artifacts: []
personal_wikis: []

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: local-promotion-shelf
    - source_binding_id: local-mutable-wiki
```

규칙:

* 현재 프로젝트의 `llm-wiki/`는 `personal_wikis`에 등록하지 않는다.
* local wiki는 예약 binding인 `local-mutable-wiki`로만 참조한다.
* promotion shelf는 예약 binding인 `local-promotion-shelf`로 참조하며, 별도 dependency registry 없이 local wiki보다 먼저 읽는다.
* `wiki_artifacts`와 `personal_wikis`는 외부 dependency registry다.
* folder artifact를 기본 포맷으로 사용하고, `.wikipkg`/tar archive는 배포용 선택 포맷으로만 사용한다.
* artifact 경로는 프로젝트 루트 기준 상대 경로를 권장한다.
* artifact가 없으면 validate는 구조만 통과할 수 있고, bundle 생성 시 warning으로 제외된다.
* 실제 읽기 우선순위는 `mutable_source_wiki_policy.source_binding_order`의 YAML 순서가 결정한다.
* Team/Project artifact나 외부 Personal Wiki가 필요할 때만 registry에 dependency를 추가한다.

## 3. wiki content 구조

기본 디렉터리 의미:

```text
llm-wiki/raw/
  원본 evidence input. silent overwrite, 삭제, untracked mutation을 피한다.

llm-wiki/sources/
  raw source를 정리한 source summary 또는 source registry-oriented page.

llm-wiki/concepts/
  재사용 가능한 개념, 설명, 구현 지식.

llm-wiki/decisions/
  accepted decision 또는 ADR-style record.

llm-wiki/comparisons/
  선택지 비교, trade-off 정리.

llm-wiki/queries/
  보존 가치가 있는 query result.

llm-wiki/metadata/
  state, event, registry, derived metadata.
```

모든 durable page는 frontmatter와 `llm-wiki/index.md` catalog를 갖는 것을 기본 원칙으로 한다. `llm-wiki/log.md`는 wiki action의 append-only 기록으로 사용한다.

## 4. 검증

설정 직후 실행한다.

```bash
llm-wiki-core/scripts/llm-wiki-core --root . validate
llm-wiki-core/scripts/llm-wiki-core --root . get-context-bundle
```

`wiki_stack.yaml`을 바꾼 뒤에도 같은 검증을 다시 실행한다.

## 5. 운영 경계

* `.agent-harness/bundles/`는 derived context snapshot이다. canonical wiki truth가 아니다.
* bundle은 이전 세션 transcript가 아니다. 이전 작업 상태는 handoff 문서, task 파일, capture 후보로 관리한다.
* Project/Team truth를 바꾸려면 직접 파일 수정이 아니라 promotion package를 작성하고 검증한다.
* 외부 artifact가 local wiki와 충돌하면 조용히 덮어쓰지 말고 conflict policy와 사용자 결정을 따른다.
