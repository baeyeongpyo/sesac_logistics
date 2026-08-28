# llm-wiki-core 설정 가이드

`llm-wiki-core` 설정은 두 단계로 나눈다.

```text
1. llm-wiki 설정
   -> local wiki runtime과 source stack 구성

2. agent 설정
   -> Agent OS overlay, AGENTS.md, 플랫폼별 hook 구성
```

각 단계는 독립된 문서를 따른다.

```text
llm-wiki-core/docs/llm_wiki_setup.md
llm-wiki-core/docs/agent_setup.md
llm-wiki-core/docs/artifact_usage.md
llm-wiki-core/docs/gitignore.md
llm-wiki-core/docs/workflow.md
```

권장 순서:

```bash
llm-wiki-core/scripts/init-llm-wiki.sh --dest . --domain "Project knowledge base"
llm-wiki-core/scripts/llm-wiki-core --root . validate
llm-wiki-core/scripts/llm-wiki-core --root . get-context-bundle

llm-wiki-core/scripts/init-agent-os.sh --dest .
```

그 다음 사용하는 에이전트에 맞는 hook 문서를 적용한다.

```text
llm-wiki-core/docs/agent_setting/codex.md
llm-wiki-core/docs/agent_setting/claude.md
llm-wiki-core/docs/agent_setting/antigravity.md
```

artifact 연결, 외부 경로, missing artifact 동작, promotion queue 구조는
`llm-wiki-core/docs/artifact_usage.md`를 따른다.

Git 추적/ignore 기준은 `llm-wiki-core/docs/gitignore.md`를 따른다.

운영 workflow는 raw 추가, ingest, submit, promotion, artifact build 순서로
`llm-wiki-core/docs/workflow.md`를 따른다.

## 경계

* `llm-wiki/`는 현재 프로젝트의 local mutable wiki다.
* `promotion-shelf/`는 promotion package에서 파생된 local-only lite artifact이며 pending runtime source로 읽힌다.
* `llm-wiki-core/`는 runtime, hooks, templates, skills, docs를 소유한다.
* `wiki_stack.yaml`은 프로젝트 루트의 dependency 선언 파일이며 프로젝트 성격에 맞게 관리한다.
* artifact는 core에 포함하지 않고 외부 folder artifact 또는 선택적 archive artifact로 연결한다.
* `.agent-os/`와 `.agent-harness/`는 agent execution overlay와 derived artifacts다.
* Project/Team truth는 직접 수정하지 않고 promotion package로 반영한다.
