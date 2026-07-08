# Agent 설정 가이드

이 문서는 `llm-wiki-core`를 사용하는 에이전트 실행 환경을 설정하는 방법만 다룬다. `llm-wiki/` 생성과 `wiki_stack.yaml` 설정은 `llm-wiki-core/docs/llm_wiki_setup.md`에서 다룬다.

## 1. Agent OS overlay 생성

Agent OS task/role overlay를 사용할 프로젝트에서는 다음을 실행한다.

```bash
llm-wiki-core/scripts/init-agent-os.sh --dest .
```

옵션:

```text
--dest PATH      생성 대상 프로젝트 루트. 기본값: .
--agents         AGENTS.md thin entrypoint를 설치한다. 기본값
--no-agents      AGENTS.md를 설치하지 않는다
--force          기존 파일을 덮어쓴다. 기본값은 기존 파일 보존
```

생성/복사되는 주요 구조:

```text
.agent-os/
  README.md
  roles/default.md
  tasks/llm-wiki-task.md

.agent-harness/
  bundles/
  pending-personal-captures/

AGENTS.md
```

`AGENTS.md`는 얇은 entrypoint다. 실제 규칙은 아래 core template에 둔다.

```text
llm-wiki-core/templates/wiki-core-agents.md
llm-wiki-core/templates/agent-os-agents.md
```

## 2. 에이전트별 hook 연결

사용하는 에이전트에 맞는 문서를 적용한다.

```text
llm-wiki-core/docs/agent_setting/codex.md
llm-wiki-core/docs/agent_setting/claude.md
llm-wiki-core/docs/agent_setting/antigravity.md
```

공통 hook 역할:

```text
pre-bundle-validate.sh
  wiki_stack.yaml boundary와 config를 검증한다.

session-start.sh
  session 시작 시 context bundle snapshot을 생성하거나 재사용한다.

user-prompt-submit-wiki-context.sh
  사용자 prompt 직전에 최신 bundle 경로와 selected pages reminder를 주입한다.

post-run-capture.sh
  실패, 교훈, 실험 결과처럼 재사용 가치가 있는 내용을 Personal capture 후보로 명시 저장한다.
```

Stop hook은 기본으로 자동 연결하지 않는다. 플랫폼별 stop event boilerplate를 저장하지 않고, 작업자가 재사용 가치가 있다고 판단한 내용만 명시적으로 capture한다.

```bash
printf 'what happened...' | llm-wiki-core/hooks/post-run-capture.sh . "Short title"
```

## 3. core-owned skill 위치

`llm-wiki-core`가 제공하는 skill은 core runtime asset이다.

```text
llm-wiki-core/skills/
  research/
  software-development/
```

root `skills/`로 복사하지 않는다. root `skills/`는 llm-wiki를 사용하는 downstream 프로젝트가 커스텀 skill을 만들 때만 사용할 수 있다.

## 4. agent context protocol

에이전트는 project-specific 질문에 답하기 전에 최신 context bundle을 확인해야 한다.

```text
.agent-harness/bundles/<run-id>/
  context_bundle.md
  warnings.yaml
  selected_pages.yaml
  source_lineage.yaml
```

bundle은 현재 wiki 상태에서 선택된 page, warnings/conflicts, source binding, lineage를 담는 읽기용 artifact다. 이전 대화 transcript가 아니므로 작업 이어받기는 task 문서, handoff 문서, capture 후보를 사용한다.

## 5. 검증

hook 스크립트 문법:

```bash
bash -n llm-wiki-core/hooks/pre-bundle-validate.sh
bash -n llm-wiki-core/hooks/session-start.sh
bash -n llm-wiki-core/hooks/user-prompt-submit-wiki-context.sh
```

에이전트별 설정 파일이 있으면 JSON 문법도 확인한다.

```bash
python3 -m json.tool .codex/hooks.json >/dev/null
python3 -m json.tool .claude/settings.json >/dev/null
python3 -m json.tool .agents/hooks.json >/dev/null
```

이 저장소 자체를 검증할 때는 전체 test script를 실행한다.

```bash
llm-wiki-core/scripts/run-tests.sh
```
