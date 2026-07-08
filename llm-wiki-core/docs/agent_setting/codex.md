# Codex 설정 가이드

## 1. 파일 복사 및 생성

* **생성 대상**: `llm-wiki/`, `llm-wiki-core/`, 프로젝트 루트 `wiki_stack.yaml`
  * **권장 명령**: `llm-wiki-core/scripts/init-llm-wiki.sh --dest .`
* **생성 대상**: Agent OS overlay와 에이전트 entrypoint
  * **권장 명령**: `llm-wiki-core/scripts/init-agent-os.sh --dest .`
  * **저장 위치**: 프로젝트 루트 `/AGENTS.md`
* **참조 대상**: llm-wiki-core 제공 skill
  * **저장 위치**: `llm-wiki-core/skills/`
  * root `skills/`로 복사하지 않습니다. root `skills/`는 downstream 프로젝트가 커스텀 skill을 만들 때만 사용합니다.

---

## 2. 기존 설정 파일 수정

* **대상 파일**: `.codex/hooks.json`
* **수정 내용**: 아래 `hooks` 설정을 파일 내에 추가/머지합니다.

```json
{
  "hooks": {
    "SessionStart": [
      {
        "matcher": "startup|resume",
        "hooks": [
          {
            "type": "command",
            "command": "ROOT=$(git rev-parse --show-toplevel 2>/dev/null || pwd); if [ -x \"$ROOT/llm-wiki-core/hooks/pre-bundle-validate.sh\" ]; then \"$ROOT/llm-wiki-core/hooks/pre-bundle-validate.sh\" \"$ROOT\"; fi; if [ -x \"$ROOT/llm-wiki-core/hooks/session-start.sh\" ]; then \"$ROOT/llm-wiki-core/hooks/session-start.sh\" \"$ROOT\"; fi",
            "statusMessage": "Validating wiki stack config..."
          }
        ]
      }
    ],
    "UserPromptSubmit": [
      {
        "hooks": [
          {
            "type": "command",
            "command": "ROOT=$(git rev-parse --show-toplevel 2>/dev/null || pwd); \"$ROOT/llm-wiki-core/hooks/user-prompt-submit-wiki-context.sh\" --agent codex --root \"$ROOT\"",
            "statusMessage": "Injecting llm-wiki-core context..."
          }
        ]
      }
    ]
  }
}
```

---

## 3. 추가 조치

* Codex TUI에서 `/hooks`를 실행하고, 해당 프로젝트의 hook 항목을 trust 합니다.
* Codex의 `UserPromptSubmit`은 matcher filtering을 사용하지 않습니다. 설정되면 모든 사용자 prompt 제출 전에 context reminder를 주입합니다.
* Stop hook은 기본으로 연결하지 않습니다. 실패, 교훈, 실험 결과처럼 재사용 가치가 있는 내용만 명시적으로 capture 합니다.

```bash
printf 'what happened...' | llm-wiki-core/hooks/post-run-capture.sh . "Short title"
```

---

## 4. 검증

```bash
python3 -m json.tool .codex/hooks.json >/dev/null
bash -n llm-wiki-core/hooks/pre-bundle-validate.sh
bash -n llm-wiki-core/hooks/session-start.sh
bash -n llm-wiki-core/hooks/user-prompt-submit-wiki-context.sh
llm-wiki-core/hooks/user-prompt-submit-wiki-context.sh --agent codex --root .
```
