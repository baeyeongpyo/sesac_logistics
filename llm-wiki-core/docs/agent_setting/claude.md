# Claude Code 설정 가이드

## 1. 파일 복사 및 생성

* **생성 대상**: `llm-wiki/`, `llm-wiki-core/`, 프로젝트 루트 `wiki_stack.yaml`
  * **권장 명령**: `llm-wiki-core/scripts/init-llm-wiki.sh --dest .`
* **생성 대상**: Agent OS overlay와 에이전트 entrypoint
  * **권장 명령**: `llm-wiki-core/scripts/init-agent-os.sh --dest .`
  * **저장 위치**: 프로젝트 루트 `/AGENTS.md`
* **선택 대상**: Claude 전용 entrypoint
  * **저장 위치**: 프로젝트 루트 `/CLAUDE.md`
  * 내용은 `AGENTS.md`를 먼저 읽고 `llm-wiki-core/skills/...`를 참조하도록 둡니다.
* **참조 대상**: llm-wiki-core 제공 skill
  * **저장 위치**: `llm-wiki-core/skills/`
  * root `skills/` 또는 `.claude/skills/`로 복사하지 않습니다.

---

## 2. 기존 설정 파일 수정

* **대상 파일**: `.claude/settings.json`
* **수정 내용**: 아래 `hooks` 설정을 파일 내에 추가/머지합니다.

```json
{
  "hooks": {
    "SessionStart": [
      {
        "hooks": [
          {
            "type": "command",
            "command": "\"$CLAUDE_PROJECT_DIR/llm-wiki-core/hooks/pre-bundle-validate.sh\" \"$CLAUDE_PROJECT_DIR\"",
            "timeout": 30,
            "statusMessage": "Validating wiki stack config..."
          },
          {
            "type": "command",
            "command": "\"$CLAUDE_PROJECT_DIR/llm-wiki-core/hooks/session-start.sh\" \"$CLAUDE_PROJECT_DIR\"",
            "timeout": 30,
            "statusMessage": "Loading llm-wiki-core context bundle..."
          }
        ]
      }
    ],
    "UserPromptSubmit": [
      {
        "hooks": [
          {
            "type": "command",
            "command": "\"$CLAUDE_PROJECT_DIR/llm-wiki-core/hooks/user-prompt-submit-wiki-context.sh\" --agent claude --root \"$CLAUDE_PROJECT_DIR\"",
            "timeout": 30,
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

* Claude Code에서 `/hooks`를 열어 프로젝트 hook을 승인합니다.
* `UserPromptSubmit` hook은 Claude Code의 `additionalContext` 형식으로 context reminder를 주입합니다.
* Stop hook은 기본으로 연결하지 않습니다. 실패, 교훈, 실험 결과처럼 재사용 가치가 있는 내용만 명시적으로 capture 합니다.

```bash
printf 'what happened...' | llm-wiki-core/hooks/post-run-capture.sh . "Short title"
```

---

## 4. 검증

```bash
python3 -m json.tool .claude/settings.json >/dev/null
bash -n llm-wiki-core/hooks/pre-bundle-validate.sh
bash -n llm-wiki-core/hooks/session-start.sh
bash -n llm-wiki-core/hooks/user-prompt-submit-wiki-context.sh
llm-wiki-core/hooks/user-prompt-submit-wiki-context.sh --agent claude --root .
```
