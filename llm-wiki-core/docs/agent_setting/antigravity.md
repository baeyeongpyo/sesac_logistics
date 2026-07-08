# Antigravity 설정 가이드

Antigravity는 사용 모드에 따라 hook 연결 방식이 다릅니다. 대화형 에이전트 환경에서는 `.agents/hooks.json`을 사용하고, SDK로 커스텀 에이전트를 만들 때는 같은 스크립트를 lifecycle hook에서 직접 호출합니다.

## 1. 파일 복사 및 생성

* **생성 대상**: `llm-wiki/`, `llm-wiki-core/`, 프로젝트 루트 `wiki_stack.yaml`
  * **권장 명령**: `llm-wiki-core/scripts/init-llm-wiki.sh --dest .`
* **생성 대상**: Agent OS overlay와 에이전트 entrypoint
  * **권장 명령**: `llm-wiki-core/scripts/init-agent-os.sh --dest .`
  * **저장 위치**: 프로젝트 루트 `/AGENTS.md`
* **참조 대상**: llm-wiki-core 제공 skill
  * **저장 위치**: `llm-wiki-core/skills/`
  * root `skills/` 또는 `.agents/skills/`로 복사하지 않습니다.

---

## 2. Antigravity 대화형 에이전트 설정

* **대상 파일**: `.agents/hooks.json`
* **수정 내용**: 아래 `wiki-context` 설정을 파일 내에 추가/머지합니다.

```json
{
  "wiki-context": {
    "PreInvocation": [
      {
        "type": "command",
        "command": "ROOT=$(git rev-parse --show-toplevel 2>/dev/null || pwd); \"$ROOT/llm-wiki-core/hooks/user-prompt-submit-wiki-context.sh\" --agent antigravity --root \"$ROOT\"",
        "timeout": 30
      }
    ]
  }
}
```

`PreInvocation`은 사용자 요청 직전에 context reminder를 주입합니다. 세션 시작 시 bundle을 미리 만들고 싶으면 아래 명령을 명시적으로 실행합니다.

```bash
ROOT=$(git rev-parse --show-toplevel 2>/dev/null || pwd)
"$ROOT/llm-wiki-core/hooks/pre-bundle-validate.sh" "$ROOT"
"$ROOT/llm-wiki-core/hooks/session-start.sh" "$ROOT"
```

---

## 3. Antigravity SDK 설정

* **대상 파일**: 에이전트 구동 파이썬 스크립트
* **수정 내용**: 에이전트 lifecycle hook에서 llm-wiki-core hook 스크립트를 호출합니다.

```python
import subprocess


def pre_invocation() -> None:
    subprocess.run(
        [
            "llm-wiki-core/hooks/user-prompt-submit-wiki-context.sh",
            "--agent",
            "antigravity",
            "--root",
            ".",
        ],
        check=False,
    )


def session_start() -> None:
    subprocess.run(["llm-wiki-core/hooks/pre-bundle-validate.sh", "."], check=False)
    subprocess.run(["llm-wiki-core/hooks/session-start.sh", "."], check=False)
```

실제 등록 API 이름은 사용하는 Antigravity 런타임에 맞춥니다. 핵심은 prompt 직전에 `user-prompt-submit-wiki-context.sh --agent antigravity --root <project-root>`를 호출하는 것입니다.

---

## 4. 추가 조치

* Stop hook은 기본으로 연결하지 않습니다. 실패, 교훈, 실험 결과처럼 재사용 가치가 있는 내용만 명시적으로 capture 합니다.

```bash
printf 'what happened...' | llm-wiki-core/hooks/post-run-capture.sh . "Short title"
```

---

## 5. 검증

```bash
python3 -m json.tool .agents/hooks.json >/dev/null
bash -n llm-wiki-core/hooks/pre-bundle-validate.sh
bash -n llm-wiki-core/hooks/session-start.sh
bash -n llm-wiki-core/hooks/user-prompt-submit-wiki-context.sh
llm-wiki-core/hooks/user-prompt-submit-wiki-context.sh --agent antigravity --root .
```
