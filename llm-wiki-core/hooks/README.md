# llm-wiki-core Hooks

이 폴더의 hook은 Project/Team Wiki truth를 직접 수정하지 않는다. 모두 `llm-wiki-core` CLI를 통해 검증하거나 derived artifact를 생성한다.

| Hook | 기능 |
|---|---|
| `pre-bundle-validate.sh` | `wiki_stack.yaml`/`wiki_stack.example.yaml` boundary 검증 |
| `session-start.sh` | `wiki_get_context_bundle`에 해당하는 wiki-context bundle snapshot 생성 또는 재사용 |
| `user-prompt-submit-wiki-context.sh` | 사용자 prompt 직전에 최신 bundle 경로와 selected pages를 context로 주입 |
| `promotion-submit-validate.sh` | promotion package schema 검증. submit만 검증하며 accept/publish하지 않음 |
| `post-run-capture.sh` | 명시적으로 전달된 run 결과를 Personal Wiki 후보 캡처로 저장 |

Bundle snapshot은 이전 세션 대화 transcript가 아니라 selected wiki pages,
warnings/conflicts, source binding, lineage를 담는 derived artifact다. 이전
작업 상태는 handoff 문서나 명시적으로 생성한 `post-run-capture.sh` 산출물로
관리한다. Stop hook은 플랫폼별 boilerplate 문구를 자동 저장하지 않는다.

기본 generated bundle retention은 최신 `run-YYYYMMDD-HHMMSS` 10개를
유지하고 최소 3개를 보호한다. 명시적 `--output` 경로는 호출자가 관리한다.

기본 pending personal capture retention은
`.agent-harness/pending-personal-captures/`의 timestamp 형식 markdown 후보
최신 50개를 유지하고 최소 10개를 보호한다. 수동으로 만든 비정형 파일명은
자동 pruning 대상이 아니다.
