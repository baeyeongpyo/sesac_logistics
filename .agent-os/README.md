# Agent OS Overlay for llm-wiki-core

Agent OS는 wiki truth store가 아니라 task/role lens다. 모든 작업 context는 직접 파일 merge가 아니라 `llm-wiki-core` bundle query를 통해 얻는다.

Bundle은 이전 세션 대화 transcript가 아니다. selected wiki pages,
warnings/conflicts, source binding, lineage를 담는 읽기용 derived artifact다.
이전 작업 상태는 task 파일, handoff 문서, capture 후보로 관리한다.

## Mandatory Flow

`llm-wiki-core/hooks/pre-bundle-validate.sh`와 `llm-wiki-core/hooks/session-start.sh`는 Claude Code의 `SessionStart` hook이 매 세션 시작 시 자동 실행한다. Agent OS task는 그 결과물을 그대로 소비한다.

```text
Agent OS task
  -> .agent-harness/bundles/<run-id>/context_bundle.md
  -> agent execution
  -> explicit llm-wiki-core/hooks/post-run-capture.sh only when useful
  -> promotion package only if Project/Team truth should change
```

Generated bundle retention은 기본적으로 최신 `run-YYYYMMDD-HHMMSS` 10개를
유지하고 최소 3개를 보호한다.

Pending personal capture retention은 기본적으로 timestamp 형식의 capture
후보 최신 50개를 유지하고 최소 10개를 보호한다. 재사용 가치 판단은 오래된
bundle이 아니라 handoff/task/capture 후보를 통해 수행한다.
Stop hook은 boilerplate stop event를 자동 capture하지 않는다. 실패, 교훈,
실험 결과처럼 재사용 가치가 있는 내용만 명시적으로 capture한다.

## Do Not

- Team/Project artifact를 직접 수정하지 않는다.
- Personal Wiki를 Project/Team Wiki로 파일 복사 병합하지 않는다.
- conflict를 `llm-wiki-core` policy 없이 조용히 선택하지 않는다.
- `.agent-harness/generated` 또는 `.agent-harness/bundles`를 canonical wiki data로 취급하지 않는다.

## Roles

역할별 rule은 `.agent-os/roles/` 아래에 둔다. 모든 역할은 `llm-wiki-core`
context bundle을 먼저 소비하고, Project/Team Wiki truth를 직접 수정하지
않는다.

- `default.md`: 기본 Agent OS 실행자.
- `technical-lead.md`: 기술 방향, 아키텍처, 위험도, 검증 전략 분석.
- `pm.md`: 목표, 범위, 우선순위, acceptance criteria 정리.
- `qa.md`: 테스트 전략, 회귀 위험, 미검증 영역 확인.
- `reviewer.md`: 코드/문서 변경의 결함, 회귀, 유지보수 위험 리뷰.
- `ux.md`: 사용자 흐름, 정보 구조, 오류 회복, 운영 UX 검토.
