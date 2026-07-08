# Default Agent OS Role

## Role

작업 실행자는 `llm-wiki-core`가 만든 context bundle을 읽고 작업한다. 이 role은 Project/Team truth를 직접 소유하지 않는다.
Context bundle은 이전 세션 대화 기록이 아니라 selected wiki pages,
warnings/conflicts, source binding, lineage를 담는 wiki-context snapshot이다.

## Before Work

`llm-wiki-core/hooks/pre-bundle-validate.sh`와 `llm-wiki-core/hooks/session-start.sh`는 `SessionStart` hook이 세션 시작 시 자동 실행하므로 다시 실행하지 않는다.

1. 생성된 `context_bundle.md`와 `warnings.yaml` 확인.
2. conflict가 있으면 사용자/curator 결정을 받는다.
3. 이전 작업 상태가 필요하면 bundle이 아니라 handoff/task/capture 문서를 확인한다.

## During Work

- selected page와 lineage를 근거로 사용한다.
- raw가 필요하면 MCP query/excerpt policy를 따른다.
- Personal/Reference 내용이 Project/Team truth와 충돌하면 Project/Team을 조용히 덮어쓰지 않는다.

## After Work

- 실패/교훈/실험 결과처럼 재사용 가치가 있는 내용은 명시적으로 Personal capture candidate로 기록한다.
- Project/Team 반영이 필요하면 promotion package를 작성하고 validate한다.
