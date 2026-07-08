---
name: llm-wiki-promotion
description: Use when auditing, reviewing, verifying, and promoting submitted promotion packages to official Project/Team Wiki canonical truth.
version: 1.0.0
author: Antigravity Agent
license: MIT
metadata:
  hermes:
    tags: [llm-wiki, promotion, curator-workflow, claim-verdict]
    related_skills: [llm-wiki-submit, agent-os-wiki-bundle]
---

# LLM Wiki Promotion

## Overview

제출된 **Promotion Package**는 큐레이터(Curator) 또는 검토 에이전트의 심사(Review)를 거쳐 정식 반영(Accept) 여부가 결정됩니다. Promotion package와 submit manifest는 target artifact를 지정하지 않습니다. 큐레이터 또는 별도 처리 프로세스가 대상 artifact wiki를 결정하고 반영합니다. 이 스킬은 큐레이터 관점에서 승격 제안을 심사하고, 최종적으로 지식을 정식 프로젝트/팀 아티팩트에 반영하는 큐레이터 워크플로우를 정의합니다.

## When to Use

- 수신함에 등록된 target-free `promotion_package`와 포함된 raw/refined 파일을 리뷰 및 승인 처리할 때
- 여러 날짜에 걸친 개인 실험 데이터의 모순을 감지하고 Timeline으로 결합할 때
- 최종 승인된 문서를 위키 아티팩트로 배포(Artifact Publish)하기 위해 로컬 소스를 업데이트할 때

## Required Flow

1. **Phase A: Source Selection Proposal**:
   - package 폴더의 `submission.yaml`, `raw_transfer_policy`, `raw_items`, `refined_pages`를 검토하여 raw 근거 처리 방식과 정제 결과가 일치하는지 확인합니다.
   - 승격 요청에 명시된 원본 파일 목록을 검토하여 Ingest 대상 소스를 선별하고 사용자(또는 메인 승인자)에게 `Source Selection Proposal`로 승인을 받습니다.
   
2. **Phase B: Promotion Draft & Claim Classification**:
   - 소스에서 추출된 Claim(주장)들을 다음 범주로 분류합니다:
     * `accepted_candidate`: 정식 위키로 편입 가능
     * `needs_verification`: 추가 검증 필요 (별도 테이블로 기록)
     * `background_only`: 일반 배경 정보 (생략 또는 cite만 유지)
     * `excluded`: 편입 제외 대상
   - 심사용 `Promotion Draft` 및 `promotion candidate YAML`을 작성하여 최종 결정을 위한 승인을 요청합니다.

3. **Phase C: Apply Promotion**:
   - 최종 승인 시, 승인자가 선택한 대상 artifact 폴더에 raw 파일과 정제 페이지를 생성하거나 업데이트하고, `index.md` 및 `log.md`를 갱신합니다.
   - 페이지 간 링크(`index.md`의 카탈로그 항목, 본문 중 다른 위키/raw 문서 참조 등)는 `file:///...` 같은 절대 경로 대신, 링크를 작성하는 파일 기준의 상대 경로(`concepts/foo.md`, `../raw/bar.md` 등)로 작성합니다. 이는 `llm_wiki_core.core`가 내부적으로 `rel_to()`로 페이지 경로를 프로젝트 루트 기준 상대 경로로 다루는 것과 동일한 규칙이며, 프로젝트 디렉터리가 이동/이름 변경되어도 링크가 깨지지 않도록 합니다.
   - 이후 검토가 끝난 변경 사항을 적용한 위키 런타임용 Read-only 아티팩트 스냅샷을 갱신합니다.

## Commands

```bash
# 1. 제출된 승격 패키지 규격 검사
llm-wiki-core/scripts/llm-wiki-core validate-promotion promotion-package.yaml

# 2. package 제출 manifest가 raw/refined 파일을 포함했는지 확인
llm-wiki-core/scripts/llm-wiki-core --root . submit-promotion promotion-package.yaml \
  --force

# 3. 승격 후 위키 설정 정합성(Schema 등) 검증
llm-wiki-core/scripts/llm-wiki-core --root . validate
```

## Common Pitfalls

1. **검토 없는 직접 수정**: 큐레이터 승인 및 리뷰 없이 Project/Team canonical 지식을 개인 소스 파일로 덮어쓰거나 수동 병합하면 위키 지식 오염이 발생합니다.
2. **배경지식의 무차별 승격**: 프로젝트 수행에 무관한 일반론(예: 일반 모터 종류 등)을 프로젝트 공식 의사결정 문서(Canonical Truth)에 승격시키지 마십시오.
3. **절대 경로 링크 사용**: `index.md`나 본문 링크를 `file:///Users/...`와 같은 머신 절대 경로로 작성하지 마십시오. 프로젝트 디렉터리 위치가 바뀌면 모든 링크가 깨집니다. 항상 링크를 작성하는 파일 기준의 상대 경로를 사용하십시오.

## Verification Checklist

- [ ] 승격하려는 주장의 원본 출처(Lineage) 및 해시값이 기록되었는가?
- [ ] package와 `submission.yaml`에 `raw_transfer_policy`에 맞는 raw 근거 또는 raw 파일과 정제 페이지가 포함되어 있는가?
- [ ] target artifact 정보가 promotion package나 submission metadata에 남지 않는가?
- [ ] 검증되지 않은 정보가 있는 경우 `needs_verification` 항목으로 분류했는가?
- [ ] 최종 병합 처리 후 `index.md`에 링크를 걸고 `log.md`에 변경 로그를 남겼는가?
- [ ] 새로/수정된 페이지 링크가 절대 경로(`file:///...`)가 아닌 상대 경로로 작성되었는가?
