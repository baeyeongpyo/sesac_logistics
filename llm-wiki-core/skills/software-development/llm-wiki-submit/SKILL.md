---
name: llm-wiki-submit
description: Use when preparing, formatting, validating, and submitting a promotion package from a personal/local workspace to the project or team incoming review queue.
version: 1.0.0
author: Antigravity Agent
license: MIT
metadata:
  hermes:
    tags: [llm-wiki, submit, promotion-package, personal-wiki]
    related_skills: [llm-wiki-promotion, agent-os-wiki-bundle]
---

# LLM Wiki Submit

## Overview

개인 노구나 작업 공간의 지식을 프로젝트(Project) 또는 팀(Team) 공식 위키에 영구히 저장하려면 파일에 직접 작성하지 않고 **Promotion Package** 형식을 갖추어 수동 검토용 큐에 제출(Submit)해야 합니다. 이 스킬은 승격 패키지 생성과 검증 규칙을 제공합니다.

## When to Use

- 개인 실험 로그, 학습 노트 또는 로컬 디버깅 메모에서 검증된 핵심 교훈이나 ADR을 프로젝트 표준 위키로 반영하고 싶을 때
- 로컬에서 작성한 승격 요청서의 스키마와 규칙이 맞는지 제출 전 검증할 때

## Required Flow

1. **Claim Extraction & Structuring**:
   - 승격 대상 소스에서 일반적인 배경 지식은 배제하고, 핵심 주장(Claim), 근거(Evidence), 출처(Lineage)만 정리합니다.
   
2. **Draft Promotion Package**:
   - `promotion_package` YAML 스키마에 따라 필수 정보를 빠짐없이 명시한 패키지 파일을 작성합니다.
   - `promotion_package`는 target artifact를 지정하지 않습니다. submit 단계도 target artifact를 알지 않으며, queue 처리자가 별도 프로세스에서 외부 artifact wiki로 반영합니다.
   - 정제된 페이지를 검토할 수 있도록 `refined_pages`를 포함합니다.
   - raw 파일을 함께 올리는 submit은 `raw_transfer_policy: raw_copy`와 `raw_items`를 사용합니다.
   - `none`, `excerpt`, `source_vault_ref` 정책에서는 raw 파일 복사본이 없어도 submit할 수 있으며, raw 근거는 policy에 맞게 evidence digest, excerpt, external source reference로 설명합니다.

3. **Format Validation**:
   - `validate-promotion` 명령어를 사용해 패키지 오류를 사전에 잡습니다.
   
4. **Queue Submission**:
   - 유효성이 입증되면 `submit-promotion <package>`로 `llm-wiki-promotion-queue/<timestamp>-<package>/`에 staging합니다.
   - submit은 현재 repo의 `wiki_stack.yaml`이나 `llm-wiki/`를 읽지 않고, promotion package 폴더 안의 `files/**`만 검증합니다.
   - queue에 복사되는 `promotion_package`에는 `submitted_at`이 자동으로 추가되어 언제 올린 package인지 확인할 수 있습니다.

## Commands

```bash
# 작성한 승격 패키지 YAML 파일 규격 검증
llm-wiki-core/scripts/llm-wiki-core validate-promotion <패키지파일명>.yaml

# raw/refined 파일이 들어 있는 promotion package를 queue에 staging
llm-wiki-core/scripts/llm-wiki-core --root . submit-promotion <패키지파일명>.yaml
```

기본 output:

```text
llm-wiki-promotion-queue/
  20260708-153000-promotion/
    promotion.yaml
    submission.yaml
    files/
      raw/...
      sources/...
```

### 필수 YAML 스키마 규격
제출용 YAML 파일은 반드시 아래의 골격을 유지해야 합니다.
```yaml
promotion_package:
  source_owner: "bae"
  claims:
    - claim_id: "claim-001"
      content: "휠제어 모터의 최적 기어 비율은 10:1이다."
      target_section: "architecture/wheel-control"
  evidence_digest:
    - "2026-05-14 테스트 런에서 10:1 적용 시 누적 오차 최소화 확인"
  lineage:
    - page_path: "personal/bae/wheel-control-validation.md"
      sha256: "abcdef1234567890..."
  confidence: high
  requested_target_pages:
    - "architecture/wheel-control.md"
  raw_transfer_policy: raw_copy
  raw_items:
    - pack_path: "files/raw/wheel-control-test.md"
      target_path: "raw/wheel-control-test.md"
      sha256: "raw-file-sha256..."
  refined_pages:
    - pack_path: "files/sources/wheel-control-validation.md"
      target_path: "sources/wheel-control-validation.md"
      sha256: "refined-page-sha256..."
  reviewer_required: true
```

## Common Pitfalls

1. **승인이 접수로 오해**: 패키지를 `submit`한 상태는 canonical 위키에 병합된 상태(`accepted`/`active`)가 아닙니다. 큐레이터의 최종 심사를 거쳐 다시 아티팩트 배포가 될 때까지 기다려야 합니다.
2. **필수 필드 누락**: `reviewer_required` 필드가 `true`가 아니거나 필수 검토 필드가 누락되면 검증 단계에서 블락됩니다.
3. **promotion/submit에 target 지정**: `target_dependency_id`는 package에도 submit 명령에도 넣지 않습니다. queue 처리자가 별도 프로세스에서 대상 artifact wiki를 결정합니다.
4. **raw policy 혼동**: `raw_copy`에서는 `raw_items`가 필수입니다. `none`, `excerpt`, `source_vault_ref`에서는 raw 파일 복사본 없이 refined page만 staging될 수 있습니다.

## Verification Checklist

- [ ] 패키지 내 `confidence`가 `high`, `medium`, `low` 중 하나인가?
- [ ] `reviewer_required: true` 항목이 명확히 포함되어 있는가?
- [ ] `target_dependency_id`, `target_artifact_ref` 같은 target 필드가 promotion package 안에 없는가?
- [ ] `refined_pages`가 있고 각 항목의 `sha256`이 실제 파일과 일치하는가?
- [ ] `raw_transfer_policy: raw_copy`라면 `raw_items`가 있고 각 항목의 `sha256`이 실제 파일과 일치하는가?
- [ ] `validate-promotion` 명령어로 에러 메세지가 없는지 검증했는가?
- [ ] `submit-promotion <package>`가 package 내부 `files/**`의 raw/refined 파일을 staging했는가?
- [ ] queue의 `promotion.yaml`과 `submission.yaml`에 같은 `submitted_at`이 기록되어 있는가?
