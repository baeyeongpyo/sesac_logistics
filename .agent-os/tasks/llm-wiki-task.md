# llm-wiki-core Task Protocol

`llm-wiki-core/hooks/pre-bundle-validate.sh`와 `llm-wiki-core/hooks/session-start.sh`는 `SessionStart` hook이 자동 실행한다.

## Commands

```bash
llm-wiki-core/scripts/llm-wiki-core --root . search-pages "query"
llm-wiki-core/scripts/llm-wiki-core --root . get-lineage llm-wiki/concepts/example.md
llm-wiki-core/scripts/llm-wiki-core validate-promotion promotion-package.yaml
```

## Bundle Artifacts

Bundle artifacts are derived wiki-context snapshots. They are not previous
conversation transcripts and they are not canonical wiki truth.

```text
.agent-harness/bundles/<run-id>/
  context_bundle.md
  dependency_manifest.yaml
  source_binding_order.yaml
  selected_pages.yaml
  selected_sources.yaml
  page_hashes.yaml
  source_lineage.yaml
  access_decisions.yaml
  conflict_warnings.yaml
  warnings.yaml
  score_breakdown.json
```

Generated bundle retention keeps the newest 10 `run-YYYYMMDD-HHMMSS`
directories and protects at least 3. Explicit `--output` directories are
caller-managed.
