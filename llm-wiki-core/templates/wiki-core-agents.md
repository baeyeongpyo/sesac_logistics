# llm-wiki-core Agent Instructions

This file is the pasteable `agents.md`/`AGENTS.md` instruction body for projects that use `llm-wiki-core` without requiring an Agent OS overlay.

It is an entrypoint, not the architecture source of truth. Read this file first, then read the canonical `llm-wiki-core` documents listed below.

## Read order

Before doing project work, read these files when they exist:

```text
docs/02-llm-wiki-core.md
docs/04-runtime-config-and-priority.md
docs/05-promotion-ingest-governance.md
docs/07-implemented-reference-runtime.md
llm-wiki-core/hooks/README.md
llm-wiki-core/skills/research/llm-wiki-core-environment-setup/SKILL.md
```

## Context bundle protocol

The `SessionStart` hook runs through `.claude/settings.json` in Claude Code or `.codex/hooks.json` in Codex. It already runs `llm-wiki-core/hooks/pre-bundle-validate.sh` and `llm-wiki-core/hooks/session-start.sh` automatically at the start of every session — do not run them again manually.

Hooks prepare the bundle; they do not replace the agent's read step. Before answering project-specific questions, inspect the newest bundle and use it as the first context source. Do not answer from general knowledge before checking whether the selected pages contain relevant project knowledge.

The bundle is a derived wiki-context snapshot, not a conversation transcript or
handoff log. It records selected wiki pages, warnings, conflicts, source
bindings, hashes, and lineage for the current project state. Use explicit
handoff documents or capture artifacts for previous-session work history.

Use the newest bundle under `.agent-harness/bundles/` as task context, especially:

```text
context_bundle.md
warnings.yaml
selected_pages.yaml
source_lineage.yaml
```

Default generated bundle retention keeps the newest 10
`run-YYYYMMDD-HHMMSS` directories and protects at least 3, so old derived
snapshots do not grow without bound. Explicit `--output` bundle directories are
caller-managed and are not pruned by the default retention policy.

## Boundaries

- Do not directly mutate Team/Project wiki artifacts.
- Project/Team truth changes require promotion package review and a new artifact publish.
- Do not create, validate, submit, review, or publish a promotion package unless the user explicitly requests promotion. By default, leave pending or local wiki data unpromoted.
- Do not ingest instruction/schema/generated files as normal wiki knowledge unless explicitly requested as schema/instruction context.
- Apply the mutation, promotion, artifact, conflict, and source-binding rules from the `llm-wiki-core` documents above.
