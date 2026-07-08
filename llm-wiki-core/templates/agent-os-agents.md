# Agent OS Agent Instructions

This file is the pasteable `agents.md`/`AGENTS.md` instruction body for projects that use `llm-wiki-core` plus the optional Agent OS overlay.

It wraps the core `llm-wiki-core` instructions. Read the core entrypoint first, then read the Agent OS overlay files.

## 1. Read llm-wiki-core first

Before applying Agent OS workflow rules, read and follow:

```text
llm-wiki-core/templates/wiki-core-agents.md
```

That file owns the llm-wiki-core read order, SessionStart hook behavior, context bundle protocol, and Project/Team wiki boundaries. Do not restate or override those rules here.

If that file is missing, ask the user to restore or paste `llm-wiki-core/templates/wiki-core-agents.md` before continuing project work.

## 2. Then read Agent OS overlay

After the core instructions, read these Agent OS / harness files when they exist:

```text
docs/03-agent-os-and-harness.md
.agent-os/README.md
.agent-os/roles/default.md
.agent-os/tasks/llm-wiki-task.md
llm-wiki-core/skills/software-development/agent-os-wiki-bundle/SKILL.md
```

## 3. Execution protocol

Follow the context bundle protocol from `llm-wiki-core/templates/wiki-core-agents.md`.

Agent OS adds only the task/role overlay: use Agent OS files to decide how to structure the work, but use the llm-wiki-core bundle as the project knowledge source.

The bundle is not previous-session conversation memory. Use Agent OS task files,
handoff notes, or capture artifacts for work-continuation history; use the
bundle for selected wiki knowledge, warnings, conflicts, and lineage.

## 4. Boundary

Agent OS is a task/role lens and execution overlay. It must consume `llm-wiki-core` context bundles and must not own, override, or directly mutate Project/Team wiki truth.
