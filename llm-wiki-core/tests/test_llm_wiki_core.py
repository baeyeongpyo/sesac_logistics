import json
import hashlib
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
CORE_PKG = ROOT / "llm-wiki-core"
if str(CORE_PKG) not in sys.path:
    sys.path.insert(0, str(CORE_PKG))

from llm_wiki_core import core

CLI = ROOT / "llm-wiki-core" / "scripts" / "llm-wiki-core"
INIT = ROOT / "llm-wiki-core" / "scripts" / "init-llm-wiki.sh"
INIT_AGENT_OS = ROOT / "llm-wiki-core" / "scripts" / "init-agent-os.sh"
INIT_ARTIFACT_WIKI = ROOT / "llm-wiki-core" / "scripts" / "init-artifact-wiki.sh"
BUILD_WIKI_ARTIFACT = ROOT / "llm-wiki-core" / "scripts" / "build-wiki-artifact.sh"
SESSION_START_HOOK = ROOT / "llm-wiki-core" / "hooks" / "session-start.sh"
USER_PROMPT_HOOK = ROOT / "llm-wiki-core" / "hooks" / "user-prompt-submit-wiki-context.sh"
CODEX_HOOKS = ROOT / ".codex" / "hooks.json"
CLAUDE_SETTINGS = ROOT / ".claude" / "settings.json"



def run(cmd, cwd=None, input_text=None):
    return subprocess.run(
        cmd,
        cwd=cwd,
        input=input_text,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )


def run_unchecked(cmd, cwd=None, input_text=None):
    return subprocess.run(
        cmd,
        cwd=cwd,
        input=input_text,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def stop_hook_commands(config: dict) -> list[str]:
    commands = []
    for entry in ((config.get("hooks") or {}).get("Stop") or []):
        for hook in entry.get("hooks") or []:
            if isinstance(hook, dict) and hook.get("command"):
                commands.append(str(hook["command"]))
    return commands


class LlmWikiCoreTest(unittest.TestCase):
    def make_page_record(
        self,
        path: str,
        title: str = "Shared Concept",
        scope: str = "team",
        sha256: str = "sha-a",
        source_binding_id: str = "team-wiki",
        selected: bool = True,
    ) -> core.PageRecord:
        return core.PageRecord(
            page_id=f"id-{path}",
            path=path,
            source_binding_id=source_binding_id,
            dependency_id=source_binding_id,
            effective_scope=scope,
            authority_level="policy" if scope in {"team", "project"} else "working",
            title=title,
            status="accepted",
            tags=[],
            sources=[],
            source_hashes={},
            review_needed=False,
            stale=False,
            confidence="high",
            sha256=sha256,
            selected=selected,
            score=1.0,
            warnings=[],
            body_sha256=f"body-{sha256}",
        )

    def test_conflict_policy_helpers_preserve_same_title_record_shape(self):
        team = self.make_page_record("artifacts/team-wiki/concepts/shared.md", scope="team", sha256="team-sha")
        local = self.make_page_record(
            "llm-wiki/concepts/shared.md",
            scope="local",
            sha256="local-sha",
            source_binding_id="local-mutable-wiki",
        )

        self.assertEqual(core.conflict_title_key("Shared Concept"), "shared concept")
        self.assertTrue(core.is_conflict_governed_scope("team"))
        self.assertTrue(core.is_conflict_governed_scope("project"))
        self.assertFalse(core.is_conflict_governed_scope("local"))
        self.assertEqual(core.conflict_content_hash(team), "team-sha")
        default_policy = core.load_conflict_detection_policy({"selection_policy": {}})
        self.assertEqual(default_policy.governed_scopes, {"project", "team"})
        empty_policy = core.load_conflict_detection_policy({
            "selection_policy": {"conflict_detection": {"governed_scopes": []}}
        })
        self.assertEqual(empty_policy.governed_scopes, set())

        record = core.build_conflict_record(core.CONFLICT_TYPE_SAME_TITLE, [team, local])
        self.assertEqual(record["type"], "same_title_conflict")
        self.assertEqual(record["title"], "Shared Concept")
        self.assertEqual(record["default_action"], "ask_user")
        self.assertEqual(core.conflict_record_key(record["type"], record["title"]), "same_title_conflict:shared concept")
        self.assertNotIn("conflict_key", record)
        self.assertNotIn("policy", record)
        self.assertEqual(
            record["pages"],
            [
                {
                    "path": "artifacts/team-wiki/concepts/shared.md",
                    "scope": "team",
                    "authority": "policy",
                    "sha256": "team-sha",
                },
                {
                    "path": "llm-wiki/concepts/shared.md",
                    "scope": "local",
                    "authority": "working",
                    "sha256": "local-sha",
                },
            ],
        )

    def test_conflict_lookup_uses_policy_key_but_keeps_title_lookup_compatibility(self):
        conflict = {
            "type": "same_title_conflict",
            "title": "Shared Concept",
            "conflict_key": "same_title_conflict:shared concept",
            "pages": [],
        }

        self.assertIs(core.find_conflict_by_title([conflict], "shared concept"), conflict)
        self.assertIs(core.find_conflict_by_key([conflict], "same_title_conflict:shared concept"), conflict)
        self.assertIsNone(core.find_conflict_by_key([conflict], "same_title_conflict:missing"))

    def test_core_owned_skills_live_under_llm_wiki_core(self):
        core_skill_paths = [
            Path("research/llm-wiki-core-environment-setup/SKILL.md"),
            Path("software-development/agent-os-role-generator/SKILL.md"),
            Path("software-development/agent-os-wiki-bundle/SKILL.md"),
            Path("software-development/llm-wiki-ingest/SKILL.md"),
            Path("software-development/llm-wiki-pack/SKILL.md"),
            Path("software-development/llm-wiki-promotion/SKILL.md"),
            Path("software-development/llm-wiki-query/SKILL.md"),
            Path("software-development/llm-wiki-submit/SKILL.md"),
        ]

        for rel_path in core_skill_paths:
            self.assertTrue((ROOT / "llm-wiki-core" / "skills" / rel_path).exists(), rel_path)
            self.assertFalse((ROOT / "skills" / rel_path).exists(), rel_path)

    def test_core_skill_references_use_core_path(self):
        files = [
            ROOT / "llm-wiki-core" / "templates" / "wiki-core-agents.md",
            ROOT / "llm-wiki-core" / "templates" / "agent-os-agents.md",
            ROOT / "llm-wiki-core" / "hooks" / "session-start.sh",
            ROOT / "llm-wiki-core" / "hooks" / "pre-bundle-validate.sh",
            ROOT / "llm-wiki-core" / "scripts" / "run-tests.sh",
        ]

        root_refs = [
            "skills/research/llm-wiki-core-environment-setup/SKILL.md",
            "skills/software-development/agent-os-wiki-bundle/SKILL.md",
            "skills/software-development/llm-wiki-ingest/SKILL.md",
        ]

        for path in files:
            text = path.read_text()
            for root_ref in root_refs:
                self.assertFalse(
                    any(
                        line.strip().startswith(root_ref) or f" {root_ref}" in line
                        for line in text.splitlines()
                    ),
                    f"{path} still points at root {root_ref}",
                )

        self.assertIn("llm-wiki-core/skills/**", core.DEFAULT_EXCLUDE_GLOBS)

    def write_accepted_concept(self, project: Path, name: str = "retention.md") -> None:
        concept = project / "llm-wiki" / "concepts" / name
        concept.parent.mkdir(parents=True, exist_ok=True)
        concept.write_text(f"""---
title: {name.removesuffix(".md").replace("-", " ").title()}
status: accepted
confidence: high
---
# {name.removesuffix(".md").replace("-", " ").title()}

This concept makes the bundle generation produce selected pages.
""")

    def seed_bundle_runs(self, project: Path, count: int) -> list[Path]:
        bundle_base = project / ".agent-harness" / "bundles"
        bundle_base.mkdir(parents=True, exist_ok=True)
        runs = []
        for idx in range(count):
            run_dir = bundle_base / f"run-200001{idx + 1:02d}-000000"
            run_dir.mkdir(parents=True)
            cb = run_dir / "context_bundle.md"
            cb.write_text(f"# old bundle {idx}\n")
            old_mtime = 946684800 + idx
            os.utime(cb, (old_mtime, old_mtime))
            os.utime(run_dir, (old_mtime, old_mtime))
            runs.append(run_dir)
        return runs

    def seed_capture_candidates(self, project: Path, count: int) -> list[Path]:
        capture_base = project / ".agent-harness" / "pending-personal-captures"
        capture_base.mkdir(parents=True, exist_ok=True)
        captures = []
        for idx in range(count):
            capture = capture_base / f"200001{idx + 1:02d}-000000-old-capture-{idx}.md"
            capture.write_text(f"# old capture {idx}\n")
            old_mtime = 946684800 + idx
            os.utime(capture, (old_mtime, old_mtime))
            captures.append(capture)
        return captures

    def write_artifact_with_page(self, project: Path, arcname: str, content: str, archive_name: str = "team-wiki.tar.gz") -> Path:
        import tarfile
        source = project / "artifact_page_src.md"
        source.write_text(content)
        artifacts_dir = project / "artifacts"
        artifacts_dir.mkdir(exist_ok=True)
        archive_path = artifacts_dir / archive_name
        with tarfile.open(archive_path, "w:gz") as tar:
            tar.add(source, arcname=arcname)
        return archive_path

    def write_local_artifact_stack(self, project: Path, artifact_ref: str = "artifacts/team-wiki.tar.gz") -> None:
        stack_path = project / "wiki_stack.yaml"
        stack_path.write_text(f"""
wiki_artifacts:
  - dependency_id: team-wiki
    artifact_ref: {artifact_ref}
    namespace: team:wiki

personal_wikis: []

selection_policy:
  default_mode: active-work
  conflict_resolution:
    default_action: ask_user
    record_user_decision: true
    autonomous_allowed: false
  priority_rules:
    priority_source: yaml_order
    earlier_entry_has_higher_priority: true
    group_block_has_equal_priority: true
    priority_grants_canonical_authority: false

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: local-mutable-wiki
    - source_binding_id: team-wiki
      dependency_id: team-wiki
      dependency_type: artifact
      effective_scope: team
      authority_level: policy
""")

    def test_duplicate_candidates_report_local_primary_for_same_sources_artifact_reference(self):
        import yaml
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Duplicate Candidate Wiki"])

            local_page = project / "llm-wiki" / "sources" / "robot_spec_summary.md"
            local_page.parent.mkdir(parents=True, exist_ok=True)
            local_page.write_text("""---
title: Robot Car HW Specification Summary
status: accepted
confidence: medium
sources:
  - llm-wiki/raw/robot_spec.md
---
# Robot Car HW Specification Summary

Local working summary.
""")

            self.write_artifact_with_page(project, "sources/robot_hardware_reference.md", """---
title: Robot Hardware Reference
status: accepted
confidence: high
sources:
  - llm-wiki/raw/robot_spec.md
---
# Robot Hardware Reference

Artifact reference summary.
""")
            self.write_local_artifact_stack(project)

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            data = json.loads(bundle.stdout)
            out_dir = Path(data["output_dir"])
            duplicate_report = yaml.safe_load((out_dir / "duplicate_candidates.yaml").read_text())

            self.assertEqual(duplicate_report["schema_version"], "duplicate-candidates/v1")
            self.assertEqual(len(duplicate_report["items"]), 1)
            item = duplicate_report["items"][0]
            self.assertEqual(item["type"], "same_sources_candidate")
            self.assertEqual(item["recommendation"], "local_primary_reference_artifact")
            self.assertEqual(item["primary_candidate"], "llm-wiki/sources/robot_spec_summary.md")
            self.assertEqual(item["reference_candidates"], ["artifacts/team-wiki/sources/robot_hardware_reference.md"])
            self.assertEqual(item["selection_effect"], "none")
            self.assertEqual(item["risk"], "local_differs_from_official_reference")
            self.assertEqual(item["evidence"]["shared_sources"], ["llm-wiki/raw/robot_spec.md"])
            self.assertEqual(item["suggested_local_captures"]["decision"], "llm-wiki/decisions/duplicate-candidate-" + item["id"].removeprefix("duplicate-candidate-") + ".md")
            self.assertEqual(item["suggested_local_captures"]["experiment"], "llm-wiki/experiments/duplicate-candidate-" + item["id"].removeprefix("duplicate-candidate-") + "-local-observation.md")
            self.assertEqual(item["suggested_local_captures"]["lesson"], "llm-wiki/lessons/duplicate-candidate-" + item["id"].removeprefix("duplicate-candidate-") + ".md")
            self.assertFalse((project / "llm-wiki" / "decisions" / (item["id"] + ".md")).exists())

            warnings = yaml.safe_load((out_dir / "warnings.yaml").read_text())
            self.assertIn(
                "duplicate_candidate:same_sources_candidate primary=llm-wiki/sources/robot_spec_summary.md references=1",
                warnings,
            )
            context = (out_dir / "context_bundle.md").read_text()
            self.assertIn("## Duplicate Candidates", context)
            self.assertIn("same_sources_candidate", context)

    def test_duplicate_candidates_report_exact_duplicate_without_selection_effect(self):
        import yaml
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Exact Duplicate Wiki"])

            content = """---
title: Shared Exact Page
status: accepted
confidence: high
sources:
  - llm-wiki/raw/shared.md
---
# Shared Exact Page

Same body in local and artifact.
"""
            local_page = project / "llm-wiki" / "concepts" / "shared-exact-page.md"
            local_page.parent.mkdir(parents=True, exist_ok=True)
            local_page.write_text(content)

            self.write_artifact_with_page(project, "concepts/shared-exact-page.md", content)
            self.write_local_artifact_stack(project)

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            out_dir = Path(json.loads(bundle.stdout)["output_dir"])
            duplicate_report = yaml.safe_load((out_dir / "duplicate_candidates.yaml").read_text())

            self.assertEqual(len(duplicate_report["items"]), 1)
            item = duplicate_report["items"][0]
            self.assertEqual(item["type"], "exact_duplicate")
            self.assertEqual(item["primary_candidate"], "llm-wiki/concepts/shared-exact-page.md")
            self.assertEqual(item["reference_candidates"], ["artifacts/team-wiki/concepts/shared-exact-page.md"])
            self.assertEqual(item["selection_effect"], "none")

    def test_duplicate_candidates_exact_duplicate_uses_normalized_body_hash(self):
        import yaml
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Body Exact Duplicate Wiki"])

            local_page = project / "llm-wiki" / "concepts" / "local-page.md"
            local_page.parent.mkdir(parents=True, exist_ok=True)
            local_page.write_text("""---
title: Local Page
status: accepted
confidence: medium
---
# Shared Body

Same body content.
""")

            self.write_artifact_with_page(project, "concepts/reference-page.md", """---
title: Reference Page
status: accepted
confidence: high
---
# Shared Body

Same body content.

""")
            self.write_local_artifact_stack(project)

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            out_dir = Path(json.loads(bundle.stdout)["output_dir"])
            duplicate_report = yaml.safe_load((out_dir / "duplicate_candidates.yaml").read_text())

            self.assertEqual(len(duplicate_report["items"]), 1)
            item = duplicate_report["items"][0]
            self.assertEqual(item["type"], "exact_duplicate")
            self.assertEqual(item["primary_candidate"], "llm-wiki/concepts/local-page.md")
            self.assertEqual(item["reference_candidates"], ["artifacts/team-wiki/concepts/reference-page.md"])
            self.assertIn("body_sha256", item["evidence"])

    def test_duplicate_candidates_record_same_title_divergence_handled_by_conflict_resolution(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Same Title Duplicate Wiki"])

            local_page = project / "llm-wiki" / "concepts" / "shared-concept.md"
            local_page.parent.mkdir(parents=True, exist_ok=True)
            local_page.write_text("""---
title: Shared Concept
status: accepted
confidence: medium
---
# Shared Concept

Local working version.
""")

            self.write_artifact_with_page(project, "concepts/shared-concept.md", """---
title: Shared Concept
status: accepted
confidence: high
---
# Shared Concept

Official reference version.
""")
            self.write_local_artifact_stack(project)

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            data = json.loads(bundle.stdout)
            self.assertTrue(data["ok"])
            self.assertEqual(len(data["resolved_conflicts"]), 1)

            out_dir = Path(data["output_dir"])
            import yaml
            duplicate_report = yaml.safe_load((out_dir / "duplicate_candidates.yaml").read_text())
            item = duplicate_report["items"][0]
            self.assertEqual(item["type"], "same_title_divergence")
            self.assertEqual(item["selection_effect"], "handled_by_conflict_resolution")
            self.assertEqual(item["risk"], "local_artifact_same_title_divergence")
            self.assertEqual(item["primary_candidate"], "llm-wiki/concepts/shared-concept.md")
            self.assertEqual(item["reference_candidates"], ["artifacts/team-wiki/concepts/shared-concept.md"])
            self.assertIn("suggested_local_captures", item)
            self.assertTrue(item["suggested_local_captures"]["decision"].startswith("llm-wiki/decisions/"))

    def test_duplicate_candidates_normalized_title_divergence_without_conflict_resolution_effect(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Normalized Title Duplicate Wiki"])

            local_page = project / "llm-wiki" / "concepts" / "shared-concept.md"
            local_page.parent.mkdir(parents=True, exist_ok=True)
            local_page.write_text("""---
title: Shared Concept
status: accepted
confidence: medium
---
# Shared Concept

Local working version.
""")

            self.write_artifact_with_page(project, "concepts/shared-concept-ref.md", """---
title: Shared-Concept
status: accepted
confidence: high
---
# Shared-Concept

Official reference version.
""")
            self.write_local_artifact_stack(project)

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            data = json.loads(bundle.stdout)
            self.assertTrue(data["ok"])
            self.assertEqual(data["resolved_conflicts"], [])

            out_dir = Path(data["output_dir"])
            import yaml
            duplicate_report = yaml.safe_load((out_dir / "duplicate_candidates.yaml").read_text())
            item = duplicate_report["items"][0]
            self.assertEqual(item["type"], "same_title_divergence")
            self.assertEqual(item["selection_effect"], "none")
            self.assertEqual(item["primary_candidate"], "llm-wiki/concepts/shared-concept.md")
            self.assertEqual(item["reference_candidates"], ["artifacts/team-wiki/concepts/shared-concept-ref.md"])

    def test_duplicate_candidates_ignore_status_excluded_local_pages(self):
        import yaml
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Experimental Local Duplicate Wiki"])

            local_page = project / "llm-wiki" / "sources" / "experimental-summary.md"
            local_page.parent.mkdir(parents=True, exist_ok=True)
            local_page.write_text("""---
title: Experimental Summary
status: experimental
sources:
  - llm-wiki/raw/shared.md
---
# Experimental Summary

Local experimental observation.
""")

            self.write_artifact_with_page(project, "sources/official-summary.md", """---
title: Official Summary
status: accepted
sources:
  - llm-wiki/raw/shared.md
---
# Official Summary

Official reference.
""")
            self.write_local_artifact_stack(project)

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            out_dir = Path(json.loads(bundle.stdout)["output_dir"])
            duplicate_report = yaml.safe_load((out_dir / "duplicate_candidates.yaml").read_text())
            self.assertEqual(duplicate_report["items"], [])

    def test_duplicate_candidates_ignore_local_only_duplicates(self):
        import yaml
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Local Only Duplicate Wiki"])

            first = project / "llm-wiki" / "sources" / "first.md"
            second = project / "llm-wiki" / "sources" / "second.md"
            first.parent.mkdir(parents=True, exist_ok=True)
            first.write_text("""---
title: First Local
status: accepted
sources:
  - llm-wiki/raw/shared.md
---
# First Local
""")
            second.write_text("""---
title: Second Local
status: accepted
sources:
  - llm-wiki/raw/shared.md
---
# Second Local
""")

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            out_dir = Path(json.loads(bundle.stdout)["output_dir"])
            duplicate_report = yaml.safe_load((out_dir / "duplicate_candidates.yaml").read_text())
            self.assertEqual(duplicate_report["items"], [])

    def test_duplicate_candidates_do_not_promote_external_personal_wiki_to_local_primary(self):
        import yaml
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            personal = project / "external-personal-wiki"
            personal_page = personal / "sources" / "personal-summary.md"
            personal_page.parent.mkdir(parents=True, exist_ok=True)
            personal_page.write_text("""---
title: Personal Summary
status: accepted
sources:
  - llm-wiki/raw/shared.md
---
# Personal Summary

External personal note.
""")

            run([str(INIT), "--dest", str(project), "--domain", "External Personal Duplicate Wiki"])
            self.write_artifact_with_page(project, "sources/official-summary.md", """---
title: Official Summary
status: accepted
sources:
  - llm-wiki/raw/shared.md
---
# Official Summary

Official reference.
""")

            stack_path = project / "wiki_stack.yaml"
            stack_path.write_text(f"""
wiki_artifacts:
  - dependency_id: team-wiki
    artifact_ref: artifacts/team-wiki.tar.gz
    namespace: team:wiki

personal_wikis:
  - dependency_id: external-personal
    path: {personal}
    namespace: personal:external

selection_policy:
  default_mode: active-work
  conflict_resolution:
    default_action: ask_user
    record_user_decision: true
    autonomous_allowed: false
  priority_rules:
    priority_source: yaml_order
    earlier_entry_has_higher_priority: true
    group_block_has_equal_priority: true
    priority_grants_canonical_authority: false

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: external-personal
      dependency_id: external-personal
      dependency_type: personal_wiki
      effective_scope: personal
      authority_level: working
    - source_binding_id: team-wiki
      dependency_id: team-wiki
      dependency_type: artifact
      effective_scope: team
      authority_level: policy
""")

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            out_dir = Path(json.loads(bundle.stdout)["output_dir"])
            duplicate_report = yaml.safe_load((out_dir / "duplicate_candidates.yaml").read_text())
            self.assertEqual(duplicate_report["items"], [])

    def test_get_context_bundle_regenerates_cached_bundle_missing_duplicate_report(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Legacy Cached Bundle Wiki"])
            self.write_accepted_concept(project)

            first_bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            first_data = json.loads(first_bundle.stdout)
            first_dir = Path(first_data["output_dir"])
            duplicate_path = first_dir / "duplicate_candidates.yaml"
            self.assertTrue(duplicate_path.exists())
            duplicate_path.unlink()

            second_bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            second_data = json.loads(second_bundle.stdout)
            second_dir = Path(second_data["output_dir"])

            self.assertTrue((second_dir / "duplicate_candidates.yaml").exists())
            self.assertIn("duplicate_candidate_count", second_data)

    def test_get_context_bundle_prunes_default_runs_to_newest_ten(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Bundle Retention Wiki"])
            seeded = self.seed_bundle_runs(project, 12)
            non_matching = project / ".agent-harness" / "bundles" / "run-not-a-timestamp"
            non_matching.mkdir()

            self.write_accepted_concept(project)

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            bundle_data = json.loads(bundle.stdout)
            self.assertTrue(bundle_data["ok"])

            active = Path(bundle_data["output_dir"])
            runs = sorted([
                d for d in (project / ".agent-harness" / "bundles").iterdir()
                if d.is_dir() and d.name.startswith("run-") and d.name != "run-not-a-timestamp"
            ], key=lambda p: p.name)

            self.assertEqual(len(runs), 10)
            self.assertIn(active.resolve(), {run_dir.resolve() for run_dir in runs})
            self.assertFalse(seeded[0].exists())
            self.assertFalse(seeded[1].exists())
            self.assertFalse(seeded[2].exists())
            self.assertTrue(seeded[-1].exists())
            self.assertTrue(non_matching.exists())

    def test_get_context_bundle_retention_never_drops_below_three(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Minimum Retention Wiki"])
            harness_config = project / ".agent-harness" / "config.yaml"
            harness_config.parent.mkdir(parents=True, exist_ok=True)
            harness_config.write_text("""
llm_wiki_core:
  bundle_retention_count: 1
""")
            seeded = self.seed_bundle_runs(project, 5)

            self.write_accepted_concept(project)

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            bundle_data = json.loads(bundle.stdout)
            self.assertTrue(bundle_data["ok"])

            active = Path(bundle_data["output_dir"])
            runs = sorted([
                d for d in (project / ".agent-harness" / "bundles").iterdir()
                if d.is_dir() and d.name.startswith("run-")
            ], key=lambda p: p.name)

            self.assertEqual(len(runs), 3)
            self.assertIn(active.resolve(), {run_dir.resolve() for run_dir in runs})
            self.assertFalse(seeded[0].exists())
            self.assertFalse(seeded[1].exists())
            self.assertFalse(seeded[2].exists())
            self.assertTrue(seeded[-1].exists())

    def test_get_context_bundle_explicit_output_does_not_prune_default_runs(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Explicit Output Wiki"])
            seeded = self.seed_bundle_runs(project, 12)

            self.write_accepted_concept(project)

            explicit = project / "manual-bundle"
            bundle = run([str(CLI), "--root", str(project), "get-context-bundle", "--output", str(explicit)])
            bundle_data = json.loads(bundle.stdout)
            self.assertTrue(bundle_data["ok"])
            self.assertEqual(Path(bundle_data["output_dir"]), explicit)

            for run_dir in seeded:
                self.assertTrue(run_dir.exists())

    def test_capture_run_prunes_pending_personal_captures_to_newest_fifty(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Capture Retention Wiki"])
            seeded = self.seed_capture_candidates(project, 52)
            non_matching = project / ".agent-harness" / "pending-personal-captures" / "manual-note.md"
            non_matching.write_text("# manual note\n")

            cap = run([str(CLI), "--root", str(project), "capture-run", "--title", "Newest Capture", "--body", "body"])
            cap_path = Path(json.loads(cap.stdout)["path"])
            self.assertTrue(cap_path.exists())

            captures = sorted([
                p for p in (project / ".agent-harness" / "pending-personal-captures").iterdir()
                if p.is_file() and p.name != "manual-note.md"
            ], key=lambda p: p.name)

            self.assertEqual(len(captures), 50)
            self.assertIn(cap_path.resolve(), {p.resolve() for p in captures})
            self.assertFalse(seeded[0].exists())
            self.assertFalse(seeded[1].exists())
            self.assertFalse(seeded[2].exists())
            self.assertTrue(seeded[-1].exists())
            self.assertTrue(non_matching.exists())

    def test_capture_run_retention_never_drops_below_ten(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Minimum Capture Retention Wiki"])
            harness_config = project / ".agent-harness" / "config.yaml"
            harness_config.parent.mkdir(parents=True, exist_ok=True)
            harness_config.write_text("""
llm_wiki_core:
  pending_capture_retention_count: 1
""")
            seeded = self.seed_capture_candidates(project, 12)

            cap = run([str(CLI), "--root", str(project), "capture-run", "--title", "Newest Capture", "--body", "body"])
            cap_path = Path(json.loads(cap.stdout)["path"])
            self.assertTrue(cap_path.exists())

            captures = sorted([
                p for p in (project / ".agent-harness" / "pending-personal-captures").iterdir()
                if p.is_file()
            ], key=lambda p: p.name)

            self.assertEqual(len(captures), 10)
            self.assertIn(cap_path.resolve(), {p.resolve() for p in captures})
            self.assertFalse(seeded[0].exists())
            self.assertFalse(seeded[1].exists())
            self.assertFalse(seeded[2].exists())
            self.assertTrue(seeded[-1].exists())

    def test_local_mutable_wiki_binding_defaults_are_valid(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Local Binding Defaults Wiki"])

            stack_path = project / "wiki_stack.yaml"
            stack_path.write_text("""
wiki_artifacts: []
personal_wikis: []

selection_policy:
  default_mode: active-work
  conflict_resolution:
    default_action: ask_user
    record_user_decision: true
    autonomous_allowed: false
  priority_rules:
    priority_source: yaml_order
    earlier_entry_has_higher_priority: true
    group_block_has_equal_priority: true
    priority_grants_canonical_authority: false

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: local-mutable-wiki
""")

            validate = run([str(CLI), "--root", str(project), "validate"])
            validate_data = json.loads(validate.stdout)
            self.assertTrue(validate_data["ok"])
            self.assertEqual(validate_data["dependency_ids"], [])
            self.assertEqual(validate_data["source_bindings"], [{
                "source_binding_id": "local-mutable-wiki",
                "dependency_id": "local-mutable-wiki",
                "dependency_type": "local_wiki",
                "effective_scope": "local",
                "authority_level": "working",
                "tier": 0,
                "group_index": None,
            }])

    def test_explicit_local_mutable_wiki_binding_collects_local_pages_once(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Explicit Local Wiki"])

            stack_path = project / "wiki_stack.yaml"
            stack_path.write_text("""
wiki_artifacts: []
personal_wikis: []

selection_policy:
  default_mode: active-work
  conflict_resolution:
    default_action: ask_user
    record_user_decision: true
    autonomous_allowed: false
  priority_rules:
    priority_source: yaml_order
    earlier_entry_has_higher_priority: true
    group_block_has_equal_priority: true
    priority_grants_canonical_authority: false

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: local-mutable-wiki
""")

            concept = project / "llm-wiki" / "concepts" / "explicit-local.md"
            concept.write_text("""---
title: Explicit Local Concept
status: accepted
confidence: high
---
# Explicit Local Concept

This local page should be collected exactly once.
""")

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            bundle_data = json.loads(bundle.stdout)
            self.assertTrue(bundle_data["ok"])
            self.assertEqual(bundle_data["selected_page_count"], 1)
            self.assertEqual(bundle_data["page_count"], 1)

            out_dir = Path(bundle_data["output_dir"])
            import yaml
            selected_pages = yaml.safe_load((out_dir / "selected_pages.yaml").read_text())
            self.assertEqual(len(selected_pages), 1)
            self.assertEqual(selected_pages[0]["source_binding_id"], "local-mutable-wiki")
            self.assertEqual(selected_pages[0]["effective_scope"], "local")
            self.assertEqual(selected_pages[0]["authority_level"], "working")

            search = run([str(CLI), "--root", str(project), "search-pages", "collected exactly once"])
            search_data = json.loads(search.stdout)
            self.assertEqual(search_data["count"], 1)
            self.assertEqual(search_data["matches"][0]["source_binding_id"], "local-mutable-wiki")

    def test_init_scaffold_uses_explicit_local_mutable_wiki_without_personal_registration(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Scaffold Local Wiki"])

            stack_text = (project / "wiki_stack.yaml").read_text()
            self.assertIn("personal_wikis: []", stack_text)
            self.assertIn("source_binding_id: local-mutable-wiki", stack_text)
            self.assertNotIn("dependency_id: personal-wiki", stack_text)
            self.assertFalse((project / "llm-wiki-core" / "wiki_stack.yaml").exists())
            self.assertTrue((project / "llm-wiki-core" / "hooks" / "session-start.sh").exists())
            self.assertTrue((project / "llm-wiki-core" / "hooks" / "user-prompt-submit-wiki-context.sh").exists())
            self.assertTrue((project / "llm-wiki-core" / "templates" / "wiki-core-agents.md").exists())

            validate = run([str(CLI), "--root", str(project), "validate"])
            validate_data = json.loads(validate.stdout)
            self.assertTrue(validate_data["ok"])
            self.assertEqual(validate_data["source_bindings"][0]["source_binding_id"], "local-mutable-wiki")

    def test_init_artifact_wiki_creates_empty_folder_artifact(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            artifact = project / "wiki-artifacts" / "new-team-wiki"

            result = run([
                str(INIT_ARTIFACT_WIKI),
                "--dest",
                str(artifact),
                "--version",
                "1.0.0",
                "--title",
                "New Team Wiki",
            ])

            self.assertIn("created artifact wiki", result.stdout)
            self.assertNotIn("wiki_stack.yaml snippet", result.stdout)
            self.assertNotIn("namespace:", result.stdout)
            self.assertTrue((artifact / "artifact.yaml").exists())
            self.assertTrue((artifact / "index.md").exists())
            self.assertTrue((artifact / "log.md").exists())
            for folder in ["raw", "sources", "concepts", "decisions", "comparisons", "queries", "metadata"]:
                self.assertTrue((artifact / folder).is_dir(), folder)

            artifact_meta = yaml.safe_load((artifact / "artifact.yaml").read_text())
            self.assertEqual(artifact_meta["version"], "1.0.0")
            self.assertNotIn("format", artifact_meta)
            self.assertNotIn("namespace", artifact_meta)

            stack = project / "wiki_stack.yaml"
            stack.write_text(f"""
wiki_artifacts:
  - dependency_id: new-team-wiki
    artifact_ref: {artifact}
    namespace: new-team:wiki

personal_wikis: []

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: new-team-wiki
      dependency_id: new-team-wiki
      dependency_type: artifact
      effective_scope: team
      authority_level: policy
""")

            validate = run([str(CLI), "--root", str(project), "validate"])
            self.assertTrue(json.loads(validate.stdout)["ok"])

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            bundle_data = json.loads(bundle.stdout)
            self.assertTrue(bundle_data["ok"])
            self.assertEqual(bundle_data["warnings"], [])
            self.assertEqual(bundle_data["selected_page_count"], 0)

    def test_init_artifact_wiki_rejects_namespace_option(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            artifact = project / "wiki-artifacts" / "new-team-wiki"

            result = run_unchecked([
                str(INIT_ARTIFACT_WIKI),
                "--dest",
                str(artifact),
                "--namespace",
                "new-team:wiki",
            ])

            self.assertEqual(result.returncode, 2)
            self.assertIn("Unknown option: --namespace", result.stderr)
            self.assertFalse(artifact.exists())

    def test_root_wiki_stack_takes_precedence_over_legacy_core_stack(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            (project / "llm-wiki-core").mkdir()
            (project / "wiki_stack.yaml").write_text("""
wiki_artifacts: []
personal_wikis: []
mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: local-mutable-wiki
""")
            (project / "llm-wiki-core" / "wiki_stack.yaml").write_text("""
wiki_artifacts: []
personal_wikis: []
mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: legacy-artifact
      dependency_id: missing-legacy-artifact
      dependency_type: artifact
""")

            result = core.validate_stack(project)

            self.assertTrue(result["ok"], result["errors"])
            self.assertEqual(result["stack_path"], str(project / "wiki_stack.yaml"))
            self.assertEqual(result["source_bindings"][0]["source_binding_id"], "local-mutable-wiki")

    def test_init_scaffold_is_idempotent_when_run_from_project_copy(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Self Init Wiki"])

            project_init = project / "llm-wiki-core" / "scripts" / "init-llm-wiki.sh"
            rerun = run_unchecked([str(project_init), "--dest", str(project)])

            self.assertEqual(rerun.returncode, 0, rerun.stderr)
            self.assertTrue((project / "llm-wiki-core" / "llm_wiki_core" / "core.py").exists())
            self.assertTrue((project / "llm-wiki-core" / "scripts" / "llm-wiki-core").exists())

    def test_validate_and_bundle_and_search(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Test Wiki"])
            concept = project / "llm-wiki" / "concepts" / "context-bundle.md"
            concept.write_text("""---
title: Context Bundle
status: accepted
tags: [bundle, llm-wiki-core]
sources: [raw/example.md]
source_hashes:
  raw/example.md: abc
confidence: high
---
# Context Bundle

A context bundle is a derived run artifact.
""")
            validate = run([str(CLI), "--root", str(project), "validate"])
            self.assertTrue(json.loads(validate.stdout)["ok"])

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            bundle_data = json.loads(bundle.stdout)
            self.assertGreaterEqual(bundle_data["selected_page_count"], 1)
            out_dir = Path(bundle_data["output_dir"])
            self.assertTrue((out_dir / "context_bundle.md").exists())
            self.assertTrue((out_dir / "source_binding_order.yaml").exists())

            search = run([str(CLI), "--root", str(project), "search-pages", "context bundle"])
            self.assertGreaterEqual(json.loads(search.stdout)["count"], 1)

            lineage = run([str(CLI), "--root", str(project), "get-lineage", "llm-wiki/concepts/context-bundle.md"])
            self.assertEqual(json.loads(lineage.stdout)["confidence"], "high")

    def test_session_start_hook_prints_wiki_context_injection_summary(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Hook Injection Test Wiki"])
            concept = project / "llm-wiki" / "concepts" / "hook-injection.md"
            concept.write_text("""---
title: Hook Injection Concept
status: accepted
tags: [hook, context]
sources: [raw/hook.md]
confidence: high
---
# Hook Injection Concept

This concept should be visible in the session-start hook summary.
""")

            hook = run([str(SESSION_START_HOOK), str(project)])

            self.assertIn("Wiki context injection summary:", hook.stdout)
            self.assertIn("Before answering project questions, inspect relevant selected wiki pages.", hook.stdout)
            self.assertIn("context_bundle.md", hook.stdout)
            self.assertIn("selected_pages.yaml", hook.stdout)
            self.assertIn("source_lineage.yaml", hook.stdout)
            self.assertIn("Hook Injection Concept", hook.stdout)
            self.assertIn("llm-wiki/concepts/hook-injection.md", hook.stdout)

    def test_user_prompt_hook_injects_bundle_context_for_codex_and_claude(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Prompt Hook Test Wiki"])
            concept = project / "llm-wiki" / "concepts" / "prompt-hook.md"
            concept.write_text("""---
title: Prompt Hook Concept
status: accepted
tags: [hook, prompt]
sources: [raw/prompt.md]
confidence: high
---
# Prompt Hook Concept

This concept should be visible in prompt hook context.
""")

            codex = run([str(USER_PROMPT_HOOK), "--agent", "codex", "--root", str(project)])
            self.assertIn("llm-wiki-core context reminder:", codex.stdout)
            self.assertIn("context_bundle.md", codex.stdout)
            self.assertIn("Prompt Hook Concept", codex.stdout)
            self.assertIn("llm-wiki/concepts/prompt-hook.md", codex.stdout)

            claude = run([str(USER_PROMPT_HOOK), "--agent", "claude", "--root", str(project)])
            payload = json.loads(claude.stdout)
            hook_output = payload["hookSpecificOutput"]
            self.assertEqual(hook_output["hookEventName"], "UserPromptSubmit")
            self.assertIn("llm-wiki-core context reminder:", hook_output["additionalContext"])
            self.assertIn("Prompt Hook Concept", hook_output["additionalContext"])

    def test_stop_hooks_do_not_create_automatic_personal_captures(self):
        codex = json.loads(CODEX_HOOKS.read_text())
        claude = json.loads(CLAUDE_SETTINGS.read_text())

        for command in stop_hook_commands(codex) + stop_hook_commands(claude):
            self.assertNotIn("post-run-capture.sh", command)
            self.assertNotIn("capture-run", command)

    def test_agent_hook_configs_use_core_owned_hook_paths(self):
        codex = CODEX_HOOKS.read_text()
        claude = CLAUDE_SETTINGS.read_text()

        self.assertIn("llm-wiki-core/hooks/session-start.sh", codex)
        self.assertIn("llm-wiki-core/hooks/user-prompt-submit-wiki-context.sh", codex)
        self.assertIn("llm-wiki-core/hooks/session-start.sh", claude)
        self.assertIn("llm-wiki-core/hooks/user-prompt-submit-wiki-context.sh", claude)
        self.assertNotIn("$ROOT/hooks/", codex)
        self.assertNotIn("CLAUDE_PROJECT_DIR}/hooks/", claude)

    def test_promotion_validation_and_capture(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project)])
            pkg = project / "promotion.yaml"
            pkg.write_text("""promotion_package:
  source_owner: bae
  claims: []
  evidence_digest: []
  lineage: []
  confidence: medium
  requested_target_pages: []
  raw_transfer_policy: none
  reviewer_required: true
""")
            promo = run([str(CLI), "validate-promotion", str(pkg)])
            self.assertTrue(json.loads(promo.stdout)["ok"])

            cap = run([str(CLI), "--root", str(project), "capture-run", "--title", "Test Capture", "--body", "body"])
            cap_path = Path(json.loads(cap.stdout)["path"])
            self.assertTrue(cap_path.exists())

    def test_promotion_package_rejects_target_artifact_fields(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project)])
            pkg = project / "promotion.yaml"
            pkg.write_text("""promotion_package:
  target_scope: team
  source_owner: bae
  claims: []
  evidence_digest: []
  lineage: []
  confidence: medium
  requested_target_pages: []
  raw_transfer_policy: raw_copy
  reviewer_required: true
""")

            promo = run_unchecked([str(CLI), "validate-promotion", str(pkg)])
            self.assertEqual(promo.returncode, 2)
            self.assertIn("target_scope belongs to submit, not promotion_package", promo.stdout)

    def test_submit_promotion_stages_raw_and_refined_files_for_target_artifact(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            pack = project / "promotion-pack"
            raw = pack / "files" / "raw" / "example_source.md"
            raw.parent.mkdir(parents=True)
            raw.write_text("# Raw Source\n\nOriginal evidence.\n")
            refined = pack / "files" / "sources" / "example_source_summary.md"
            refined.parent.mkdir(parents=True)
            refined.write_text("""---
title: Example Source Summary
status: accepted
confidence: high
sources:
  - llm-wiki/raw/example_source.md
---
# Example Source Summary

Curated evidence.
""")
            raw_sha = hashlib.sha256(raw.read_bytes()).hexdigest()
            refined_sha = hashlib.sha256(refined.read_bytes()).hexdigest()

            pkg = pack / "promotion.yaml"
            pkg.write_text(f"""promotion_package:
  source_owner: bae
  claims:
    - claim_id: claim-001
      content: Example source summary should be promoted.
      target_section: sources/example_source_summary
  evidence_digest:
    - Raw source was summarized into a curated page.
  lineage:
    - page_path: llm-wiki/sources/example_source_summary.md
      sha256: "{refined_sha}"
  confidence: high
  requested_target_pages:
    - sources/example_source_summary.md
  raw_transfer_policy: raw_copy
  raw_items:
    - pack_path: files/raw/example_source.md
      target_path: raw/example_source.md
      sha256: "{raw_sha}"
  refined_pages:
    - pack_path: files/sources/example_source_summary.md
      target_path: sources/example_source_summary.md
      sha256: "{refined_sha}"
  reviewer_required: true
""")

            submit = run([
                str(CLI),
                "--root",
                str(project),
                "submit-promotion",
                str(pkg),
            ])
            submit_data = json.loads(submit.stdout)

            self.assertTrue(submit_data["ok"], submit_data)
            self.assertNotIn("target_dependency_id", submit_data)
            out_dir = Path(submit_data["output_dir"])
            rel_parts = out_dir.resolve().relative_to(project.resolve()).parts
            self.assertEqual(rel_parts[0], "llm-wiki-promotion-queue")
            self.assertRegex(rel_parts[1], r"^\d{8}-\d{6}-promotion$")
            self.assertEqual(len(rel_parts), 2)
            self.assertEqual((out_dir / "files" / "raw" / "example_source.md").read_text(), raw.read_text())
            self.assertEqual((out_dir / "files" / "sources" / "example_source_summary.md").read_text(), refined.read_text())
            queued_package = yaml.safe_load((out_dir / "promotion.yaml").read_text())
            self.assertRegex(queued_package["promotion_package"]["submitted_at"], r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z$")
            submission = yaml.safe_load((out_dir / "submission.yaml").read_text())
            self.assertEqual(submission["promotion_package"], "promotion.yaml")
            self.assertEqual(submission["submitted_at"], queued_package["promotion_package"]["submitted_at"])
            self.assertNotIn("target_dependency_id", submission)
            self.assertNotIn("target_artifact_ref", submission)
            self.assertNotIn("target_effective_scope", submission)
            self.assertEqual(submission["included_files"][0]["pack_path"], "files/raw/example_source.md")
            self.assertEqual(len(submission["included_files"]), 2)

    def test_submit_promotion_allows_non_copy_raw_transfer_policies_without_raw_files(self):
        for raw_transfer_policy in ["none", "excerpt", "source_vault_ref"]:
            with self.subTest(raw_transfer_policy=raw_transfer_policy):
                with tempfile.TemporaryDirectory() as td:
                    project = Path(td)
                    pack = project / "promotion-pack"
                    refined = pack / "files" / "sources" / "example_source_summary.md"
                    refined.parent.mkdir(parents=True)
                    refined.write_text("""---
title: Example Source Summary
status: accepted
confidence: high
---
# Example Source Summary

Curated evidence with raw handled by policy.
""")
                    refined_sha = hashlib.sha256(refined.read_bytes()).hexdigest()

                    pkg = pack / "promotion.yaml"
                    pkg.write_text(f"""promotion_package:
  source_owner: bae
  claims:
    - claim_id: claim-001
      content: Example source summary should be promoted.
      target_section: sources/example_source_summary
  evidence_digest:
    - Raw evidence is handled with policy {raw_transfer_policy}.
  lineage:
    - page_path: llm-wiki/sources/example_source_summary.md
      sha256: "{refined_sha}"
  confidence: high
  requested_target_pages:
    - sources/example_source_summary.md
  raw_transfer_policy: {raw_transfer_policy}
  refined_pages:
    - pack_path: files/sources/example_source_summary.md
      target_path: sources/example_source_summary.md
      sha256: "{refined_sha}"
  reviewer_required: true
""")

                    submit = run([
                        str(CLI),
                        "--root",
                        str(project),
                        "submit-promotion",
                        str(pkg),
                    ])
                    submit_data = json.loads(submit.stdout)

                    self.assertTrue(submit_data["ok"], submit_data)
                    out_dir = Path(submit_data["output_dir"])
                    self.assertFalse((out_dir / "files" / "raw").exists())
                    self.assertEqual((out_dir / "files" / "sources" / "example_source_summary.md").read_text(), refined.read_text())
                    submission = yaml.safe_load((out_dir / "submission.yaml").read_text())
                    self.assertEqual([item["kind"] for item in submission["included_files"]], ["refined"])

    def test_agent_os_init_does_not_generate_lowercase_agents_md(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT_AGENT_OS), "--dest", str(project)])
            root_names = {p.name for p in project.iterdir()}
            self.assertIn("AGENTS.md", root_names)
            self.assertNotIn("agents.md", root_names)
            self.assertIn("llm-wiki-core/templates/agent-os-agents.md", (project / "AGENTS.md").read_text())
            self.assertTrue((project / "llm-wiki-core" / "templates" / "wiki-core-agents.md").exists())
            self.assertTrue((project / "llm-wiki-core" / "templates" / "agent-os-agents.md").exists())
            self.assertTrue((project / "llm-wiki-core" / "templates" / "snippet-wiki-core-agents.txt").exists())
            self.assertTrue((project / "llm-wiki-core" / "templates" / "snippet-agent-os-agents.txt").exists())
            self.assertFalse((project / "templates").exists())

        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            existing = project / "agents.md"
            existing.write_text("existing antigravity instructions\n")
            run([str(INIT_AGENT_OS), "--dest", str(project), "--force"])
            root_names = {p.name for p in project.iterdir()}
            self.assertIn("agents.md", root_names)
            self.assertNotIn("AGENTS.md", root_names)
            self.assertEqual(existing.read_text(), "existing antigravity instructions\n")
            self.assertTrue((project / "llm-wiki-core" / "templates" / "wiki-core-agents.md").exists())
            self.assertTrue((project / "llm-wiki-core" / "templates" / "agent-os-agents.md").exists())
            self.assertFalse((project / "templates").exists())

    def test_core_only_install_can_create_agent_os_overlay(self):
        with tempfile.TemporaryDirectory() as td:
            sandbox = Path(td)
            core_src = ROOT / "llm-wiki-core"
            core_copy = sandbox / "llm-wiki-core"
            import shutil
            shutil.copytree(core_src, core_copy, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))

            project = sandbox / "project"
            run([str(core_copy / "scripts" / "init-llm-wiki.sh"), "--dest", str(project)])
            run([str(project / "llm-wiki-core" / "scripts" / "init-agent-os.sh"), "--dest", str(project)])

            self.assertTrue((project / ".agent-os" / "README.md").exists())
            self.assertTrue((project / ".agent-os" / "roles" / "default.md").exists())
            self.assertTrue((project / ".agent-os" / "tasks" / "llm-wiki-task.md").exists())
            self.assertTrue((project / "AGENTS.md").exists())
            self.assertIn("llm-wiki-core/templates/agent-os-agents.md", (project / "AGENTS.md").read_text())

            validate = run([str(project / "llm-wiki-core" / "scripts" / "llm-wiki-core"), "--root", str(project), "validate"])
            self.assertTrue(json.loads(validate.stdout)["ok"])

    def test_compressed_artifact_reading(self):
        import tarfile
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            # 1. Initialize scaffold
            run([str(INIT), "--dest", str(project), "--domain", "Artifact Test Wiki"])
            
            # 2. Prepare mock wiki pages for compressing
            wiki_src = project / "temp_wiki_src"
            wiki_src.mkdir()
            concept_dir = wiki_src / "concepts"
            concept_dir.mkdir()
            
            concept = concept_dir / "immutable-concept.md"
            concept.write_text("""---
title: Immutable Concept
status: accepted
tags: [immutable, test]
sources: [raw/source.md]
source_hashes:
  raw/source.md: def123
confidence: high
stale: true
review_needed: true
---
# Immutable Concept

This content is loaded directly from a compressed archive.
""")
            
            # 3. Create tar.gz archive
            artifacts_dir = project / "artifacts"
            artifacts_dir.mkdir()
            archive_path = artifacts_dir / "team-wiki.tar.gz"
            
            with tarfile.open(archive_path, "w:gz") as tar:
                tar.add(concept, arcname="concepts/immutable-concept.md")
                
            # 4. Set up wiki_stack.yaml listing the compressed artifact
            stack_path = project / "wiki_stack.yaml"
            stack_path.write_text("""
wiki_artifacts:
  - dependency_id: team-wiki
    artifact_ref: artifacts/team-wiki.tar.gz
    namespace: team:wiki

personal_wikis:
  - dependency_id: personal-wiki
    path: llm-wiki
    namespace: personal:wiki

selection_policy:
  default_mode: active-work
  conflict_resolution:
    default_action: ask_user
    record_user_decision: true
    autonomous_allowed: false
  priority_rules:
    priority_source: yaml_order
    earlier_entry_has_higher_priority: true
    group_block_has_equal_priority: true
    priority_grants_canonical_authority: false

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: team-wiki
      dependency_id: team-wiki
      dependency_type: artifact
      effective_scope: team
      authority_level: policy
    - source_binding_id: personal-wiki
      dependency_id: personal-wiki
      dependency_type: personal_wiki
      effective_scope: personal
      authority_level: advisory
""")
            
            # 5. Run validate
            validate = run([str(CLI), "--root", str(project), "validate"])
            self.assertTrue(json.loads(validate.stdout)["ok"])
            
            # 6. Test bundle generation (retrieves from archive)
            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            bundle_data = json.loads(bundle.stdout)
            self.assertTrue(bundle_data["ok"])
            self.assertEqual(bundle_data["selected_page_count"], 1)
            
            out_dir = Path(bundle_data["output_dir"])
            bundle_md = (out_dir / "context_bundle.md").read_text()
            self.assertIn("Immutable Concept", bundle_md)
            
            # 7. Test page search
            search = run([str(CLI), "--root", str(project), "search-pages", "directly from a compressed archive"])
            search_data = json.loads(search.stdout)
            self.assertEqual(search_data["count"], 1)
            self.assertEqual(search_data["matches"][0]["title"], "Immutable Concept")
            
            # 8. Test lineage lookup
            lineage = run([str(CLI), "--root", str(project), "get-lineage", "artifacts/team-wiki/concepts/immutable-concept.md"])
            lineage_data = json.loads(lineage.stdout)
            self.assertEqual(lineage_data["confidence"], "high")
            self.assertEqual(lineage_data["page"], "artifacts/team-wiki/concepts/immutable-concept.md")

    def test_wikipkg_artifact_reading(self):
        import zipfile
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            # 1. Initialize scaffold
            run([str(INIT), "--dest", str(project), "--domain", "Wikipkg Test Wiki"])

            # 2. Prepare mock wiki page for packing
            wiki_src = project / "temp_wiki_src"
            wiki_src.mkdir()
            concept_dir = wiki_src / "concepts"
            concept_dir.mkdir()

            concept = concept_dir / "wikipkg-concept.md"
            concept.write_text("""---
title: Wikipkg Concept
status: accepted
tags: [wikipkg, test]
sources: [raw/source.md]
source_hashes:
  raw/source.md: abc789
confidence: high
---
# Wikipkg Concept

This content is loaded directly from a wikipkg (zip) archive.
""")

            # 3. Create .wikipkg (zip) archive
            artifacts_dir = project / "artifacts"
            artifacts_dir.mkdir()
            archive_path = artifacts_dir / "team-wiki.wikipkg"

            with zipfile.ZipFile(archive_path, "w") as zf:
                zf.write(concept, arcname="concepts/wikipkg-concept.md")

            # 4. Set up wiki_stack.yaml listing the wikipkg artifact
            stack_path = project / "wiki_stack.yaml"
            stack_path.write_text("""
wiki_artifacts:
  - dependency_id: team-wiki
    artifact_ref: artifacts/team-wiki.wikipkg
    namespace: team:wiki

personal_wikis:
  - dependency_id: personal-wiki
    path: llm-wiki
    namespace: personal:wiki

selection_policy:
  default_mode: active-work
  conflict_resolution:
    default_action: ask_user
    record_user_decision: true
    autonomous_allowed: false
  priority_rules:
    priority_source: yaml_order
    earlier_entry_has_higher_priority: true
    group_block_has_equal_priority: true
    priority_grants_canonical_authority: false

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: team-wiki
      dependency_id: team-wiki
      dependency_type: artifact
      effective_scope: team
      authority_level: policy
    - source_binding_id: personal-wiki
      dependency_id: personal-wiki
      dependency_type: personal_wiki
      effective_scope: personal
      authority_level: advisory
""")

            # 5. Run validate
            validate = run([str(CLI), "--root", str(project), "validate"])
            self.assertTrue(json.loads(validate.stdout)["ok"])

            # 6. Test bundle generation (retrieves from archive)
            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            bundle_data = json.loads(bundle.stdout)
            self.assertTrue(bundle_data["ok"])
            self.assertEqual(bundle_data["selected_page_count"], 1)

            out_dir = Path(bundle_data["output_dir"])
            bundle_md = (out_dir / "context_bundle.md").read_text()
            self.assertIn("Wikipkg Concept", bundle_md)

            # 7. Test page search
            search = run([str(CLI), "--root", str(project), "search-pages", "directly from a wikipkg"])
            search_data = json.loads(search.stdout)
            self.assertEqual(search_data["count"], 1)
            self.assertEqual(search_data["matches"][0]["title"], "Wikipkg Concept")

            # 8. Test lineage lookup
            lineage = run([str(CLI), "--root", str(project), "get-lineage", "artifacts/team-wiki/concepts/wikipkg-concept.md"])
            lineage_data = json.loads(lineage.stdout)
            self.assertEqual(lineage_data["confidence"], "high")
            self.assertEqual(lineage_data["page"], "artifacts/team-wiki/concepts/wikipkg-concept.md")

    def test_raw_directory_excluded_from_personal_wiki_selection(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Raw Exclusion Test Wiki"])

            raw_dir = project / "llm-wiki" / "raw"
            raw_dir.mkdir(parents=True, exist_ok=True)
            (raw_dir / "raw-note.md").write_text("""# Raw Note

This is unprocessed raw evidence and should never be directly queryable.
""")

            concept = project / "llm-wiki" / "concepts" / "refined-concept.md"
            concept.parent.mkdir(parents=True, exist_ok=True)
            concept.write_text("""---
title: Refined Concept
status: accepted
sources: [raw/raw-note.md]
confidence: high
---
# Refined Concept

This is the refined document grounded in raw evidence.
""")

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            bundle_data = json.loads(bundle.stdout)

            out_dir = Path(bundle_data["output_dir"])
            import yaml
            selected_pages = yaml.safe_load((out_dir / "selected_pages.yaml").read_text())
            selected_paths = {p["path"] for p in selected_pages}

            self.assertNotIn("llm-wiki/raw/raw-note.md", selected_paths)
            self.assertIn("llm-wiki/concepts/refined-concept.md", selected_paths)

            search = run([str(CLI), "--root", str(project), "search-pages", "unprocessed raw evidence"])
            self.assertEqual(json.loads(search.stdout)["count"], 0)

    def test_raw_derived_directory_excluded_from_personal_wiki_selection(self):
        import yaml
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Raw Derived Exclusion Test Wiki"])

            raw_derived = project / "llm-wiki" / "raw-derived" / "demo_ws" / "prepared.md"
            raw_derived.parent.mkdir(parents=True, exist_ok=True)
            raw_derived.write_text("""---
title: Raw Derived Prepared Unit
status: accepted
---
# Raw Derived Prepared Unit

This prepared raw-derived metadata should never be directly queryable.
""")

            concept = project / "llm-wiki" / "concepts" / "refined-concept.md"
            concept.parent.mkdir(parents=True, exist_ok=True)
            concept.write_text("""---
title: Refined Concept
status: accepted
sources: [llm-wiki/raw/source.md]
confidence: high
---
# Refined Concept

This is the selected refined document.
""")

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            bundle_data = json.loads(bundle.stdout)

            out_dir = Path(bundle_data["output_dir"])
            selected_pages = yaml.safe_load((out_dir / "selected_pages.yaml").read_text())
            selected_paths = {p["path"] for p in selected_pages}

            self.assertNotIn("llm-wiki/raw-derived/demo_ws/prepared.md", selected_paths)
            self.assertIn("llm-wiki/concepts/refined-concept.md", selected_paths)

            search = run([str(CLI), "--root", str(project), "search-pages", "prepared raw-derived metadata"])
            self.assertEqual(json.loads(search.stdout)["count"], 0)

    def test_raw_directory_excluded_even_when_packaged_as_artifact(self):
        import zipfile
        import yaml
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Raw Artifact Exclusion Test Wiki"])

            staging = project / "temp_pack_src"
            staging.mkdir()
            (staging / "raw").mkdir()
            (staging / "raw" / "raw-note.md").write_text("# Raw Note\n\nUnprocessed raw evidence inside an artifact.\n")
            (staging / "concepts").mkdir()
            (staging / "concepts" / "shared-concept.md").write_text("""---
title: Shared Concept
status: accepted
confidence: high
---
# Shared Concept

Refined content shared via artifact.
""")

            artifacts_dir = project / "artifacts"
            artifacts_dir.mkdir()
            archive_path = artifacts_dir / "team-wiki.wikipkg"
            with zipfile.ZipFile(archive_path, "w") as zf:
                zf.write(staging / "raw" / "raw-note.md", arcname="raw/raw-note.md")
                zf.write(staging / "concepts" / "shared-concept.md", arcname="concepts/shared-concept.md")

            stack_path = project / "wiki_stack.yaml"
            stack_path.write_text("""
wiki_artifacts:
  - dependency_id: team-wiki
    artifact_ref: artifacts/team-wiki.wikipkg
    namespace: team:wiki

personal_wikis: []

selection_policy:
  default_mode: active-work
  conflict_resolution:
    default_action: ask_user
    record_user_decision: true
    autonomous_allowed: false
  priority_rules:
    priority_source: yaml_order
    earlier_entry_has_higher_priority: true
    group_block_has_equal_priority: true
    priority_grants_canonical_authority: false

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: team-wiki
      dependency_id: team-wiki
      dependency_type: artifact
      effective_scope: team
      authority_level: policy
""")

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            bundle_data = json.loads(bundle.stdout)
            out_dir = Path(bundle_data["output_dir"])
            selected_pages = yaml.safe_load((out_dir / "selected_pages.yaml").read_text())
            selected_paths = {p["path"] for p in selected_pages}

            self.assertNotIn("artifacts/team-wiki/raw/raw-note.md", selected_paths)
            self.assertIn("artifacts/team-wiki/concepts/shared-concept.md", selected_paths)

    def test_pack_artifact_excludes_generated_junk(self):
        import zipfile
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Pack Artifact Test Wiki"])

            concept = project / "llm-wiki" / "concepts" / "keep-me.md"
            concept.write_text("---\ntitle: Keep Me\nstatus: accepted\nconfidence: high\n---\n# Keep Me\n")

            junk_build = project / "llm-wiki" / "raw" / "ros2_ws" / "build"
            junk_build.mkdir(parents=True)
            (junk_build / "artifact.o").write_text("binary build output")

            junk_pycache = project / "llm-wiki" / "raw" / "__pycache__"
            junk_pycache.mkdir(parents=True)
            (junk_pycache / "mod.pyc").write_text("bytecode")

            (project / "llm-wiki" / ".DS_Store").write_text("mac junk")

            output = project / "artifacts" / "team-wiki.wikipkg"
            pack = run([str(CLI), "--root", str(project), "pack-artifact", "--output", str(output)])
            pack_data = json.loads(pack.stdout)
            self.assertTrue(pack_data["ok"])
            self.assertTrue(output.exists())

            with zipfile.ZipFile(output) as zf:
                names = zf.namelist()

            self.assertTrue(any(n.endswith("concepts/keep-me.md") for n in names))
            self.assertFalse(any("/build/" in n for n in names))
            self.assertFalse(any("__pycache__" in n for n in names))
            self.assertFalse(any(n.endswith(".DS_Store") for n in names))

    def test_pack_artifact_adds_fixed_wikipkg_extension_to_output_name(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Pack Artifact Output Name Test Wiki"])

            concept = project / "llm-wiki" / "concepts" / "keep-me.md"
            concept.write_text("---\ntitle: Keep Me\nstatus: accepted\nconfidence: high\n---\n# Keep Me\n")

            requested_output = project / "artifacts" / "team-wiki"
            expected_output = project / "artifacts" / "team-wiki.wikipkg"
            pack = run([str(CLI), "--root", str(project), "pack-artifact", "--output", str(requested_output)])
            pack_data = json.loads(pack.stdout)

            self.assertTrue(pack_data["ok"])
            self.assertEqual(Path(pack_data["output"]), expected_output)
            self.assertFalse(requested_output.exists())
            self.assertTrue(expected_output.exists())

    def test_build_wiki_artifact_does_not_write_namespace_to_artifact_metadata(self):
        import tarfile
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Build Artifact Namespace Test Wiki"])

            output = project / "artifacts" / "team-wiki-1.0.0.tar.gz"
            result = run([
                "bash",
                str(BUILD_WIKI_ARTIFACT),
                "--src",
                str(project / "llm-wiki"),
                "--dest",
                str(output),
                "--namespace",
                "team:wiki",
            ])

            self.assertIn("namespace: team:wiki", result.stdout)
            with tarfile.open(output, "r:gz") as tar:
                artifact_member = next(m for m in tar.getmembers() if m.name in {"artifact.yaml", "./artifact.yaml"})
                f = tar.extractfile(artifact_member)
                self.assertIsNotNone(f)
                artifact_meta = yaml.safe_load(f.read().decode("utf-8"))

            self.assertEqual(artifact_meta["version"], "0.1.0")
            self.assertNotIn("format", artifact_meta)
            self.assertNotIn("namespace", artifact_meta)


    def test_get_context_bundle_emits_raw_derived_plan_for_complex_raw_directory(self):
        import yaml
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Raw Derived Test"])

            raw_ws = project / "llm-wiki" / "raw" / "demo_ws"
            (raw_ws / "src" / "peripherals" / "launch" / "include").mkdir(parents=True)
            (raw_ws / "src" / "peripherals" / "config").mkdir(parents=True)
            (raw_ws / "build").mkdir()
            (raw_ws / "log").mkdir()
            (raw_ws / "src" / "peripherals" / "package.xml").write_text("<package><name>peripherals</name></package>")
            (raw_ws / "src" / "peripherals" / "launch" / "include" / "hp60c.launch.py").write_text("namespace='ascamera_hp60c'\n")
            (raw_ws / "src" / "peripherals" / "config" / "camera_info.yaml").write_text("camera_name: hp60c\n")
            (raw_ws / "build" / "generated.pyc").write_bytes(b"generated")
            (raw_ws / "log" / "build.log").write_text("ignore me\n")

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            bundle_data = json.loads(bundle.stdout)
            self.assertTrue(bundle_data["ok"])

            out_dir = Path(bundle_data["output_dir"])
            registry = yaml.safe_load((out_dir / "raw_derived_registry.yaml").read_text())
            manifests = yaml.safe_load((out_dir / "raw_derived_manifests.yaml").read_text())
            divider = yaml.safe_load((out_dir / "raw_divider_plan.yaml").read_text())

            self.assertEqual(registry["schema_version"], "raw-derived-registry/v1")
            self.assertEqual(registry["items"][0]["id"], "demo_ws")
            self.assertEqual(registry["items"][0]["raw_path"], "llm-wiki/raw/demo_ws")
            self.assertEqual(registry["items"][0]["kind"], "project_tree")

            manifest = manifests["items"]["demo_ws"]
            self.assertEqual(manifest["schema_version"], "raw-derived-manifest/v1")
            self.assertEqual(manifest["source_type"], "source_code_snapshot")
            self.assertEqual(manifest["mutability"], "updatable_snapshot")
            self.assertGreaterEqual(manifest["fingerprint"]["file_count"], 5)

            units = divider["items"]["demo_ws"]["units"]
            unit_ids = {u["id"] for u in units}
            self.assertIn("project_overview", unit_ids)
            self.assertIn("package_peripherals", unit_ids)
            self.assertNotIn("build", "\n".join(str(u) for u in units))
            self.assertNotIn("log", "\n".join(str(u) for u in units))

    def test_list_raw_items_reports_complex_and_simple_raw_items(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Raw List Test"])
            (project / "llm-wiki" / "raw" / "simple_note.md").write_text("# Simple\n")
            raw_ws = project / "llm-wiki" / "raw" / "demo_ws"
            (raw_ws / "src" / "pkg").mkdir(parents=True)
            (raw_ws / "src" / "pkg" / "package.xml").write_text("<package><name>pkg</name></package>")

            listed = run([str(CLI), "--root", str(project), "list-raw-items"])
            data = json.loads(listed.stdout)
            by_id = {item["id"]: item for item in data["items"]}
            self.assertEqual(by_id["simple_note"]["kind"], "raw_file")
            self.assertEqual(by_id["demo_ws"]["kind"], "project_tree")

    def test_get_raw_derived_manifest_returns_one_complex_item(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Raw Manifest Test"])
            raw_ws = project / "llm-wiki" / "raw" / "demo_ws"
            (raw_ws / "src" / "pkg").mkdir(parents=True)
            (raw_ws / "src" / "pkg" / "package.xml").write_text("<package><name>pkg</name></package>")

            manifest = run([str(CLI), "--root", str(project), "get-raw-derived-manifest", "demo_ws"])
            data = json.loads(manifest.stdout)
            self.assertTrue(data["found"])
            self.assertEqual(data["manifest"]["id"], "demo_ws")
            self.assertEqual(data["divider"]["raw_item"], "demo_ws")


    def test_orphaned_source_warns_when_log_entry_missing(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Test Wiki"])

            summary = project / "llm-wiki" / "sources" / "orphan_test_summary.md"
            summary.write_text("""---
title: Orphan Test Summary
status: accepted
tags: [summary]
sources: [llm-wiki/raw/orphan_test.md]
---
# Orphan Test
""")

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            bundle_data = json.loads(bundle.stdout)
            self.assertIn("orphaned_source: orphan_test.md", bundle_data["warnings"])

    def test_orphaned_source_warning_clears_once_log_entry_added(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Test Wiki"])

            summary = project / "llm-wiki" / "sources" / "orphan_test_summary.md"
            summary.write_text("""---
title: Orphan Test Summary
status: accepted
tags: [summary]
sources: [llm-wiki/raw/orphan_test.md]
---
# Orphan Test
""")

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            self.assertIn("orphaned_source: orphan_test.md", json.loads(bundle.stdout)["warnings"])

            # Sleep past mtime granularity so the cache is provably invalidated
            # by the log.md edit below, not coincidentally reused.
            time.sleep(1.1)
            log_path = project / "llm-wiki" / "log.md"
            with log_path.open("a") as f:
                f.write("\n## [2026-06-18] ingest | raw/orphan_test.md\n- Source: test\n- Details: test\n")

            bundle2 = run([str(CLI), "--root", str(project), "get-context-bundle"])
            self.assertNotIn("orphaned_source: orphan_test.md", json.loads(bundle2.stdout)["warnings"])

    def test_orphaned_source_matches_legacy_log_format_without_raw_prefix(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Test Wiki"])

            summary = project / "llm-wiki" / "sources" / "orphan_test_summary.md"
            summary.write_text("""---
title: Orphan Test Summary
status: accepted
tags: [summary]
sources: [llm-wiki/raw/orphan_test.md]
---
# Orphan Test
""")

            log_path = project / "llm-wiki" / "log.md"
            with log_path.open("a") as f:
                f.write("\n## [2026-06-18] ingest | orphan_test.md\n- Source: test\n- Details: test\n")

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            self.assertNotIn("orphaned_source: orphan_test.md", json.loads(bundle.stdout)["warnings"])


    def test_conflict_resolved_by_priority_when_tiers_differ(self):
        import tarfile
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Conflict Test Wiki"])

            concept = project / "team_concept_src.md"
            concept.write_text("""---
title: Shared Concept
status: accepted
confidence: high
---
# Shared Concept

Team version.
""")
            artifacts_dir = project / "artifacts"
            artifacts_dir.mkdir()
            archive_path = artifacts_dir / "team-wiki.tar.gz"
            with tarfile.open(archive_path, "w:gz") as tar:
                tar.add(concept, arcname="concepts/shared-concept.md")

            personal_concept = project / "llm-wiki" / "concepts" / "shared-concept.md"
            personal_concept.write_text("""---
title: Shared Concept
status: accepted
confidence: high
---
# Shared Concept

Personal version (different content).
""")

            stack_path = project / "wiki_stack.yaml"
            stack_path.write_text("""
wiki_artifacts:
  - dependency_id: team-wiki
    artifact_ref: artifacts/team-wiki.tar.gz
    namespace: team:wiki

personal_wikis:
  - dependency_id: personal-wiki
    path: llm-wiki
    namespace: personal:wiki

selection_policy:
  default_mode: active-work
  conflict_resolution:
    default_action: ask_user
    record_user_decision: true
    autonomous_allowed: false
  priority_rules:
    priority_source: yaml_order
    earlier_entry_has_higher_priority: true
    group_block_has_equal_priority: true
    priority_grants_canonical_authority: false

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: team-wiki
      dependency_id: team-wiki
      dependency_type: artifact
      effective_scope: team
      authority_level: policy
    - source_binding_id: personal-wiki
      dependency_id: personal-wiki
      dependency_type: personal_wiki
      effective_scope: personal
      authority_level: advisory
""")

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            bundle_data = json.loads(bundle.stdout)
            self.assertTrue(bundle_data["ok"])
            self.assertEqual(bundle_data["conflicts"], [])
            resolved = bundle_data["resolved_conflicts"]
            self.assertEqual(len(resolved), 1)
            self.assertEqual(resolved[0]["title"], "Shared Concept")
            self.assertEqual(resolved[0]["chosen"], "artifacts/team-wiki/concepts/shared-concept.md")
            self.assertEqual(resolved[0]["rule"], "priority")
            self.assertIn('show-conflict "Shared Concept"', resolved[0]["command"])

    def test_conflict_remains_unresolved_when_tiers_are_equal(self):
        import tarfile
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Conflict Tie Test"])

            concept = project / "team_concept_src.md"
            concept.write_text("""---
title: Tied Concept
status: accepted
confidence: high
---
# Tied Concept

Team version.
""")
            artifacts_dir = project / "artifacts"
            artifacts_dir.mkdir()
            archive_path = artifacts_dir / "team-wiki.tar.gz"
            with tarfile.open(archive_path, "w:gz") as tar:
                tar.add(concept, arcname="concepts/tied-concept.md")

            personal_concept = project / "llm-wiki" / "concepts" / "tied-concept.md"
            personal_concept.write_text("""---
title: Tied Concept
status: accepted
confidence: high
---
# Tied Concept

Personal version (different content).
""")

            stack_path = project / "wiki_stack.yaml"
            stack_path.write_text("""
wiki_artifacts:
  - dependency_id: team-wiki
    artifact_ref: artifacts/team-wiki.tar.gz
    namespace: team:wiki

personal_wikis:
  - dependency_id: personal-wiki
    path: llm-wiki
    namespace: personal:wiki

selection_policy:
  default_mode: active-work
  conflict_resolution:
    default_action: ask_user
    record_user_decision: true
    autonomous_allowed: false
  priority_rules:
    priority_source: yaml_order
    earlier_entry_has_higher_priority: true
    group_block_has_equal_priority: true
    priority_grants_canonical_authority: false

mutable_source_wiki_policy:
  source_binding_order:
    - group:
        - source_binding_id: team-wiki
          dependency_id: team-wiki
          dependency_type: artifact
          effective_scope: team
          authority_level: policy
        - source_binding_id: personal-wiki
          dependency_id: personal-wiki
          dependency_type: personal_wiki
          effective_scope: personal
          authority_level: advisory
""")

            bundle = run_unchecked([str(CLI), "--root", str(project), "get-context-bundle"])
            bundle_data = json.loads(bundle.stdout)
            self.assertFalse(bundle_data["ok"])
            self.assertEqual(bundle.returncode, 1)
            self.assertEqual(bundle_data["resolved_conflicts"], [])
            self.assertEqual(len(bundle_data["conflicts"]), 1)
            self.assertEqual(bundle_data["conflicts"][0]["title"], "Tied Concept")

    def test_conflict_detection_governed_scopes_are_configurable(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Configurable Conflict Policy Test"])

            first = project / "llm-wiki" / "concepts" / "first-local.md"
            second = project / "llm-wiki" / "concepts" / "second-local.md"
            first.parent.mkdir(parents=True, exist_ok=True)
            first.write_text("""---
title: Local Policy Conflict
status: accepted
confidence: high
---
# Local Policy Conflict

First local version.
""")
            second.write_text("""---
title: Local Policy Conflict
status: accepted
confidence: high
---
# Local Policy Conflict

Second local version.
""")

            stack_path = project / "wiki_stack.yaml"
            stack_path.write_text("""
wiki_artifacts: []
personal_wikis: []

selection_policy:
  default_mode: active-work
  conflict_detection:
    governed_scopes:
      - local
  conflict_resolution:
    default_action: ask_user
    record_user_decision: true
    autonomous_allowed: false
  priority_rules:
    priority_source: yaml_order
    earlier_entry_has_higher_priority: true
    group_block_has_equal_priority: true
    priority_grants_canonical_authority: false

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: local-mutable-wiki
""")

            bundle = run_unchecked([str(CLI), "--root", str(project), "get-context-bundle"])
            bundle_data = json.loads(bundle.stdout)
            self.assertEqual(bundle.returncode, 1)
            self.assertFalse(bundle_data["ok"])
            self.assertEqual(len(bundle_data["conflicts"]), 1)
            self.assertEqual(bundle_data["conflicts"][0]["title"], "Local Policy Conflict")

    def test_conflict_detection_empty_governed_scopes_disables_conflicts(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Disabled Conflict Policy Test"])

            local_page = project / "llm-wiki" / "concepts" / "disabled-conflict.md"
            local_page.parent.mkdir(parents=True, exist_ok=True)
            local_page.write_text("""---
title: Disabled Policy Conflict
status: accepted
confidence: high
---
# Disabled Policy Conflict

Local version.
""")

            self.write_artifact_with_page(project, "concepts/disabled-conflict.md", """---
title: Disabled Policy Conflict
status: accepted
confidence: high
---
# Disabled Policy Conflict

Artifact version.
""")

            stack_path = project / "wiki_stack.yaml"
            stack_path.write_text("""
wiki_artifacts:
  - dependency_id: team-wiki
    artifact_ref: artifacts/team-wiki.tar.gz
    namespace: team:wiki

personal_wikis: []

selection_policy:
  default_mode: active-work
  conflict_detection:
    governed_scopes: []
  conflict_resolution:
    default_action: ask_user
    record_user_decision: true
    autonomous_allowed: false
  priority_rules:
    priority_source: yaml_order
    earlier_entry_has_higher_priority: true
    group_block_has_equal_priority: true
    priority_grants_canonical_authority: false

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: local-mutable-wiki
    - source_binding_id: team-wiki
      dependency_id: team-wiki
      dependency_type: artifact
      effective_scope: team
      authority_level: policy
""")

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            bundle_data = json.loads(bundle.stdout)
            self.assertTrue(bundle_data["ok"])
            self.assertEqual(bundle_data["conflicts"], [])
            self.assertEqual(bundle_data["resolved_conflicts"], [])

    def test_conflict_pages_deduplicated_when_local_mutable_wiki_overlaps_personal_wiki(self):
        import tarfile
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Dedup Conflict Test"])

            concept = project / "team_concept_src.md"
            concept.write_text("""---
title: Dedup Concept
status: accepted
confidence: high
---
# Dedup Concept

Team version.
""")
            artifacts_dir = project / "artifacts"
            artifacts_dir.mkdir()
            archive_path = artifacts_dir / "team-wiki.tar.gz"
            with tarfile.open(archive_path, "w:gz") as tar:
                tar.add(concept, arcname="concepts/dedup-concept.md")

            personal_concept = project / "llm-wiki" / "concepts" / "dedup-concept.md"
            personal_concept.write_text("""---
title: Dedup Concept
status: accepted
confidence: high
---
# Dedup Concept

Personal version.
""")

            stack_path = project / "wiki_stack.yaml"
            stack_path.write_text("""
wiki_artifacts:
  - dependency_id: team-wiki
    artifact_ref: artifacts/team-wiki.tar.gz
    namespace: team:wiki

personal_wikis:
  - dependency_id: personal-wiki
    path: llm-wiki
    namespace: personal:wiki

selection_policy:
  default_mode: active-work
  conflict_resolution:
    default_action: ask_user
    record_user_decision: true
    autonomous_allowed: false
  priority_rules:
    priority_source: yaml_order
    earlier_entry_has_higher_priority: true
    group_block_has_equal_priority: true
    priority_grants_canonical_authority: false

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: team-wiki
      dependency_id: team-wiki
      dependency_type: artifact
      effective_scope: team
      authority_level: policy
    - source_binding_id: personal-wiki
      dependency_id: personal-wiki
      dependency_type: personal_wiki
      effective_scope: personal
      authority_level: advisory
""")

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            resolved = json.loads(bundle.stdout)["resolved_conflicts"]
            self.assertEqual(len(resolved), 1)
            self.assertEqual(len(resolved[0]["rejected"]), 1)
            self.assertEqual(resolved[0]["chosen"], "artifacts/team-wiki/concepts/dedup-concept.md")
            self.assertEqual(resolved[0]["rejected"], ["llm-wiki/concepts/dedup-concept.md"])


    def test_show_conflict_returns_diff_for_active_conflict(self):
        import tarfile
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Show Conflict Test"])

            concept = project / "team_concept_src.md"
            concept.write_text("""---
title: Diffable Concept
status: accepted
confidence: high
---
# Diffable Concept

Team version line.
""")
            artifacts_dir = project / "artifacts"
            artifacts_dir.mkdir()
            archive_path = artifacts_dir / "team-wiki.tar.gz"
            with tarfile.open(archive_path, "w:gz") as tar:
                tar.add(concept, arcname="concepts/diffable-concept.md")

            personal_concept = project / "llm-wiki" / "concepts" / "diffable-concept.md"
            personal_concept.write_text("""---
title: Diffable Concept
status: accepted
confidence: high
---
# Diffable Concept

Personal version line.
""")

            stack_path = project / "wiki_stack.yaml"
            stack_path.write_text("""
wiki_artifacts:
  - dependency_id: team-wiki
    artifact_ref: artifacts/team-wiki.tar.gz
    namespace: team:wiki

personal_wikis:
  - dependency_id: personal-wiki
    path: llm-wiki
    namespace: personal:wiki

selection_policy:
  default_mode: active-work
  conflict_resolution:
    default_action: ask_user
    record_user_decision: true
    autonomous_allowed: false
  priority_rules:
    priority_source: yaml_order
    earlier_entry_has_higher_priority: true
    group_block_has_equal_priority: true
    priority_grants_canonical_authority: false

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: team-wiki
      dependency_id: team-wiki
      dependency_type: artifact
      effective_scope: team
      authority_level: policy
    - source_binding_id: personal-wiki
      dependency_id: personal-wiki
      dependency_type: personal_wiki
      effective_scope: personal
      authority_level: advisory
""")

            show = run([str(CLI), "--root", str(project), "show-conflict", "Diffable Concept"])
            show_data = json.loads(show.stdout)
            self.assertTrue(show_data["found"])
            self.assertEqual(show_data["title"], "Diffable Concept")
            self.assertEqual(len(show_data["pages"]), 2)
            self.assertIn("Team version line.", show_data["diff"])
            self.assertIn("Personal version line.", show_data["diff"])

    def test_show_conflict_reports_not_found_for_unknown_title(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Show Conflict Missing Test"])

            show = run_unchecked([str(CLI), "--root", str(project), "show-conflict", "Nonexistent Title"])
            self.assertEqual(show.returncode, 1)
            show_data = json.loads(show.stdout)
            self.assertFalse(show_data["found"])


    def test_resolve_conflict_override_takes_effect_on_next_bundle(self):
        import tarfile
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Resolve Conflict Test"])

            concept = project / "team_concept_src.md"
            concept.write_text("""---
title: Override Concept
status: accepted
confidence: high
---
# Override Concept

Team version.
""")
            artifacts_dir = project / "artifacts"
            artifacts_dir.mkdir()
            archive_path = artifacts_dir / "team-wiki.tar.gz"
            with tarfile.open(archive_path, "w:gz") as tar:
                tar.add(concept, arcname="concepts/override-concept.md")

            personal_concept = project / "llm-wiki" / "concepts" / "override-concept.md"
            personal_concept.write_text("""---
title: Override Concept
status: accepted
confidence: high
---
# Override Concept

Personal version (improved).
""")

            stack_path = project / "wiki_stack.yaml"
            stack_path.write_text("""
wiki_artifacts:
  - dependency_id: team-wiki
    artifact_ref: artifacts/team-wiki.tar.gz
    namespace: team:wiki

personal_wikis:
  - dependency_id: personal-wiki
    path: llm-wiki
    namespace: personal:wiki

selection_policy:
  default_mode: active-work
  conflict_resolution:
    default_action: ask_user
    record_user_decision: true
    autonomous_allowed: false
  priority_rules:
    priority_source: yaml_order
    earlier_entry_has_higher_priority: true
    group_block_has_equal_priority: true
    priority_grants_canonical_authority: false

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: team-wiki
      dependency_id: team-wiki
      dependency_type: artifact
      effective_scope: team
      authority_level: policy
    - source_binding_id: personal-wiki
      dependency_id: personal-wiki
      dependency_type: personal_wiki
      effective_scope: personal
      authority_level: advisory
""")

            # Before override: team (higher priority) wins.
            bundle1 = run([str(CLI), "--root", str(project), "get-context-bundle"])
            resolved1 = json.loads(bundle1.stdout)["resolved_conflicts"]
            self.assertEqual(resolved1[0]["chosen"], "artifacts/team-wiki/concepts/override-concept.md")
            self.assertEqual(resolved1[0]["rule"], "priority")

            # Record an override choosing the personal (lower-priority) page.
            time.sleep(1.1)  # defeat mtime-granularity false negatives in the cache check
            override = run([
                str(CLI), "--root", str(project), "resolve-conflict", "Override Concept",
                "--choose", "llm-wiki/concepts/override-concept.md",
            ])
            override_data = json.loads(override.stdout)
            self.assertTrue(override_data["ok"])

            # After override: personal wins, and the cache was invalidated to pick it up.
            bundle2 = run([str(CLI), "--root", str(project), "get-context-bundle"])
            resolved2 = json.loads(bundle2.stdout)["resolved_conflicts"]
            self.assertEqual(resolved2[0]["chosen"], "llm-wiki/concepts/override-concept.md")
            self.assertEqual(resolved2[0]["rule"], "user_decision")

    def test_resolve_conflict_rejects_unknown_title_and_invalid_path(self):
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Resolve Conflict Invalid Test"])

            bad_title = run_unchecked([
                str(CLI), "--root", str(project), "resolve-conflict", "No Such Title",
                "--choose", "llm-wiki/concepts/nope.md",
            ])
            self.assertEqual(bad_title.returncode, 1)
            self.assertFalse(json.loads(bad_title.stdout)["ok"])

    def test_resolved_conflict_loser_does_not_leak_into_selected_pages(self):
        import tarfile
        with tempfile.TemporaryDirectory() as td:
            project = Path(td)
            run([str(INIT), "--dest", str(project), "--domain", "Leak Test"])

            concept = project / "team_concept_src.md"
            concept.write_text("""---
title: Leak Concept
status: accepted
confidence: high
---
# Leak Concept

Team version.
""")
            artifacts_dir = project / "artifacts"
            artifacts_dir.mkdir()
            archive_path = artifacts_dir / "team-wiki.tar.gz"
            with tarfile.open(archive_path, "w:gz") as tar:
                tar.add(concept, arcname="concepts/leak-concept.md")

            personal_concept = project / "llm-wiki" / "concepts" / "leak-concept.md"
            personal_concept.write_text("""---
title: Leak Concept
status: accepted
confidence: high
---
# Leak Concept

Personal version.
""")

            stack_path = project / "wiki_stack.yaml"
            stack_path.write_text("""
wiki_artifacts:
  - dependency_id: team-wiki
    artifact_ref: artifacts/team-wiki.tar.gz
    namespace: team:wiki

personal_wikis:
  - dependency_id: personal-wiki
    path: llm-wiki
    namespace: personal:wiki

selection_policy:
  default_mode: active-work
  conflict_resolution:
    default_action: ask_user
    record_user_decision: true
    autonomous_allowed: false
  priority_rules:
    priority_source: yaml_order
    earlier_entry_has_higher_priority: true
    group_block_has_equal_priority: true
    priority_grants_canonical_authority: false

mutable_source_wiki_policy:
  source_binding_order:
    - source_binding_id: team-wiki
      dependency_id: team-wiki
      dependency_type: artifact
      effective_scope: team
      authority_level: policy
    - source_binding_id: personal-wiki
      dependency_id: personal-wiki
      dependency_type: personal_wiki
      effective_scope: personal
      authority_level: advisory
""")

            bundle = run([str(CLI), "--root", str(project), "get-context-bundle"])
            bundle_data = json.loads(bundle.stdout)
            self.assertEqual(bundle_data["selected_page_count"], 1)
            out_dir = Path(bundle_data["output_dir"])
            bundle_md = (out_dir / "context_bundle.md").read_text()
            self.assertEqual(bundle_md.count("### Leak Concept"), 1)
            self.assertNotIn("Personal version.", bundle_md)


if __name__ == "__main__":
    unittest.main()
