from __future__ import annotations

import argparse
import difflib
import hashlib
import json
import os
import re
import shutil
import sys
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

try:
    import yaml
except Exception as exc:  # pragma: no cover
    raise SystemExit("PyYAML is required: python3 -m pip install --user PyYAML") from exc

FORBIDDEN_KEYS = {
    "trust_level",
    "binding_id",
    "source_binding_groups",
    "group_id",
    "parallel",
    "priority",
    "default_include",
    "read_only",
    "cannot_override",
    "artifact_bindings",
    "context_stack",
}

DEFAULT_EXCLUDE_GLOBS = [
    "AGENTS.md",
    "**/AGENTS.md",
    "**/SCHEMA.md",
    "**/README.md",
    "**/index.md",
    "**/log.md",
    "raw/**",
    "**/raw/**",
    "raw-derived/**",
    "**/raw-derived/**",
    ".agent-harness/**",
    ".agent-os/**",
    "skills/**",
    "llm-wiki-core/skills/**",
    "**/skills/**",
    "hooks/**",
    "**/hooks/**",
    "tests/**",
    "**/tests/**",
    "scripts/**",
    "**/scripts/**",
    "llm_wiki_core/**",
    "**/llm_wiki_core/**",
    "docs/harness/**",
    "docs/schema/examples/**",
    "examples/**",
    "fixtures/**",
    "tests/fixtures/**",
]

DEFAULT_STATUS_EXCLUDE = {"experimental", "draft"}
DEFAULT_PENALIZE_FLAGS = {"review_needed", "contested", "stale"}

TAR_ARCHIVE_EXTENSIONS = [".tar.gz", ".tgz", ".tar.zst", ".tar"]
ZIP_ARCHIVE_EXTENSIONS = [".wikipkg"]
ARCHIVE_EXTENSIONS = TAR_ARCHIVE_EXTENSIONS + ZIP_ARCHIVE_EXTENSIONS
LOCAL_MUTABLE_WIKI_ID = "local-mutable-wiki"
LOCAL_WIKI_TYPE = "local_wiki"

DEFAULT_PACK_EXCLUDE_DIRS = {"build", "install", "log", "__pycache__", ".git"}
DEFAULT_PACK_EXCLUDE_FILE_NAMES = {".DS_Store"}
DEFAULT_PACK_EXCLUDE_FILE_SUFFIXES = (".pyc",)
PROMOTION_PACKAGE_TARGET_KEYS = {"target_scope", "target_dependency_id", "target_artifact", "target_artifact_ref", "target_binding_id"}
PROMOTION_REFINED_TARGET_PREFIXES = {"concepts", "decisions", "comparisons", "queries", "sources", "metadata"}

CONFLICT_DECISIONS_REL_PATH = Path("llm-wiki") / "metadata" / "conflict_decisions.yaml"
BUNDLE_RUN_NAME_RE = re.compile(r"^run-\d{8}-\d{6}$")
DEFAULT_BUNDLE_RETENTION_COUNT = 10
MIN_BUNDLE_RETENTION_COUNT = 3
DEFAULT_PENDING_CAPTURE_RETENTION_COUNT = 50
MIN_PENDING_CAPTURE_RETENTION_COUNT = 10
PENDING_CAPTURE_NAME_RE = re.compile(r"^\d{8}-\d{6}-.+\.md$")

RAW_DERIVED_SCHEMA_VERSION = "raw-derived-registry/v1"
RAW_DERIVED_MANIFEST_SCHEMA_VERSION = "raw-derived-manifest/v1"
RAW_DIVIDER_SCHEMA_VERSION = "raw-divider/v1"
RAW_INVENTORY_SCHEMA_VERSION = "raw-inventory/v1"
RAW_PREPARED_UNIT_SCHEMA_VERSION = "raw-prepared-unit/v1"
DUPLICATE_CANDIDATES_SCHEMA_VERSION = "duplicate-candidates/v1"

CONFLICT_TYPE_SAME_TITLE = "same_title_conflict"
CONFLICT_DEFAULT_ACTION_ASK_USER = "ask_user"
CONFLICT_GOVERNED_SCOPES = {"project", "team"}

RESOLUTION_RULE_PRIORITY = "priority"
RESOLUTION_RULE_USER_DECISION = "user_decision"

DUPLICATE_TYPE_EXACT = "exact_duplicate"
DUPLICATE_TYPE_SAME_TITLE_DIVERGENCE = "same_title_divergence"
DUPLICATE_TYPE_SAME_SOURCES = "same_sources_candidate"
DUPLICATE_RECOMMENDATION_LOCAL_PRIMARY_REFERENCE_ARTIFACT = "local_primary_reference_artifact"
DUPLICATE_RISK_LOCAL_DIFFERS_FROM_REFERENCE = "local_differs_from_official_reference"
DUPLICATE_RISK_LOCAL_ARTIFACT_TITLE_DIVERGENCE = "local_artifact_same_title_divergence"
DUPLICATE_SELECTION_EFFECT_NONE = "none"
DUPLICATE_SELECTION_EFFECT_HANDLED_BY_CONFLICT_RESOLUTION = "handled_by_conflict_resolution"

RAW_DERIVED_EXCLUDE_GLOBS = [
    "build/**",
    "install/**",
    "log/**",
    "src/build/**",
    "**/__pycache__/**",
    "**/*.pyc",
    "**/*.so",
    "**/*.pt",
    "**/*.onnx",
    "**/*.engine",
    "**/.DS_Store",
]


@dataclass
class SourceBinding:
    source_binding_id: str
    dependency_id: str
    dependency_type: str
    effective_scope: str
    authority_level: str
    tier: int
    group_index: Optional[int] = None


@dataclass
class ConflictDetectionPolicy:
    governed_scopes: set[str]


@dataclass
class PageRecord:
    page_id: str
    path: str
    source_binding_id: str
    dependency_id: str
    effective_scope: str
    authority_level: str
    title: str
    status: str
    tags: List[str]
    sources: List[str]
    source_hashes: Dict[str, str]
    review_needed: bool
    stale: bool
    confidence: str
    sha256: str
    selected: bool
    score: float
    warnings: List[str]
    body_sha256: str = ""


def now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def parse_positive_int(value: Any, default: int) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return default
    return parsed if parsed > 0 else default


def load_yaml(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(str(path))
    data = yaml.safe_load(path.read_text())
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must parse to a YAML mapping")
    return data


def dump_yaml(data: Any) -> str:
    return yaml.safe_dump(data, sort_keys=False, allow_unicode=True)


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def rel_to(path: Path, root: Path) -> str:
    try:
        return str(path.resolve().relative_to(root.resolve()))
    except Exception:
        return str(path)


def path_matches_glob(rel: str, patterns: Iterable[str]) -> bool:
    import fnmatch
    p = Path(rel)
    for pat in patterns:
        if p.match(pat) or rel == pat or fnmatch.fnmatch(rel, pat) or fnmatch.fnmatch(rel, f"**/{pat}"):
            return True
    return False


def read_frontmatter(text: str) -> Tuple[Dict[str, Any], str]:
    if text.startswith("---\n"):
        end = text.find("\n---\n", 4)
        if end != -1:
            raw = text[4:end]
            body = text[end + 5 :]
            fm = yaml.safe_load(raw) or {}
            if not isinstance(fm, dict):
                fm = {}
            return fm, body
    return {}, text


def normalize_markdown_body(body: str) -> str:
    normalized = body.replace("\r\n", "\n").replace("\r", "\n")
    return "\n".join(line.rstrip() for line in normalized.strip().split("\n"))


def markdown_body_sha256(text: str) -> str:
    _, body = read_frontmatter(text)
    return sha256_text(normalize_markdown_body(body))


def normalize_list(value: Any) -> List[str]:
    if value is None:
        return []
    if isinstance(value, list):
        return [str(x) for x in value]
    return [str(value)]


def load_conflict_detection_policy(stack: Dict[str, Any]) -> ConflictDetectionPolicy:
    selection_policy = stack.get("selection_policy") or {}
    raw_policy = selection_policy.get("conflict_detection") or {}
    if "governed_scopes" in raw_policy:
        governed_scopes = normalize_list(raw_policy.get("governed_scopes"))
    else:
        governed_scopes = sorted(CONFLICT_GOVERNED_SCOPES)
    return ConflictDetectionPolicy(governed_scopes={str(scope) for scope in governed_scopes})


def conflict_policy_entry(policy: Optional[ConflictDetectionPolicy]) -> Dict[str, Any]:
    active_policy = policy or ConflictDetectionPolicy(governed_scopes=set(CONFLICT_GOVERNED_SCOPES))
    return {"governed_scopes": sorted(active_policy.governed_scopes)}


def validate_no_forbidden_keys(data: Any, path: str = "") -> List[str]:
    found: List[str] = []
    if isinstance(data, dict):
        for k, v in data.items():
            next_path = f"{path}/{k}" if path else str(k)
            if k in FORBIDDEN_KEYS:
                found.append(next_path)
            found.extend(validate_no_forbidden_keys(v, next_path))
    elif isinstance(data, list):
        for i, v in enumerate(data):
            found.extend(validate_no_forbidden_keys(v, f"{path}[{i}]"))
    return found


def dependency_ids(stack: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    deps: Dict[str, Dict[str, Any]] = {}
    for item in stack.get("wiki_artifacts", []) or []:
        if isinstance(item, dict) and item.get("dependency_id"):
            deps[str(item["dependency_id"])] = {**item, "dependency_type": "artifact"}
    for item in stack.get("personal_wikis", []) or []:
        if isinstance(item, dict) and item.get("dependency_id"):
            deps[str(item["dependency_id"])] = {**item, "dependency_type": "personal_wiki"}
    return deps


def build_source_binding(entry: Dict[str, Any], tier: int, group_index: Optional[int] = None) -> SourceBinding:
    raw_dependency_type = str(entry.get("dependency_type", ""))
    raw_dependency_id = str(entry.get("dependency_id", ""))
    raw_source_binding_id = str(entry.get("source_binding_id", ""))
    is_local_wiki = (
        raw_dependency_type == LOCAL_WIKI_TYPE
        or raw_dependency_id == LOCAL_MUTABLE_WIKI_ID
        or raw_source_binding_id == LOCAL_MUTABLE_WIKI_ID
    )
    source_binding_id = raw_source_binding_id or raw_dependency_id or (LOCAL_MUTABLE_WIKI_ID if is_local_wiki else "unnamed")
    dependency_id = raw_dependency_id or (LOCAL_MUTABLE_WIKI_ID if is_local_wiki else "")
    dependency_type = raw_dependency_type or (LOCAL_WIKI_TYPE if is_local_wiki else "artifact")
    effective_scope = str(entry.get("effective_scope") or ("local" if is_local_wiki else "scope-neutral"))
    authority_level = str(entry.get("authority_level") or ("working" if is_local_wiki else "reference"))
    return SourceBinding(
        source_binding_id=source_binding_id,
        dependency_id=dependency_id,
        dependency_type=dependency_type,
        effective_scope=effective_scope,
        authority_level=authority_level,
        tier=tier,
        group_index=group_index,
    )


def flatten_source_bindings(stack: Dict[str, Any]) -> List[SourceBinding]:
    order = (((stack.get("mutable_source_wiki_policy") or {}).get("source_binding_order")) or [])
    bindings: List[SourceBinding] = []
    tier = 0
    for entry in order:
        if not isinstance(entry, dict):
            tier += 1
            continue
        if "group" in entry:
            group = entry.get("group") or []
            for item in group:
                if isinstance(item, dict):
                    bindings.append(build_source_binding(item, tier, tier))
            tier += 1
        else:
            bindings.append(build_source_binding(entry, tier))
            tier += 1
    return bindings


def load_harness_config(root: Path) -> Dict[str, Any]:
    p = root / ".agent-harness" / "config.yaml"
    if p.exists():
        return load_yaml(p)
    return {}


def resolve_stack_path(root: Path, stack_path: Optional[Path] = None) -> Path:
    if stack_path:
        return stack_path

    # 1. Check if configured in harness config
    harness = load_harness_config(root)
    cfg_stack = (harness.get("llm_wiki_core") or {}).get("wiki_stack")
    if cfg_stack:
        p = Path(cfg_stack)
        if not p.is_absolute():
            p = root / p
        if p.exists():
            return p

    # 2. Prefer project-owned stack files. Legacy core-local files remain
    # readable for projects created before the root stack layout.
    root_stack = root / "wiki_stack.yaml"
    if root_stack.exists():
        return root_stack
    root_example = root / "wiki_stack.example.yaml"
    if root_example.exists():
        return root_example

    legacy_stack = root / "llm-wiki-core" / "wiki_stack.yaml"
    if legacy_stack.exists():
        return legacy_stack
    legacy_example = root / "llm-wiki-core" / "wiki_stack.example.yaml"
    if legacy_example.exists():
        return legacy_example

    # 3. Fallback
    return root / "wiki_stack.yaml"



def validate_stack(root: Path, stack_path: Optional[Path] = None) -> Dict[str, Any]:
    stack_path = resolve_stack_path(root, stack_path)
    stack = load_yaml(stack_path)
    errors: List[str] = []
    warnings: List[str] = []

    forbidden = validate_no_forbidden_keys(stack)
    if forbidden:
        errors.append("forbidden keys found: " + ", ".join(forbidden))

    deps = dependency_ids(stack)
    bindings = flatten_source_bindings(stack)
    seen_binding_ids = set()
    for b in bindings:
        if b.dependency_type != LOCAL_WIKI_TYPE and not b.dependency_id:
            errors.append(f"source binding {b.source_binding_id!r} has no dependency_id")
        if b.source_binding_id in seen_binding_ids:
            errors.append(f"duplicate source_binding_id: {b.source_binding_id}")
        seen_binding_ids.add(b.source_binding_id)
        if b.dependency_type != LOCAL_WIKI_TYPE and b.dependency_id and b.dependency_id not in deps:
            errors.append(f"source binding {b.source_binding_id!r} references unknown dependency_id {b.dependency_id!r}")
        if b.dependency_type not in {"artifact", "personal_wiki", LOCAL_WIKI_TYPE}:
            errors.append(f"source binding {b.source_binding_id!r} has invalid dependency_type {b.dependency_type!r}")

    for idx, entry in enumerate(((stack.get("mutable_source_wiki_policy") or {}).get("source_binding_order")) or []):
        if isinstance(entry, dict) and "group" in entry:
            group = entry.get("group") or []
            if len(group) < 2:
                warnings.append(f"source_binding_order[{idx}] group has fewer than 2 entries")

    return {
        "ok": not errors,
        "root": str(root),
        "stack_path": str(stack_path),
        "errors": errors,
        "warnings": warnings,
        "dependency_ids": sorted(deps),
        "source_bindings": [asdict(b) for b in bindings],
    }


def iter_markdown_pages(root: Path, rel_root: str) -> Iterable[Path]:
    base = (root / rel_root).expanduser()
    if not base.exists():
        return []
    return sorted([p for p in base.rglob("*.md") if p.is_file()])


def discover_sources(root: Path, stack: Dict[str, Any], warnings: List[str]) -> List[Tuple[SourceBinding, Path, str]]:
    """Return (binding, filesystem root, source kind label)."""
    result: List[Tuple[SourceBinding, Path, str]] = []
    deps = dependency_ids(stack)
    bindings = flatten_source_bindings(stack)
    seen_source_roots: set = set()

    def add_source(binding: SourceBinding, source_root: Path, source_kind: str) -> None:
        try:
            key = source_root.resolve()
        except Exception:
            key = source_root.absolute()
        if key in seen_source_roots:
            return
        seen_source_roots.add(key)
        result.append((binding, source_root, source_kind))

    harness = load_harness_config(root)
    local_wiki_root = (((harness.get("llm_wiki_core") or {}).get("local_wiki_root")) or "llm-wiki")
    local_path = root / local_wiki_root

    def binding_points_to_local_wiki_path(binding: SourceBinding) -> bool:
        if binding.dependency_type != "personal_wiki":
            return False
        dep = deps.get(binding.dependency_id)
        if not dep:
            return False
        p = Path(os.path.expanduser(str(dep.get("path", ""))))
        if not p.is_absolute():
            p = root / p
        try:
            return p.resolve() == local_path.resolve()
        except Exception:
            return p.absolute() == local_path.absolute()

    explicit_local_binding = any(b.dependency_type == LOCAL_WIKI_TYPE for b in bindings)
    local_path_bound_by_stack = explicit_local_binding or any(binding_points_to_local_wiki_path(b) for b in bindings)
    if local_path.exists() and not local_path_bound_by_stack:
        add_source(SourceBinding(
            source_binding_id=LOCAL_MUTABLE_WIKI_ID,
            dependency_id=LOCAL_MUTABLE_WIKI_ID,
            dependency_type=LOCAL_WIKI_TYPE,
            effective_scope="local",
            authority_level="working",
            tier=-1,
        ), local_path, LOCAL_WIKI_TYPE)

    for b in bindings:
        if b.dependency_type == LOCAL_WIKI_TYPE:
            if local_path.exists():
                add_source(b, local_path, LOCAL_WIKI_TYPE)
            else:
                warnings.append(f"local wiki path missing for {b.source_binding_id}: {local_path}")
            continue
        dep = deps.get(b.dependency_id)
        if not dep:
            continue
        if b.dependency_type == "personal_wiki":
            p = Path(os.path.expanduser(str(dep.get("path", ""))))
            if not p.is_absolute():
                p = root / p
            if p.exists():
                add_source(b, p, "personal_wiki")
            else:
                warnings.append(f"personal wiki path missing for {b.dependency_id}: {p}")
        elif b.dependency_type == "artifact":
            ref = str(dep.get("artifact_ref", ""))
            p = Path(os.path.expanduser(ref))
            if not p.is_absolute():
                p = root / p
            if p.is_dir():
                add_source(b, p, "artifact_dir")
            elif p.exists() and any(p.name.endswith(ext) for ext in ARCHIVE_EXTENSIONS):
                add_source(b, p, "artifact_file")
            elif p.exists():
                warnings.append(f"artifact file exists but format is not supported: {p}")
            else:
                warnings.append(f"artifact ref missing for {b.dependency_id}: {p}")
    return result


@dataclass
class ArchiveMember:
    name: str
    size: int
    is_file: bool


def list_archive_members(archive_path: Path) -> List[ArchiveMember]:
    if any(archive_path.name.endswith(ext) for ext in ZIP_ARCHIVE_EXTENSIONS):
        import zipfile
        try:
            with zipfile.ZipFile(archive_path, "r") as zf:
                return [
                    ArchiveMember(name=i.filename, size=i.file_size, is_file=not i.is_dir())
                    for i in zf.infolist()
                ]
        except Exception:
            return []

    exts = [".tar.gz", ".tgz", ".tar"]
    if any(archive_path.name.endswith(ext) for ext in exts):
        import tarfile
        try:
            with tarfile.open(archive_path, "r") as tar:
                return [
                    ArchiveMember(name=m.name, size=m.size, is_file=m.isfile())
                    for m in tar.getmembers()
                ]
        except Exception:
            pass
    # Fallback to subprocess tar
    import subprocess
    try:
        res = subprocess.run(["tar", "-tf", str(archive_path)], capture_output=True, text=True, check=True)
        members = []
        for line in res.stdout.splitlines():
            line = line.strip()
            if not line:
                continue
            is_file = not line.endswith("/")
            members.append(ArchiveMember(name=line, size=0, is_file=is_file))
        return members
    except Exception:
        return []


def read_archive_member(archive_path: Path, member_name: str) -> bytes:
    if any(archive_path.name.endswith(ext) for ext in ZIP_ARCHIVE_EXTENSIONS):
        import zipfile
        with zipfile.ZipFile(archive_path, "r") as zf:
            return zf.read(member_name)

    exts = [".tar.gz", ".tgz", ".tar"]
    if any(archive_path.name.endswith(ext) for ext in exts):
        import tarfile
        try:
            with tarfile.open(archive_path, "r") as tar:
                member = tar.getmember(member_name)
                f = tar.extractfile(member)
                if f:
                    try:
                        return f.read()
                    finally:
                        f.close()
        except Exception:
            pass
    # Fallback to subprocess tar
    import subprocess
    try:
        res = subprocess.run(["tar", "-O", "-xf", str(archive_path), member_name], capture_output=True, check=True)
        return res.stdout
    except Exception as e:
        raise FileNotFoundError(f"Failed to read archive member {member_name} from {archive_path}: {e}")


def read_page_content(root: Path, p: PageRecord, stack: Dict[str, Any]) -> str:
    fs_path = root / p.path
    if fs_path.exists() and fs_path.is_file():
        return fs_path.read_text(errors="replace")
    
    deps = dependency_ids(stack)
    dep = deps.get(p.dependency_id)
    if dep and dep.get("dependency_type") == "artifact":
        ref = str(dep.get("artifact_ref", ""))
        archive_path = Path(os.path.expanduser(ref))
        if not archive_path.is_absolute():
            archive_path = root / archive_path
        
        if archive_path.exists() and archive_path.is_file():
            members = list_archive_members(archive_path)
            for m in members:
                if not m.is_file:
                    continue
                clean_name = m.name.lstrip("/")
                stem = archive_path.name
                for ext in ARCHIVE_EXTENSIONS:
                    if stem.endswith(ext):
                        stem = stem[:-len(ext)]
                        break
                virtual_path = archive_path.parent / stem / clean_name
                rel = rel_to(virtual_path, root)
                if rel == p.path:
                    content_bytes = read_archive_member(archive_path, m.name)
                    return content_bytes.decode("utf-8", errors="replace")
                    
    raise FileNotFoundError(f"Page content not found for {p.path}")


def build_page_record_from_data(root: Path, rel_path: str, text: str, sha256_val: str, binding: SourceBinding, selected: bool = True) -> PageRecord:
    fm, body = read_frontmatter(text)
    title = str(fm.get("title") or next((line.lstrip("# ").strip() for line in body.splitlines() if line.startswith("#")), Path(rel_path).stem))
    status = str(fm.get("status") or "accepted")
    tags = normalize_list(fm.get("tags"))
    sources = normalize_list(fm.get("sources"))
    source_hashes = fm.get("source_hashes") if isinstance(fm.get("source_hashes"), dict) else {}
    review_needed = bool(fm.get("review_needed", False))
    stale = bool(fm.get("stale", False))
    confidence = str(fm.get("confidence", "medium"))
    warnings: List[str] = []
    score = 1.0
    if path_matches_glob(rel_path, DEFAULT_EXCLUDE_GLOBS):
        selected = False
        warnings.append("excluded_instruction_or_schema_path")
        score = 0.0
    elif binding.dependency_type != "artifact":
        if review_needed:
            warnings.append("review_needed")
            score -= 0.25
        if stale:
            warnings.append("stale")
            score -= 0.25
        if status in DEFAULT_STATUS_EXCLUDE:
            selected = False
            warnings.append(f"excluded_status:{status}")
            score = 0.0
    return PageRecord(
        page_id=sha256_text(f"{binding.source_binding_id}:{rel_path}")[:16],
        path=rel_path,
        source_binding_id=binding.source_binding_id,
        dependency_id=binding.dependency_id,
        effective_scope=binding.effective_scope,
        authority_level=binding.authority_level,
        title=title,
        status=status,
        tags=tags,
        sources=sources,
        source_hashes={str(k): str(v) for k, v in source_hashes.items()},
        review_needed=review_needed,
        stale=stale,
        confidence=confidence,
        sha256=sha256_val,
        selected=selected,
        score=max(0.0, score),
        warnings=warnings,
        body_sha256=markdown_body_sha256(text),
    )


def build_page_record(root: Path, source_root: Path, page: Path, binding: SourceBinding, selected: bool = True) -> PageRecord:
    raw = page.read_bytes()
    text = raw.decode("utf-8", errors="replace")
    rel = rel_to(page, root)
    sha256_val = hashlib.sha256(raw).hexdigest()
    return build_page_record_from_data(root, rel, text, sha256_val, binding, selected)


def collect_pages(root: Path, stack: Dict[str, Any], _content_cache: Optional[Dict[str, str]] = None) -> Tuple[List[PageRecord], List[str]]:
    warnings: List[str] = []
    pages: List[PageRecord] = []
    for binding, source_root, source_kind in discover_sources(root, stack, warnings):
        if source_kind == "artifact_file":
            members = list_archive_members(source_root)
            for member in sorted(members, key=lambda m: m.name):
                if not member.is_file or not member.name.endswith(".md"):
                    continue
                clean_name = member.name.lstrip("/")
                stem = source_root.name
                for ext in ARCHIVE_EXTENSIONS:
                    if stem.endswith(ext):
                        stem = stem[:-len(ext)]
                        break
                virtual_path = source_root.parent / stem / clean_name
                rel = rel_to(virtual_path, root)
                if path_matches_glob(rel, DEFAULT_EXCLUDE_GLOBS):
                    continue
                try:
                    content_bytes = read_archive_member(source_root, member.name)
                    text = content_bytes.decode("utf-8", errors="replace")
                    sha256_val = hashlib.sha256(content_bytes).hexdigest()
                    if _content_cache is not None:
                        _content_cache[rel] = text
                    pages.append(build_page_record_from_data(root, rel, text, sha256_val, binding))
                except Exception as e:
                    warnings.append(f"failed to read archive member {member.name} from {source_root.name}: {e}")
        else:
            for p in sorted(source_root.rglob("*.md")):
                if not p.is_file():
                    continue
                rel = rel_to(p, root)
                if path_matches_glob(rel, DEFAULT_EXCLUDE_GLOBS):
                    continue
                raw = p.read_bytes()
                text = raw.decode("utf-8", errors="replace")
                sha256_val = hashlib.sha256(raw).hexdigest()
                if _content_cache is not None:
                    _content_cache[rel] = text
                pages.append(build_page_record_from_data(root, rel, text, sha256_val, binding))
    return pages, warnings


def conflict_title_key(title: str) -> str:
    return str(title or "").lower()


def conflict_record_key(conflict_type: str, title: str) -> str:
    return f"{conflict_type}:{conflict_title_key(title)}"


def conflict_content_hash(page: PageRecord) -> str:
    return page.sha256


def is_conflict_governed_scope(scope: str, policy: Optional[ConflictDetectionPolicy] = None) -> bool:
    active_policy = policy or ConflictDetectionPolicy(governed_scopes=set(CONFLICT_GOVERNED_SCOPES))
    return str(scope) in active_policy.governed_scopes


def conflict_page_entry(page: PageRecord) -> Dict[str, Any]:
    return {
        "path": page.path,
        "scope": page.effective_scope,
        "authority": page.authority_level,
        "sha256": page.sha256,
    }


def build_conflict_record(
    conflict_type: str,
    pages: List[PageRecord],
    policy: Optional[ConflictDetectionPolicy] = None,
) -> Dict[str, Any]:
    title = pages[0].title if pages else ""
    return {
        "type": conflict_type,
        "title": title,
        "default_action": CONFLICT_DEFAULT_ACTION_ASK_USER,
        "pages": [conflict_page_entry(p) for p in pages],
    }


def find_conflict_by_key(conflicts: List[Dict[str, Any]], conflict_key: str) -> Optional[Dict[str, Any]]:
    return next(
        (
            c for c in conflicts
            if (c.get("conflict_key") or conflict_record_key(str(c.get("type") or ""), str(c.get("title") or ""))) == conflict_key
        ),
        None,
    )


def find_conflict_by_title(conflicts: List[Dict[str, Any]], title: str) -> Optional[Dict[str, Any]]:
    title_key = conflict_title_key(title)
    return next((c for c in conflicts if conflict_title_key(str(c.get("title") or "")) == title_key), None)


def detect_conflicts(pages: List[PageRecord], policy: Optional[ConflictDetectionPolicy] = None) -> List[Dict[str, Any]]:
    active_policy = policy or ConflictDetectionPolicy(governed_scopes=set(CONFLICT_GOVERNED_SCOPES))
    by_title: Dict[str, List[PageRecord]] = {}
    for p in pages:
        if p.selected:
            by_title.setdefault(conflict_title_key(p.title), []).append(p)
    conflicts = []
    for title, group in by_title.items():
        # Older stacks may still bind one physical page through more than one
        # source. Collapse by path so conflict handling never double-counts it.
        unique_by_path: Dict[str, PageRecord] = {}
        for p in group:
            unique_by_path[p.path] = p
        group = list(unique_by_path.values())
        hashes = {conflict_content_hash(p) for p in group}
        scopes = {p.effective_scope for p in group}
        if len(group) > 1 and len(hashes) > 1 and any(is_conflict_governed_scope(s, active_policy) for s in scopes):
            conflicts.append(build_conflict_record(CONFLICT_TYPE_SAME_TITLE, group, active_policy))
    return conflicts


def normalize_duplicate_title(title: str) -> str:
    lowered = title.lower().strip()
    normalized = re.sub(r"[^a-z0-9가-힣]+", " ", lowered)
    return re.sub(r"\s+", " ", normalized).strip()


def normalize_duplicate_source(source: str) -> str:
    value = source.strip().replace("\\", "/")
    value = re.sub(r"^\./+", "", value)
    return value.lower()


def page_dependency_type(page: PageRecord, binding_by_id: Dict[str, Dict[str, Any]]) -> str:
    if page.source_binding_id == LOCAL_MUTABLE_WIKI_ID:
        return LOCAL_WIKI_TYPE
    return str(binding_by_id.get(page.source_binding_id, {}).get("dependency_type") or page.dependency_id)


def is_local_working_page(page: PageRecord, binding_by_id: Dict[str, Dict[str, Any]]) -> bool:
    return page.source_binding_id == LOCAL_MUTABLE_WIKI_ID or page_dependency_type(page, binding_by_id) == LOCAL_WIKI_TYPE


def is_artifact_reference_page(page: PageRecord, binding_by_id: Dict[str, Dict[str, Any]]) -> bool:
    return page_dependency_type(page, binding_by_id) == "artifact"


def is_duplicate_detection_page(page: PageRecord) -> bool:
    if page.selected:
        return True
    return not any(
        warning == "excluded_instruction_or_schema_path" or warning.startswith("excluded_status:")
        for warning in page.warnings
    )


def duplicate_page_entry(page: PageRecord, binding_by_id: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    return {
        "path": page.path,
        "source_binding_id": page.source_binding_id,
        "dependency_type": page_dependency_type(page, binding_by_id),
        "effective_scope": page.effective_scope,
        "authority_level": page.authority_level,
        "title": page.title,
        "sha256": page.sha256,
        "body_sha256": page.body_sha256,
    }


def build_duplicate_candidate(
    candidate_type: str,
    local_pages: List[PageRecord],
    artifact_pages: List[PageRecord],
    binding_by_id: Dict[str, Dict[str, Any]],
    evidence: Dict[str, Any],
    selection_effect: str = DUPLICATE_SELECTION_EFFECT_NONE,
    risk: str = DUPLICATE_RISK_LOCAL_DIFFERS_FROM_REFERENCE,
) -> Dict[str, Any]:
    primary = sorted(local_pages, key=lambda p: (p.path, p.source_binding_id))[0]
    references = sorted(artifact_pages, key=lambda p: (p.path, p.source_binding_id))
    pages = sorted(local_pages + artifact_pages, key=lambda p: (p.path, p.source_binding_id))
    stable_key = "|".join(
        [candidate_type, f"{primary.source_binding_id}:{primary.path}"]
        + [f"{p.source_binding_id}:{p.path}" for p in references]
    )
    candidate_id = "duplicate-candidate-" + sha256_text(stable_key)[:12]
    return {
        "id": candidate_id,
        "type": candidate_type,
        "recommendation": DUPLICATE_RECOMMENDATION_LOCAL_PRIMARY_REFERENCE_ARTIFACT,
        "primary_candidate": primary.path,
        "reference_candidates": [p.path for p in references],
        "pages": [duplicate_page_entry(p, binding_by_id) for p in pages],
        "evidence": evidence,
        "risk": risk,
        "selection_effect": selection_effect,
        "suggested_local_captures": {
            "decision": f"llm-wiki/decisions/{candidate_id}.md",
            "experiment": f"llm-wiki/experiments/{candidate_id}-local-observation.md",
            "lesson": f"llm-wiki/lessons/{candidate_id}.md",
        },
    }


def detect_duplicate_candidates(
    pages: List[PageRecord],
    source_bindings: List[Dict[str, Any]],
    resolved_conflicts: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    binding_by_id = {str(b.get("source_binding_id")): b for b in source_bindings}
    unique_by_identity: Dict[Tuple[str, str], PageRecord] = {}
    for page in pages:
        if is_duplicate_detection_page(page):
            unique_by_identity[(page.source_binding_id, page.path)] = page
    selected_pages = list(unique_by_identity.values())
    resolved_conflict_titles = {
        conflict_title_key(str(item.get("title") or ""))
        for item in (resolved_conflicts or [])
        if item.get("title")
    }
    items: List[Dict[str, Any]] = []
    emitted_paths: set[Tuple[str, ...]] = set()

    def add_candidate(candidate: Dict[str, Any]) -> None:
        key = tuple(sorted(f"{p['source_binding_id']}:{p['path']}" for p in candidate["pages"]))
        if key in emitted_paths:
            return
        emitted_paths.add(key)
        items.append(candidate)

    by_hash: Dict[str, List[PageRecord]] = {}
    for page in selected_pages:
        by_hash.setdefault(page.body_sha256 or page.sha256, []).append(page)
    for body_sha256, group in by_hash.items():
        local_pages = [p for p in group if is_local_working_page(p, binding_by_id)]
        artifact_pages = [p for p in group if is_artifact_reference_page(p, binding_by_id)]
        if local_pages and artifact_pages:
            add_candidate(build_duplicate_candidate(
                DUPLICATE_TYPE_EXACT,
                local_pages,
                artifact_pages,
                binding_by_id,
                {"body_sha256": body_sha256},
            ))

    by_title: Dict[str, List[PageRecord]] = {}
    for page in selected_pages:
        key = normalize_duplicate_title(page.title)
        if key:
            by_title.setdefault(key, []).append(page)
    for title_key, group in by_title.items():
        local_pages = [p for p in group if is_local_working_page(p, binding_by_id)]
        artifact_pages = [p for p in group if is_artifact_reference_page(p, binding_by_id)]
        hashes = {p.sha256 for p in group}
        if local_pages and artifact_pages and len(hashes) > 1:
            exact_title_keys = {conflict_title_key(p.title) for p in local_pages + artifact_pages}
            selection_effect = DUPLICATE_SELECTION_EFFECT_NONE
            if len(exact_title_keys) == 1 and next(iter(exact_title_keys)) in resolved_conflict_titles:
                selection_effect = DUPLICATE_SELECTION_EFFECT_HANDLED_BY_CONFLICT_RESOLUTION
            add_candidate(build_duplicate_candidate(
                DUPLICATE_TYPE_SAME_TITLE_DIVERGENCE,
                local_pages,
                artifact_pages,
                binding_by_id,
                {"normalized_title_key": title_key},
                selection_effect=selection_effect,
                risk=DUPLICATE_RISK_LOCAL_ARTIFACT_TITLE_DIVERGENCE,
            ))

    for local_page in sorted([p for p in selected_pages if is_local_working_page(p, binding_by_id)], key=lambda p: p.path):
        local_sources = {normalize_duplicate_source(s) for s in local_page.sources if normalize_duplicate_source(s)}
        if not local_sources:
            continue
        matching_artifacts: List[PageRecord] = []
        shared_sources: set[str] = set()
        for artifact_page in sorted([p for p in selected_pages if is_artifact_reference_page(p, binding_by_id)], key=lambda p: p.path):
            artifact_sources = {normalize_duplicate_source(s) for s in artifact_page.sources if normalize_duplicate_source(s)}
            overlap = local_sources & artifact_sources
            if overlap:
                matching_artifacts.append(artifact_page)
                shared_sources.update(overlap)
        if matching_artifacts:
            add_candidate(build_duplicate_candidate(
                DUPLICATE_TYPE_SAME_SOURCES,
                [local_page],
                matching_artifacts,
                binding_by_id,
                {
                    "shared_sources": sorted(shared_sources),
                    "normalized_title_key": normalize_duplicate_title(local_page.title),
                },
            ))

    items = sorted(items, key=lambda item: (item["type"], item["primary_candidate"], item["reference_candidates"]))
    return {
        "schema_version": DUPLICATE_CANDIDATES_SCHEMA_VERSION,
        "generated_at": now_iso(),
        "items": items,
    }


def duplicate_candidate_warnings(duplicate_candidates: Dict[str, Any]) -> List[str]:
    warnings: List[str] = []
    for item in duplicate_candidates.get("items", []):
        warnings.append(
            f"duplicate_candidate:{item.get('type')} primary={item.get('primary_candidate')} "
            f"references={len(item.get('reference_candidates') or [])}"
        )
    return warnings


def render_context_bundle(
    root: Path,
    pages: List[PageRecord],
    warnings: List[str],
    conflicts: List[Dict[str, Any]],
    resolved_conflicts: Optional[List[Dict[str, Any]]] = None,
    duplicate_candidates: Optional[Dict[str, Any]] = None,
) -> str:
    resolved_conflicts = resolved_conflicts or []
    selected = [p for p in pages if p.selected]
    lines = [
        "# llm-wiki-core Context Bundle",
        "",
        f"Generated: {now_iso()}",
        f"Root: `{root}`",
        "",
        "## Boundary",
        "",
        "This bundle is a derived run artifact. It is not canonical wiki truth.",
        "Project/Team changes require promotion review and new artifact publish.",
        "",
        "## Warnings",
        "",
    ]
    if warnings or conflicts:
        for w in warnings:
            lines.append(f"- {w}")
        for c in conflicts:
            lines.append(f"- conflict:{c.get('type')} title={c.get('title')} default_action={c.get('default_action')}")
    else:
        lines.append("- none")
    if resolved_conflicts:
        lines += ["", "## Resolved Conflicts", ""]
        for r in resolved_conflicts:
            lines.append(f"- title={r.get('title')} chosen={r.get('chosen')} rule={r.get('rule')}")
            if r.get("note"):
                lines.append(f"  note: {r['note']}")
    duplicate_items = (duplicate_candidates or {}).get("items", [])
    if duplicate_items:
        lines += ["", "## Duplicate Candidates", ""]
        for item in duplicate_items[:10]:
            lines.append(
                f"- type={item.get('type')} primary={item.get('primary_candidate')} "
                f"references={len(item.get('reference_candidates') or [])} "
                f"risk={item.get('risk')} selection_effect={item.get('selection_effect')}"
            )
    lines += ["", "## Selected Pages", ""]
    if not selected:
        lines.append("No selected pages.")
    for p in selected:
        lines.append(f"### {p.title}")
        lines.append(f"- path: `{p.path}`")
        lines.append(f"- scope: `{p.effective_scope}` / authority: `{p.authority_level}`")
        lines.append(f"- status: `{p.status}` / confidence: `{p.confidence}` / score: `{p.score:.2f}`")
        if p.warnings:
            lines.append(f"- warnings: {', '.join(p.warnings)}")
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def get_bundle_retention_count(harness: Dict[str, Any]) -> int:
    llm_cfg = harness.get("llm_wiki_core") or {}
    minimum = parse_positive_int(llm_cfg.get("bundle_retention_min_protected"), MIN_BUNDLE_RETENTION_COUNT)
    count = parse_positive_int(llm_cfg.get("bundle_retention_count"), DEFAULT_BUNDLE_RETENTION_COUNT)
    return max(count, minimum, MIN_BUNDLE_RETENTION_COUNT)


def find_prunable_bundle_runs(bundle_base: Path) -> List[Path]:
    if not bundle_base.exists() or not bundle_base.is_dir():
        return []
    return sorted(
        [
            d for d in bundle_base.iterdir()
            if d.is_dir() and BUNDLE_RUN_NAME_RE.match(d.name)
        ],
        key=lambda d: d.name,
    )


def prune_bundle_runs(bundle_base: Path, keep_count: int, protect: Optional[Path] = None) -> List[str]:
    runs = find_prunable_bundle_runs(bundle_base)
    effective_keep = max(keep_count, MIN_BUNDLE_RETENTION_COUNT)
    if len(runs) <= effective_keep:
        return []

    protected = protect.resolve() if protect is not None else None
    keep = {d.resolve() for d in runs[-effective_keep:]}
    if protected is not None:
        keep.add(protected)

    pruned: List[str] = []
    for run_dir in runs:
        resolved = run_dir.resolve()
        if resolved in keep:
            continue
        shutil.rmtree(run_dir)
        pruned.append(str(run_dir))
    return pruned


def get_pending_capture_retention_count(harness: Dict[str, Any]) -> int:
    llm_cfg = harness.get("llm_wiki_core") or {}
    minimum = parse_positive_int(
        llm_cfg.get("pending_capture_retention_min_protected"),
        MIN_PENDING_CAPTURE_RETENTION_COUNT,
    )
    count = parse_positive_int(
        llm_cfg.get("pending_capture_retention_count"),
        DEFAULT_PENDING_CAPTURE_RETENTION_COUNT,
    )
    return max(count, minimum, MIN_PENDING_CAPTURE_RETENTION_COUNT)


def find_prunable_pending_captures(capture_base: Path) -> List[Path]:
    if not capture_base.exists() or not capture_base.is_dir():
        return []
    return sorted(
        [
            p for p in capture_base.iterdir()
            if p.is_file() and PENDING_CAPTURE_NAME_RE.match(p.name)
        ],
        key=lambda p: p.name,
    )


def prune_pending_captures(capture_base: Path, keep_count: int, protect: Optional[Path] = None) -> List[str]:
    captures = find_prunable_pending_captures(capture_base)
    effective_keep = max(keep_count, MIN_PENDING_CAPTURE_RETENTION_COUNT)
    if len(captures) <= effective_keep:
        return []

    protected = protect.resolve() if protect is not None else None
    keep = {p.resolve() for p in captures[-effective_keep:]}
    if protected is not None:
        keep.add(protected)

    pruned: List[str] = []
    for capture in captures:
        resolved = capture.resolve()
        if resolved in keep:
            continue
        capture.unlink()
        pruned.append(str(capture))
    return pruned


def get_max_source_mtime(root: Path, stack: Dict[str, Any], stack_path: Path) -> float:
    mtimes = [stack_path.stat().st_mtime] if stack_path.exists() else [0.0]
    harness_path = root / ".agent-harness" / "config.yaml"
    if harness_path.exists():
        mtimes.append(harness_path.stat().st_mtime)
    decisions_path = root / CONFLICT_DECISIONS_REL_PATH
    if decisions_path.exists():
        mtimes.append(decisions_path.stat().st_mtime)

    warnings: List[str] = []
    for binding, source_root, source_kind in discover_sources(root, stack, warnings):
        if not source_root.exists():
            continue
        if source_kind == "artifact_file":
            mtimes.append(source_root.stat().st_mtime)
        else:
            mtimes.append(source_root.stat().st_mtime)
            try:
                for p in source_root.rglob("*.md"):
                    if p.is_file():
                        mtimes.append(p.stat().st_mtime)
            except Exception:
                pass
    return max(mtimes) if mtimes else 0.0


def find_orphaned_sources(root: Path) -> List[str]:
    sources_dir = root / "llm-wiki" / "sources"
    log_path = root / "llm-wiki" / "log.md"
    if not sources_dir.is_dir():
        return []
    log_text = log_path.read_text(encoding="utf-8") if log_path.exists() else ""
    orphans: List[str] = []
    seen: set = set()
    for page in sorted(sources_dir.glob("*.md")):
        try:
            fm, _ = read_frontmatter(page.read_text(encoding="utf-8", errors="replace"))
        except Exception:
            continue
        for src in normalize_list(fm.get("sources")):
            name = Path(src).name
            if not name or name in seen:
                continue
            pattern = r"ingest\s*\|\s*(raw/)?" + re.escape(name) + r"\b"
            if not re.search(pattern, log_text):
                seen.add(name)
                orphans.append(f"orphaned_source: {name}")
    return orphans


def md5_bytes(data: bytes) -> str:
    return hashlib.md5(data).hexdigest()


def raw_item_id(path: Path) -> str:
    name = path.name
    return name[:-3] if name.endswith(".md") else name


def iter_raw_files(raw_item: Path) -> List[Path]:
    if raw_item.is_file():
        return [raw_item]
    files: List[Path] = []
    for p in sorted(raw_item.rglob("*")):
        if p.is_file():
            files.append(p)
    return files


def raw_relative_path(root: Path, path: Path) -> str:
    return rel_to(path, root)


def should_exclude_raw_derived_path(rel_to_raw_item: str) -> bool:
    return path_matches_glob(rel_to_raw_item, RAW_DERIVED_EXCLUDE_GLOBS)


def hash_raw_tree(root: Path, raw_item: Path) -> Dict[str, Any]:
    files = iter_raw_files(raw_item)
    h = hashlib.md5()
    total_size = 0
    included_count = 0
    for p in files:
        raw_rel = p.relative_to(raw_item).as_posix() if raw_item.is_dir() else p.name
        if should_exclude_raw_derived_path(raw_rel):
            continue
        data = p.read_bytes()
        total_size += len(data)
        included_count += 1
        h.update(raw_rel.encode("utf-8"))
        h.update(b"\0")
        h.update(hashlib.md5(data).hexdigest().encode("ascii"))
        h.update(b"\0")
    return {
        "file_count": len(files),
        "included_file_count": included_count,
        "total_size_bytes": total_size,
        "tree_hash": "md5:" + h.hexdigest(),
        "hash_algorithm": "md5-tree-v1",
    }


def classify_raw_item(raw_item: Path) -> Dict[str, str]:
    if raw_item.is_file():
        return {"kind": "raw_file", "source_type": "document_snapshot", "mutability": "updatable_snapshot"}
    package_files = list(raw_item.rglob("package.xml"))
    if package_files:
        return {"kind": "project_tree", "source_type": "source_code_snapshot", "mutability": "updatable_snapshot"}
    return {"kind": "document_collection", "source_type": "document_snapshot", "mutability": "updatable_snapshot"}


def build_divider_units(root: Path, raw_item: Path, item_id: str) -> List[Dict[str, Any]]:
    if raw_item.is_file():
        return []
    units: List[Dict[str, Any]] = [{
        "id": "project_overview",
        "kind": "project_overview",
        "purpose": "high-level structure and runtime map",
        "raw_selectors": ["package.xml", "src/**/package.xml", "src/**/launch/**", "src/**/config/**"],
    }]
    package_files = sorted(raw_item.rglob("package.xml"))
    for package_xml in package_files:
        rel_parent = package_xml.parent.relative_to(raw_item).as_posix()
        if should_exclude_raw_derived_path(rel_parent + "/package.xml"):
            continue
        package_name = package_xml.parent.name
        units.append({
            "id": "package_" + re.sub(r"[^A-Za-z0-9_]+", "_", package_name).strip("_").lower(),
            "kind": "ros2_package",
            "purpose": f"ROS2 package summary for {package_name}",
            "source_path": rel_parent,
            "raw_selectors": [rel_parent + "/**"],
        })
    return units


def discover_raw_items(root: Path) -> List[Dict[str, Any]]:
    raw_dir = root / "llm-wiki" / "raw"
    if not raw_dir.is_dir():
        return []
    items: List[Dict[str, Any]] = []
    for item in sorted(raw_dir.iterdir(), key=lambda p: p.name):
        if item.name.startswith("."):
            continue
        item_id = raw_item_id(item)
        classification = classify_raw_item(item)
        items.append({
            "id": item_id,
            "raw_path": raw_relative_path(root, item),
            "derived_path": f"llm-wiki/raw-derived/{item_id}",
            **classification,
            "status": "active",
        })
    return items


def build_raw_derived_plan(root: Path) -> Dict[str, Any]:
    discovered = discover_raw_items(root)
    registry_items = []
    manifests: Dict[str, Any] = {}
    divider_items: Dict[str, Any] = {}
    for item in discovered:
        if item["kind"] == "raw_file":
            continue
        raw_path = root / item["raw_path"]
        registry_items.append({
            "id": item["id"],
            "raw_path": item["raw_path"],
            "derived_path": item["derived_path"],
            "kind": item["kind"],
            "status": item["status"],
        })
        manifests[item["id"]] = {
            "schema_version": RAW_DERIVED_MANIFEST_SCHEMA_VERSION,
            "id": item["id"],
            "raw_path": item["raw_path"],
            "kind": item["kind"],
            "source_type": item["source_type"],
            "mutability": item["mutability"],
            "revision": {
                "revision_id": datetime.now().strftime("%Y-%m-%d-local"),
                "captured_at": now_iso(),
                "previous_revision_id": None,
            },
            "fingerprint": hash_raw_tree(root, raw_path),
            "outputs": {
                "divider": "divider.yaml",
                "inventory": "inventory/files.yaml",
                "prepared_root": "prepared/",
                "lineage": "lineage.yaml",
            },
        }
        divider_items[item["id"]] = {
            "schema_version": RAW_DIVIDER_SCHEMA_VERSION,
            "raw_item": item["id"],
            "strategy": item["kind"],
            "exclude": RAW_DERIVED_EXCLUDE_GLOBS,
            "units": build_divider_units(root, raw_path, item["id"]),
        }
    return {
        "registry": {"schema_version": RAW_DERIVED_SCHEMA_VERSION, "items": registry_items},
        "manifests": {"schema_version": "raw-derived-manifests/v1", "items": manifests},
        "divider": {"schema_version": "raw-divider-plan/v1", "items": divider_items},
    }


def list_raw_items(root: Path) -> Dict[str, Any]:
    return {"items": discover_raw_items(root)}


def get_raw_derived_manifest(root: Path, raw_id: str) -> Dict[str, Any]:
    plan = build_raw_derived_plan(root)
    manifests = plan["manifests"]["items"]
    dividers = plan["divider"]["items"]
    if raw_id not in manifests:
        return {
            "found": False,
            "raw_id": raw_id,
            "message": f"No raw-derived manifest found for {raw_id!r}.",
        }
    return {
        "found": True,
        "raw_id": raw_id,
        "manifest": manifests[raw_id],
        "divider": dividers.get(raw_id),
    }


def load_conflict_decisions(root: Path) -> Dict[str, Dict[str, Any]]:
    path = root / CONFLICT_DECISIONS_REL_PATH
    if not path.exists():
        return {}
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def save_conflict_decision(root: Path, title: str, chosen_path: str) -> None:
    path = root / CONFLICT_DECISIONS_REL_PATH
    path.parent.mkdir(parents=True, exist_ok=True)
    decisions = load_conflict_decisions(root)
    decisions[conflict_title_key(title)] = {"chosen_path": chosen_path, "decided_at": now_iso()}
    path.write_text(dump_yaml(decisions))


def resolve_conflicts_by_priority(
    root: Path,
    pages: List[PageRecord],
    conflicts: List[Dict[str, Any]],
    tier_by_binding: Dict[str, int],
    decisions: Dict[str, Dict[str, Any]],
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    pages_by_path = {p.path: p for p in pages}
    remaining: List[Dict[str, Any]] = []
    resolved: List[Dict[str, Any]] = []

    for c in conflicts:
        entries = c.get("pages") or []
        title = c.get("title", "")
        if len(entries) < 2:
            remaining.append(c)
            continue

        decision = decisions.get(conflict_title_key(title))
        if decision and decision.get("chosen_path") in {e["path"] for e in entries}:
            chosen_path = decision["chosen_path"]
            rule = RESOLUTION_RULE_USER_DECISION
        else:
            ranked = sorted(
                entries,
                key=lambda e: tier_by_binding.get(pages_by_path[e["path"]].source_binding_id, float("inf")),
            )
            top_tier = tier_by_binding.get(pages_by_path[ranked[0]["path"]].source_binding_id, float("inf"))
            second_tier = tier_by_binding.get(pages_by_path[ranked[1]["path"]].source_binding_id, float("inf"))
            if top_tier == second_tier:
                remaining.append(c)
                continue
            chosen_path = ranked[0]["path"]
            rule = RESOLUTION_RULE_PRIORITY

        rejected_paths = [e["path"] for e in entries if e["path"] != chosen_path]
        rejected_set = set(rejected_paths)
        for p in pages:
            if p.path in rejected_set:
                p.selected = False

        resolution: Dict[str, Any] = {
            "title": title,
            "chosen": chosen_path,
            "rejected": rejected_paths,
            "rule": rule,
            "command": f'llm-wiki-core/scripts/llm-wiki-core --root . show-conflict "{title}"',
        }
        if rule == RESOLUTION_RULE_PRIORITY:
            try:
                chosen_mtime = (root / chosen_path).stat().st_mtime
                newer_rejected = [
                    p for p in rejected_paths
                    if (root / p).exists() and (root / p).stat().st_mtime > chosen_mtime
                ]
            except Exception:
                newer_rejected = []
            if newer_rejected:
                resolution["note"] = (
                    "주의: 제외된 페이지가 선택된 페이지보다 더 최근에 수정되었습니다 - 검토를 권장합니다."
                )
        resolved.append(resolution)

    return remaining, resolved


def get_bundle(root: Path, output_dir: Optional[Path] = None) -> Dict[str, Any]:
    stack_path = resolve_stack_path(root)
    stack = load_yaml(stack_path)
    explicit_output = output_dir is not None

    # Try to reuse the latest generated bundle if files haven't changed
    harness = load_harness_config(root)
    default_out = (((harness.get("llm_wiki_core") or {}).get("bundle_output_dir")) or ".agent-harness/bundles")
    bundle_base = root / default_out
    retention_count = get_bundle_retention_count(harness)

    if bundle_base.exists() and bundle_base.is_dir():
        runs = find_prunable_bundle_runs(bundle_base)
        if runs:
            latest_run = runs[-1]
            cb_path = latest_run / "context_bundle.md"
            if cb_path.exists():
                bundle_mtime = cb_path.stat().st_mtime
                max_source_mtime = get_max_source_mtime(root, stack, stack_path)
                if bundle_mtime >= max_source_mtime:
                    try:
                        selected_pages = yaml.safe_load((latest_run / "selected_pages.yaml").read_text(encoding="utf-8"))
                        warnings_data = yaml.safe_load((latest_run / "warnings.yaml").read_text(encoding="utf-8"))
                        conflicts_data = yaml.safe_load((latest_run / "conflict_warnings.yaml").read_text(encoding="utf-8"))
                        resolved_conflicts_data = yaml.safe_load((latest_run / "resolved_conflicts.yaml").read_text(encoding="utf-8"))
                        selected_pages = selected_pages if isinstance(selected_pages, list) else []
                        warnings_data = warnings_data if isinstance(warnings_data, list) else []
                        conflicts_data = conflicts_data if isinstance(conflicts_data, list) else []
                        resolved_conflicts_data = resolved_conflicts_data if isinstance(resolved_conflicts_data, list) else []
                        duplicate_candidates_data = {
                            "schema_version": DUPLICATE_CANDIDATES_SCHEMA_VERSION,
                            "items": [],
                        }
                        duplicate_path = latest_run / "duplicate_candidates.yaml"
                        if not duplicate_path.exists():
                            raise ValueError("cached bundle missing duplicate_candidates.yaml")
                        loaded_duplicates = yaml.safe_load(duplicate_path.read_text(encoding="utf-8"))
                        if isinstance(loaded_duplicates, dict):
                            duplicate_candidates_data = loaded_duplicates

                        page_count = 0
                        sb_path = latest_run / "score_breakdown.json"
                        if sb_path.exists():
                            try:
                                sb = json.loads(sb_path.read_text(encoding="utf-8"))
                                page_count = len(sb.get("pages", []))
                            except Exception:
                                page_count = len(selected_pages)
                        else:
                            page_count = len(selected_pages)

                        validation = validate_stack(root, stack_path)

                        if output_dir is not None:
                            target_dir = Path(output_dir)
                            if not target_dir.is_absolute():
                                target_dir = root / target_dir
                            if target_dir != latest_run:
                                target_dir.mkdir(parents=True, exist_ok=True)
                                for item in latest_run.iterdir():
                                    if item.is_file():
                                        shutil.copy2(item, target_dir / item.name)
                                output_dir = target_dir
                        else:
                            output_dir = latest_run
                        if not explicit_output:
                            prune_bundle_runs(bundle_base, retention_count, Path(output_dir))

                        return {
                            "ok": validation["ok"] and not conflicts_data,
                            "output_dir": str(output_dir),
                            "selected_page_count": len(selected_pages),
                            "page_count": page_count,
                            "warnings": warnings_data,
                            "conflicts": conflicts_data,
                            "resolved_conflicts": resolved_conflicts_data,
                            "duplicate_candidate_count": len(duplicate_candidates_data.get("items", [])),
                        }
                    except Exception:
                        pass

    # Fallback to fresh generation
    validation = validate_stack(root, stack_path)
    pages, warnings = collect_pages(root, stack)
    warnings.extend(validation.get("warnings", []))
    if not validation["ok"]:
        warnings.extend(["validation_error:" + e for e in validation["errors"]])
    warnings.extend(find_orphaned_sources(root))
    conflict_policy = load_conflict_detection_policy(stack)
    conflicts = detect_conflicts(pages, conflict_policy)
    tier_by_binding = {b["source_binding_id"]: b["tier"] for b in validation["source_bindings"]}
    decisions = load_conflict_decisions(root)
    conflicts, resolved_conflicts = resolve_conflicts_by_priority(root, pages, conflicts, tier_by_binding, decisions)
    warnings.extend([f"conflict_auto_resolved: {r['title']}" for r in resolved_conflicts])
    duplicate_candidates = detect_duplicate_candidates(pages, validation["source_bindings"], resolved_conflicts)
    warnings.extend(duplicate_candidate_warnings(duplicate_candidates))

    if output_dir is None:
        run_id = datetime.now().strftime("run-%Y%m%d-%H%M%S")
        output_dir = root / default_out / run_id
    if not output_dir.is_absolute():
        output_dir = root / output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    selected_pages = [asdict(p) for p in pages if p.selected]
    all_pages = [asdict(p) for p in pages]
    stack_bindings = validation["source_bindings"]
    raw_derived_plan = build_raw_derived_plan(root)
    dependency_manifest = {
        "generated_at": now_iso(),
        "stack_path": str(stack_path),
        "dependency_ids": validation["dependency_ids"],
        "source_count": len(validation["dependency_ids"]),
    }

    artifacts = {
        "context_bundle.md": render_context_bundle(root, pages, warnings, conflicts, resolved_conflicts, duplicate_candidates),
        "dependency_manifest.yaml": dump_yaml(dependency_manifest),
        "source_binding_order.yaml": dump_yaml(stack_bindings),
        "selected_pages.yaml": dump_yaml(selected_pages),
        "selected_sources.yaml": dump_yaml({"sources": validation["dependency_ids"], "local_mutable_wiki_included": True}),
        "page_hashes.yaml": dump_yaml({p.path: p.sha256 for p in pages}),
        "source_lineage.yaml": dump_yaml({p.path: {"sources": p.sources, "source_hashes": p.source_hashes} for p in pages}),
        "access_decisions.yaml": dump_yaml({"raw_excerpt": "denied_by_default", "direct_filesystem_access_to_raw": "denied_for_team_project"}),
        "conflict_warnings.yaml": dump_yaml(conflicts),
        "duplicate_candidates.yaml": dump_yaml(duplicate_candidates),
        "resolved_conflicts.yaml": dump_yaml(resolved_conflicts),
        "warnings.yaml": dump_yaml(warnings),
        "raw_derived_registry.yaml": dump_yaml(raw_derived_plan["registry"]),
        "raw_derived_manifests.yaml": dump_yaml(raw_derived_plan["manifests"]),
        "raw_divider_plan.yaml": dump_yaml(raw_derived_plan["divider"]),
        "score_breakdown.json": json.dumps({"pages": all_pages}, indent=2, ensure_ascii=False),
    }
    for name, content in artifacts.items():
        (output_dir / name).write_text(content)
    if not explicit_output:
        prune_bundle_runs(bundle_base, retention_count, output_dir)
    return {
        "ok": validation["ok"] and not conflicts,
        "output_dir": str(output_dir),
        "selected_page_count": len(selected_pages),
        "page_count": len(pages),
        "warnings": warnings,
        "conflicts": conflicts,
        "resolved_conflicts": resolved_conflicts,
        "duplicate_candidate_count": len(duplicate_candidates.get("items", [])),
    }


def show_conflict(root: Path, title: str) -> Dict[str, Any]:
    stack_path = resolve_stack_path(root)
    stack = load_yaml(stack_path)
    content_cache: Dict[str, str] = {}
    pages, _ = collect_pages(root, stack, content_cache)
    conflicts = detect_conflicts(pages, load_conflict_detection_policy(stack))
    target = find_conflict_by_title(conflicts, title)
    if not target:
        return {"found": False, "title": title, "message": f"No active conflict found for title {title!r}."}

    entries = target.get("pages") or []
    texts: Dict[str, str] = {}
    for e in entries:
        cached = content_cache.get(e["path"])
        if cached is not None:
            texts[e["path"]] = cached
            continue
        full_path = root / e["path"]
        try:
            texts[e["path"]] = full_path.read_text(encoding="utf-8", errors="replace")
        except Exception:
            texts[e["path"]] = ""

    diff_text = ""
    if len(entries) >= 2:
        a_path, b_path = entries[0]["path"], entries[1]["path"]
        diff_lines = list(difflib.unified_diff(
            texts[a_path].splitlines(keepends=True),
            texts[b_path].splitlines(keepends=True),
            fromfile=a_path,
            tofile=b_path,
        ))
        diff_text = "".join(diff_lines)

    return {
        "found": True,
        "title": target.get("title"),
        "pages": entries,
        "diff": diff_text,
    }


def search_pages(root: Path, query: str, limit: int = 20) -> Dict[str, Any]:
    harness = load_harness_config(root)
    default_out = (((harness.get("llm_wiki_core") or {}).get("bundle_output_dir")) or ".agent-harness/bundles")
    bundle_base = root / default_out

    warnings: List[str] = []
    matches = []
    q = query.lower()

    # Find latest bundle
    latest_run = None
    if bundle_base.exists() and bundle_base.is_dir():
        runs = find_prunable_bundle_runs(bundle_base)
        if runs:
            latest_run = runs[-1]

    if not latest_run or not (latest_run / "selected_pages.yaml").exists():
        return {
            "query": query,
            "count": 0,
            "warnings": ["no_active_bundle_found: please generate a bundle first using get-context-bundle"],
            "matches": []
        }

    try:
        raw = yaml.safe_load((latest_run / "selected_pages.yaml").read_text(encoding="utf-8"))
        selected_pages = raw if isinstance(raw, list) else []
    except Exception as e:
        return {
            "query": query,
            "count": 0,
            "warnings": [f"failed to load selected_pages.yaml: {e}"],
            "matches": []
        }

    stack_path = resolve_stack_path(root)
    stack = load_yaml(stack_path)

    for p_dict in selected_pages:
        if len(matches) >= limit:
            break

        rel_path = p_dict.get("path")
        if not rel_path:
            continue

        try:
            p_rec = PageRecord(
                page_id=p_dict.get("page_id", ""),
                path=rel_path,
                source_binding_id=p_dict.get("source_binding_id", ""),
                dependency_id=p_dict.get("dependency_id", ""),
                effective_scope=p_dict.get("effective_scope", ""),
                authority_level=p_dict.get("authority_level", ""),
                title=p_dict.get("title", ""),
                status=p_dict.get("status", ""),
                tags=p_dict.get("tags") or [],
                sources=p_dict.get("sources") or [],
                source_hashes=p_dict.get("source_hashes") or {},
                review_needed=p_dict.get("review_needed", False),
                stale=p_dict.get("stale", False),
                confidence=p_dict.get("confidence", "medium"),
                sha256=p_dict.get("sha256", ""),
                selected=p_dict.get("selected", True),
                score=p_dict.get("score", 1.0),
                warnings=p_dict.get("warnings") or [],
            )
            text = read_page_content(root, p_rec, stack)
        except Exception as e:
            warnings.append(f"failed to read page content for {rel_path}: {e}")
            continue

        title = p_rec.title
        tags = p_rec.tags

        # Check matching criteria
        text_lower = text.lower()
        if q in text_lower or q in title.lower() or any(q in tag.lower() for tag in tags) or q in Path(rel_path).stem.lower():
            idx = text_lower.find(q)
            snippet = text[max(0, idx - 120): idx + 240] if idx >= 0 else text[:240]
            matches.append({**asdict(p_rec), "snippet": snippet})

    return {"query": query, "count": len(matches), "warnings": warnings, "matches": matches}


def get_lineage(root: Path, page_path: str) -> Dict[str, Any]:
    target_path = Path(page_path)
    if not target_path.is_absolute():
        target_path = root / target_path
    try:
        resolved_target = target_path.resolve()
    except Exception:
        resolved_target = target_path
        
    rel_path = rel_to(target_path, root)
    stack_path = resolve_stack_path(root)
    stack = load_yaml(stack_path)
    
    warnings: List[str] = []
    matched_page: Optional[PageRecord] = None
    
    for binding, source_root, source_kind in discover_sources(root, stack, warnings):
        if matched_page:
            break
            
        if source_kind == "artifact_file":
            try:
                members = list_archive_members(source_root)
            except Exception:
                continue
                
            for member in members:
                if not member.is_file or not member.name.endswith(".md"):
                    continue
                clean_name = member.name.lstrip("/")
                stem = source_root.name
                for ext in ARCHIVE_EXTENSIONS:
                    if stem.endswith(ext):
                        stem = stem[:-len(ext)]
                        break
                virtual_path = source_root.parent / stem / clean_name
                rel = rel_to(virtual_path, root)
                
                if rel == rel_path or virtual_path.absolute() == target_path.absolute():
                    if path_matches_glob(rel, DEFAULT_EXCLUDE_GLOBS):
                        continue
                    try:
                        content_bytes = read_archive_member(source_root, member.name)
                        text = content_bytes.decode("utf-8", errors="replace")
                        sha256_val = hashlib.sha256(content_bytes).hexdigest()
                        matched_page = build_page_record_from_data(root, rel, text, sha256_val, binding)
                    except Exception:
                        pass
                    break
        else:
            try:
                resolved_source = source_root.resolve()
            except Exception:
                resolved_source = source_root
                
            try:
                resolved_target.relative_to(resolved_source)
                is_under = True
            except ValueError:
                is_under = False
                
            if is_under and target_path.exists() and target_path.is_file():
                rel = rel_to(target_path, root)
                if not path_matches_glob(rel, DEFAULT_EXCLUDE_GLOBS):
                    matched_page = build_page_record(root, source_root, target_path, binding)
                    break
                    
    if matched_page:
        return {
            "page": matched_page.path,
            "sha256": matched_page.sha256,
            "sources": matched_page.sources,
            "source_hashes": matched_page.source_hashes,
            "review_needed": matched_page.review_needed,
            "stale": matched_page.stale,
            "confidence": matched_page.confidence,
        }
        
    p = target_path
    if not p.exists():
        raise FileNotFoundError(str(p))
    text = p.read_text(errors="replace")
    fm, _ = read_frontmatter(text)
    return {
        "page": rel_to(p, root),
        "sha256": sha256_file(p),
        "sources": normalize_list(fm.get("sources")),
        "source_hashes": fm.get("source_hashes") if isinstance(fm.get("source_hashes"), dict) else {},
        "review_needed": bool(fm.get("review_needed", False)),
        "stale": bool(fm.get("stale", False)),
        "confidence": str(fm.get("confidence", "medium")),
    }


def pack_artifact(
    root: Path,
    source: Path,
    output: Path,
    extra_exclude_dirs: Optional[List[str]] = None,
    extra_exclude_files: Optional[List[str]] = None,
) -> Dict[str, Any]:
    import zipfile

    if not source.is_absolute():
        source = root / source
    if not source.exists() or not source.is_dir():
        raise FileNotFoundError(f"pack source not found or not a directory: {source}")
    if not output.is_absolute():
        output = root / output
    if output.suffix != ".wikipkg":
        output = output.with_name(output.name + ".wikipkg")
    output.parent.mkdir(parents=True, exist_ok=True)

    exclude_dirs = DEFAULT_PACK_EXCLUDE_DIRS | set(extra_exclude_dirs or [])
    exclude_files = DEFAULT_PACK_EXCLUDE_FILE_NAMES | set(extra_exclude_files or [])

    arcname_root = source.name
    count = 0
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for dirpath, dirnames, filenames in os.walk(source):
            dirnames[:] = sorted(d for d in dirnames if d not in exclude_dirs)
            for f in sorted(filenames):
                if f in exclude_files or f.endswith(DEFAULT_PACK_EXCLUDE_FILE_SUFFIXES):
                    continue
                full = Path(dirpath) / f
                arcname = Path(arcname_root) / full.relative_to(source)
                zf.write(full, arcname=str(arcname))
                count += 1

    return {
        "ok": True,
        "source": str(source),
        "output": str(output),
        "file_count": count,
        "size_bytes": output.stat().st_size,
    }


def validate_promotion_package(path: Path) -> Dict[str, Any]:
    data = load_yaml(path)
    pkg = data.get("promotion_package")
    errors: List[str] = []
    if not isinstance(pkg, dict):
        errors.append("missing promotion_package mapping")
        pkg = {}
    required = ["source_owner", "claims", "evidence_digest", "lineage", "confidence", "requested_target_pages", "raw_transfer_policy", "reviewer_required"]
    for key in required:
        if key not in pkg:
            errors.append(f"missing promotion_package.{key}")
    for key in sorted(PROMOTION_PACKAGE_TARGET_KEYS):
        if key in pkg:
            errors.append(f"promotion_package.{key} belongs to submit, not promotion_package")
    if pkg.get("raw_transfer_policy") not in {"none", "excerpt", "source_vault_ref", "raw_copy", None}:
        errors.append("raw_transfer_policy must be none|excerpt|source_vault_ref|raw_copy")
    if pkg.get("confidence") not in {"high", "medium", "low", None}:
        errors.append("confidence must be high|medium|low")
    if pkg.get("reviewer_required") is not True:
        errors.append("reviewer_required must be true")
    return {"ok": not errors, "errors": errors, "path": str(path)}


def safe_relative_path(value: Any, field: str, errors: List[str]) -> Optional[Path]:
    if not isinstance(value, str) or not value.strip():
        errors.append(f"{field} must be a non-empty relative path")
        return None
    p = Path(value)
    if p.is_absolute() or ".." in p.parts:
        errors.append(f"{field} must be a safe relative path")
        return None
    return p


def normalize_promotion_file_entries(pkg: Dict[str, Any], key: str, errors: List[str], required: bool) -> List[Dict[str, Any]]:
    entries = pkg.get(key)
    if not isinstance(entries, list) or not entries:
        if required:
            errors.append(f"promotion_package.{key} must contain at least one item")
        return []
    normalized: List[Dict[str, Any]] = []
    for idx, entry in enumerate(entries):
        if not isinstance(entry, dict):
            errors.append(f"promotion_package.{key}[{idx}] must be a mapping")
            continue
        normalized.append(entry)
    return normalized


def validate_promotion_file_entries(package_root: Path, pkg: Dict[str, Any]) -> Tuple[List[str], List[Dict[str, Any]]]:
    errors: List[str] = []
    included: List[Dict[str, Any]] = []

    raw_transfer_policy = pkg.get("raw_transfer_policy")
    groups = [
        ("raw_items", "raw", {"raw"}, raw_transfer_policy == "raw_copy"),
        ("refined_pages", "refined", PROMOTION_REFINED_TARGET_PREFIXES, True),
    ]
    for key, kind, allowed_prefixes, required in groups:
        for idx, entry in enumerate(normalize_promotion_file_entries(pkg, key, errors, required)):
            if "source_path" in entry:
                errors.append(f"promotion_package.{key}[{idx}].source_path is not supported during submit; use pack_path")
            pack_rel = safe_relative_path(entry.get("pack_path"), f"promotion_package.{key}[{idx}].pack_path", errors)
            target_rel = safe_relative_path(entry.get("target_path"), f"promotion_package.{key}[{idx}].target_path", errors)
            expected_sha = entry.get("sha256")
            if not isinstance(expected_sha, str) or not expected_sha:
                errors.append(f"promotion_package.{key}[{idx}].sha256 is required")
            if target_rel and (not target_rel.parts or target_rel.parts[0] not in allowed_prefixes):
                allowed = "|".join(sorted(allowed_prefixes))
                errors.append(f"promotion_package.{key}[{idx}].target_path must start with {allowed}/")
            if not pack_rel:
                continue
            pack_path = package_root / pack_rel
            if not pack_path.exists() or not pack_path.is_file():
                errors.append(f"promotion_package.{key}[{idx}].pack_path not found: {pack_rel}")
                continue
            actual_sha = sha256_file(pack_path)
            if isinstance(expected_sha, str) and expected_sha and actual_sha != expected_sha:
                errors.append(f"promotion_package.{key}[{idx}].sha256 mismatch for {pack_rel}")
            if target_rel:
                included.append({
                    "kind": kind,
                    "pack_path": str(pack_rel),
                    "target_path": str(target_rel),
                    "sha256": actual_sha,
                })
    return errors, included


def submit_promotion_package(
    root: Path,
    package_path: Path,
    output_dir: Optional[Path] = None,
    force: bool = False,
) -> Dict[str, Any]:
    package_path = package_path.resolve()
    package_root = package_path.parent
    validation = validate_promotion_package(package_path)
    data = load_yaml(package_path)
    pkg = data.get("promotion_package") if isinstance(data.get("promotion_package"), dict) else {}
    errors = list(validation["errors"])

    entry_errors, included_files = validate_promotion_file_entries(package_root, pkg)
    errors.extend(entry_errors)

    if output_dir is None:
        queue_name = f"{datetime.now().strftime('%Y%m%d-%H%M%S')}-{package_path.stem}"
        output_dir = root / "llm-wiki-promotion-queue" / queue_name
    elif not output_dir.is_absolute():
        output_dir = root / output_dir

    if output_dir.exists() and not force:
        errors.append(f"output_dir already exists: {output_dir}")

    if errors:
        return {
            "ok": False,
            "errors": errors,
            "package": str(package_path),
            "output_dir": str(output_dir),
        }

    output_dir.mkdir(parents=True, exist_ok=True)
    files_dir = output_dir / "files"
    submitted_at = now_iso()
    queued_data = dict(data)
    queued_pkg = dict(pkg)
    queued_pkg["submitted_at"] = submitted_at
    queued_data["promotion_package"] = queued_pkg
    (output_dir / package_path.name).write_text(dump_yaml(queued_data))
    copied_files: List[Dict[str, Any]] = []
    for item in included_files:
        src = package_root / item["pack_path"]
        dest = files_dir / item["target_path"]
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dest)
        copied_files.append({**item, "staged_path": rel_to(dest, output_dir)})

    submission = {
        "schema_version": "promotion-submission/v1",
        "submitted_at": submitted_at,
        "promotion_package": package_path.name,
        "included_files": copied_files,
    }
    (output_dir / "submission.yaml").write_text(dump_yaml(submission))
    return {
        "ok": True,
        "errors": [],
        "package": str(package_path),
        "output_dir": str(output_dir),
        "included_file_count": len(copied_files),
        "included_files": copied_files,
    }


def capture_run(root: Path, title: str, body: str) -> Dict[str, Any]:
    out = root / ".agent-harness" / "pending-personal-captures"
    out.mkdir(parents=True, exist_ok=True)
    safe = re.sub(r"[^A-Za-z0-9._-]+", "-", title.strip().lower()).strip("-") or "capture"
    path = out / f"{datetime.now().strftime('%Y%m%d-%H%M%S')}-{safe}.md"
    path.write_text(f"---\nclassification: personal_capture_candidate\ncanonical_wiki_data: false\ncreated: {now_iso()}\ntitle: {json.dumps(title, ensure_ascii=False)}\n---\n\n# {title}\n\n{body.strip()}\n")
    harness = load_harness_config(root)
    pruned = prune_pending_captures(out, get_pending_capture_retention_count(harness), path)
    return {"ok": True, "path": str(path), "pruned": pruned}


def cmd_validate(args: argparse.Namespace) -> int:
    root = Path(args.root).resolve()
    result = validate_stack(root)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 2


def cmd_bundle(args: argparse.Namespace) -> int:
    root = Path(args.root).resolve()
    out = Path(args.output) if args.output else None
    result = get_bundle(root, out)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 1


def cmd_search(args: argparse.Namespace) -> int:
    root = Path(args.root).resolve()
    result = search_pages(root, args.query, args.limit)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


def resolve_conflict(root: Path, title: str, choose_path: str) -> Dict[str, Any]:
    stack_path = resolve_stack_path(root)
    stack = load_yaml(stack_path)
    pages, _ = collect_pages(root, stack)
    conflicts = detect_conflicts(pages, load_conflict_detection_policy(stack))
    target = find_conflict_by_title(conflicts, title)
    if not target:
        return {"ok": False, "title": title, "message": f"No active conflict found for title {title!r}."}

    valid_paths = {e["path"] for e in (target.get("pages") or [])}
    if choose_path not in valid_paths:
        return {
            "ok": False,
            "title": title,
            "message": f"{choose_path!r} is not one of this conflict's pages: {sorted(valid_paths)}",
        }

    save_conflict_decision(root, title, choose_path)
    return {"ok": True, "title": target.get("title"), "chosen_path": choose_path}


def cmd_show_conflict(args: argparse.Namespace) -> int:
    root = Path(args.root).resolve()
    result = show_conflict(root, args.title)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["found"] else 1


def cmd_resolve_conflict(args: argparse.Namespace) -> int:
    root = Path(args.root).resolve()
    result = resolve_conflict(root, args.title, args.choose)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 1


def cmd_lineage(args: argparse.Namespace) -> int:
    root = Path(args.root).resolve()
    result = get_lineage(root, args.page)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


def cmd_list_raw_items(args: argparse.Namespace) -> int:
    root = Path(args.root).resolve()
    result = list_raw_items(root)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


def cmd_get_raw_derived_manifest(args: argparse.Namespace) -> int:
    root = Path(args.root).resolve()
    result = get_raw_derived_manifest(root, args.raw_id)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["found"] else 1


def cmd_pack(args: argparse.Namespace) -> int:
    root = Path(args.root).resolve()
    if args.source:
        source = Path(args.source)
    else:
        harness = load_harness_config(root)
        source = Path(((harness.get("llm_wiki_core") or {}).get("local_wiki_root")) or "llm-wiki")
    result = pack_artifact(root, source, Path(args.output), args.exclude_dir, args.exclude_file)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 1


def cmd_promotion(args: argparse.Namespace) -> int:
    result = validate_promotion_package(Path(args.package).resolve())
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 2


def cmd_submit_promotion(args: argparse.Namespace) -> int:
    root = Path(args.root).resolve()
    result = submit_promotion_package(
        root=root,
        package_path=Path(args.package),
        output_dir=Path(args.output_dir) if args.output_dir else None,
        force=args.force,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 2


def cmd_capture(args: argparse.Namespace) -> int:
    root = Path(args.root).resolve()
    body = args.body if args.body is not None else sys.stdin.read()
    result = capture_run(root, args.title, body)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="llm-wiki-core", description="Reference CLI for llm-wiki-core context bundles and governance checks")
    p.add_argument("--root", default=".", help="Project root")
    sub = p.add_subparsers(dest="command", required=True)

    s = sub.add_parser("validate", help="Validate wiki_stack.yaml/wiki_stack.example.yaml")
    s.set_defaults(func=cmd_validate)

    s = sub.add_parser("get-context-bundle", help="Create a task context bundle snapshot")
    s.add_argument("--output", help="Output bundle directory; default .agent-harness/bundles/run-<timestamp>")
    s.set_defaults(func=cmd_bundle)

    s = sub.add_parser("search-pages", help="Search selected wiki pages")
    s.add_argument("query")
    s.add_argument("--limit", type=int, default=20)
    s.set_defaults(func=cmd_search)

    s = sub.add_parser("show-conflict", help="Show both sides of a same-title conflict, with a diff")
    s.add_argument("title")
    s.set_defaults(func=cmd_show_conflict)

    s = sub.add_parser("resolve-conflict", help="Record a human decision overriding priority-based conflict resolution")
    s.add_argument("title")
    s.add_argument("--choose", required=True, help="Path of the page to treat as authoritative for this title")
    s.set_defaults(func=cmd_resolve_conflict)

    s = sub.add_parser("get-lineage", help="Show page lineage metadata")
    s.add_argument("page")
    s.set_defaults(func=cmd_lineage)

    s = sub.add_parser("list-raw-items", help="List top-level raw items and their raw-derived classification")
    s.set_defaults(func=cmd_list_raw_items)

    s = sub.add_parser("get-raw-derived-manifest", help="Show the derived manifest/divider plan for one complex raw item")
    s.add_argument("raw_id")
    s.set_defaults(func=cmd_get_raw_derived_manifest)

    s = sub.add_parser("pack-artifact", help="Pack a wiki directory into a .wikipkg (zip) artifact for sharing")
    s.add_argument("--source", help="Directory to pack; default is the local mutable wiki root")
    s.add_argument("--output", required=True, help="Output artifact name/path; .wikipkg is added automatically")
    s.add_argument("--exclude-dir", action="append", default=[], help="Additional directory name to exclude (repeatable)")
    s.add_argument("--exclude-file", action="append", default=[], help="Additional file name to exclude (repeatable)")
    s.set_defaults(func=cmd_pack)

    s = sub.add_parser("validate-promotion", help="Validate a promotion package YAML")
    s.add_argument("package")
    s.set_defaults(func=cmd_promotion)

    s = sub.add_parser("submit-promotion", help="Stage a target-free promotion package and its raw/refined files")
    s.add_argument("package")
    s.add_argument("--output-dir", help="Submission output directory; default llm-wiki-promotion-queue/<timestamp>-<package-stem>")
    s.add_argument("--force", action="store_true", help="Overwrite an existing submission output directory")
    s.set_defaults(func=cmd_submit_promotion)

    s = sub.add_parser("capture-run", help="Write a post-run personal capture candidate")
    s.add_argument("--title", required=True)
    s.add_argument("--body")
    s.set_defaults(func=cmd_capture)
    return p


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
