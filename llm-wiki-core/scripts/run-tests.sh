#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
PYTHONPATH=./llm-wiki-core python3 -m unittest discover -s llm-wiki-core/tests -p 'test_*.py'
bash -n llm-wiki-core/scripts/init-llm-wiki.sh
bash -n llm-wiki-core/scripts/init-agent-os.sh
bash -n llm-wiki-core/scripts/init-artifact-wiki.sh
bash -n llm-wiki-core/scripts/llm-wiki-core
for hook in llm-wiki-core/hooks/*.sh; do
  bash -n "$hook"
done
llm-wiki-core/scripts/llm-wiki-core --root . validate >/tmp/llm-wiki-core-validate.json
python3 - <<'PY'
from pathlib import Path
import re, yaml
all_skills = list(Path('llm-wiki-core/skills').glob('**/SKILL.md'))
for p in all_skills:
    text = p.read_text()
    assert text.startswith('---'), f'{p}: frontmatter must start at byte 0'
    m = re.search(r'\n---\s*\n', text[3:])
    assert m, f'{p}: missing closing frontmatter marker'
    fm = yaml.safe_load(text[3:m.start()+3])
    assert isinstance(fm, dict), f'{p}: frontmatter must be mapping'
    assert fm.get('name'), f'{p}: missing name'
    assert fm.get('description'), f'{p}: missing description'
    assert len(fm['description']) <= 1024, f'{p}: description too long'
print('skill frontmatter ok')
PY
