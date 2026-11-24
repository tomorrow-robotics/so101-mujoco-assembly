"""Skill library: cross-episode procedural memory for the Q0 harness (not part of upstream Q0-Preview).

A skill is a directory `assets/skills/<name>/` with a `SKILL.md`:

    ---
    name: <same as the directory>
    description: <one line: when to use it>
    applies_to: <tasks or 'all'>
    status: verified | unverified
    evidence: <episodes / measurements it comes from>
    ---
    <body: when to use, procedure, parameters in part body frames, checks, signs of failure>

and optional extra files (e.g. `helpers.py` with small numpy functions the policy may import in its sandbox).
A body line `{{include: <file under assets/experience>}}` is replaced by that file at staging time, so measured
tables and step guides are written once and stay in sync.

`stage(task, sandbox)` writes /work/skills/: `INDEX.md` (one line per skill that applies to the task; the only part
the policy is asked to read up front) and one read-only directory per skill. Episodic records stay in
/work/experience.md (q0_harness/experience.py); skills are what was distilled from them.

    python -m q0_harness.skills list [task]        python -m q0_harness.skills show <name>
"""
import argparse
import re
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent/'assets/skills'
INCLUDE_ROOT = Path(__file__).resolve().parent/'assets/experience'
FIELDS = ('name', 'description', 'applies_to', 'status', 'evidence')


def parse(path):
    text = Path(path).read_text()
    m = re.match(r'---\n(.*?)\n---\n(.*)', text, re.S)
    if not m:
        raise ValueError(f'{path}: missing front matter')
    meta = {}
    for line in m.group(1).splitlines():
        k, _, v = line.partition(':')
        meta[k.strip()] = v.strip()
    missing = [k for k in FIELDS if not meta.get(k)]
    if missing:
        raise ValueError(f'{path}: front matter lacks {missing}')
    if meta['name'] != Path(path).parent.name:
        raise ValueError(f"{path}: name {meta['name']!r} differs from its directory")
    if meta['status'] not in ('verified', 'unverified'):
        raise ValueError(f'{path}: status must be verified or unverified')
    return meta, m.group(2)


def applies(meta, task):
    scope = [s.strip() for s in meta['applies_to'].split(',')]
    return task is None or 'all' in scope or task in scope


def library(task=None):
    out = []
    for d in sorted(p for p in ROOT.iterdir() if (p/'SKILL.md').exists()) if ROOT.exists() else []:
        meta, body = parse(d/'SKILL.md')
        if applies(meta, task):
            out.append((d, meta, body))
    return out


def expand(body):
    def sub(m):
        f = INCLUDE_ROOT/m.group(1).strip()
        return f.read_text().strip() if f.exists() else f'(not available yet: {m.group(1).strip()})'
    return re.sub(r'\{\{include:\s*([^}]+)\}\}', sub, body)


def index(task):
    skills = library(task)
    lines = ['# Skills', '',
             'Reusable procedures distilled from earlier episodes on this robot. Each line is one skill: read the line, and '
             'open `/work/skills/<name>/SKILL.md` for the ones that apply to what you are about to do. '
             '"verified" means an episode that used it ended in success or it rests on station measurements; '
             '"unverified" is a proposal that has not been confirmed yet. Skills describe what worked; the current images and '
             'the CAD still decide the numbers.', '']
    for _, meta, _ in skills:
        lines.append(f"- `{meta['name']}` ({meta['status']}): {meta['description']}")
    return '\n'.join(lines)+'\n'


def stage(task, sandbox):
    """Write /work/skills (read-only) into the agent workspace. -> info dict."""
    dst = Path(sandbox)/'skills'
    dst.mkdir(parents=True, exist_ok=True)
    names = []
    for d, meta, body in library(task):
        out = dst/meta['name']; out.mkdir(exist_ok=True)
        head = '\n'.join(f'{k}: {meta[k]}' for k in FIELDS)
        (out/'SKILL.md').write_text(f'---\n{head}\n---\n{expand(body)}')
        for f in d.iterdir():
            if f.name != 'SKILL.md' and f.is_file():
                shutil.copy2(f, out/f.name)
        for f in out.iterdir():
            f.chmod(0o444)
        names.append(meta['name'])
    (dst/'INDEX.md').write_text(index(task)); (dst/'INDEX.md').chmod(0o444)
    return {'path': '/work/skills/INDEX.md', 'skills': names}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    l = sub.add_parser('list'); l.add_argument('task', nargs='?')
    s = sub.add_parser('show'); s.add_argument('name')
    a = ap.parse_args()
    if a.cmd == 'list':
        print(index(a.task))
    else:
        meta, body = parse(ROOT/a.name/'SKILL.md')
        print(meta); print(expand(body))


if __name__ == '__main__':
    main()
