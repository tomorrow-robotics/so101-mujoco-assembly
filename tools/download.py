#!/usr/bin/env python3
"""Download, extract and verify a release of the SO-101 MuJoCo assembly scene.

    python tools/download.py --version v1.1 --out data
    python tools/download.py --version v1.1 --out data --local-tar so101_mujoco_assembly_v1.1.tar.gz

The archive extracts to <out>/so101_mujoco_assembly_<version>/ and every file is
checked against the MANIFEST.json shipped inside it (SHA-256 and size).
"""
import argparse
import hashlib
import json
import sys
import tarfile
from pathlib import Path, PurePosixPath

DEFAULT_REPO = "SeanZheng/so101-mujoco-assembly"


def fetch(repo, version):
    from huggingface_hub import hf_hub_download  # only needed when downloading

    name = f"so101_mujoco_assembly_{version}.tar.gz"
    return Path(hf_hub_download(repo, f"{version}/{name}", repo_type="dataset"))


def safe_extract(tar_path, out_dir):
    """Extract regular files and directories only; refuse anything that could escape out_dir."""
    with tarfile.open(tar_path) as tar:
        members = tar.getmembers()
        for m in members:
            path = PurePosixPath(m.name)
            if path.is_absolute() or ".." in path.parts or "\\" in m.name:
                raise SystemExit(f"unsafe path in archive: {m.name!r}")
            if not (m.isfile() or m.isdir()):
                raise SystemExit(f"unsupported member (link or device) in archive: {m.name!r}")
        if hasattr(tarfile, "data_filter"):
            tar.extractall(out_dir, members=members, filter="data")
        else:  # Python without extraction filters; members were checked above
            tar.extractall(out_dir, members=members)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def verify(root):
    """Return (manifest, problems); problems is empty when every listed file matches."""
    manifest = json.loads((root / "MANIFEST.json").read_text())
    files = manifest["files"]
    problems = []
    if manifest.get("n_files", len(files)) != len(files):
        problems.append(f"MANIFEST.json: n_files={manifest['n_files']} but {len(files)} entries")
    for rel, want in sorted(files.items()):
        path = root / rel
        if not path.is_file():
            problems.append(f"missing: {rel}")
        elif path.stat().st_size != want["bytes"]:
            problems.append(f"size mismatch: {rel} ({path.stat().st_size} != {want['bytes']} bytes)")
        elif sha256(path) != want["sha256"]:
            problems.append(f"sha256 mismatch: {rel}")
    return manifest, problems


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--version", default="v1.1")
    ap.add_argument("--out", required=True, type=Path, help="directory to extract into")
    ap.add_argument("--repo", default=DEFAULT_REPO, help="Hugging Face dataset repo")
    ap.add_argument("--local-tar", type=Path, help="use this archive instead of downloading")
    args = ap.parse_args()

    tar_path = args.local_tar or fetch(args.repo, args.version)
    if not tar_path.is_file():
        raise SystemExit(f"archive not found: {tar_path}")
    args.out.mkdir(parents=True, exist_ok=True)
    print(f"extracting {tar_path} -> {args.out}")
    safe_extract(tar_path, args.out)

    root = args.out / f"so101_mujoco_assembly_{args.version}"
    if not (root / "MANIFEST.json").is_file():
        raise SystemExit(f"no MANIFEST.json in {root}")
    manifest, problems = verify(root)
    if problems:
        print(f"VERIFICATION FAILED ({len(problems)} problem(s)):", file=sys.stderr)
        for p in problems:
            print(f"  {p}", file=sys.stderr)
        return 1
    print(f"verified {len(manifest['files'])} files")
    print(f"scene: {(root / manifest['scene']).resolve()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
