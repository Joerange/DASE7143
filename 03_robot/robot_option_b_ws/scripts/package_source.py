#!/usr/bin/env python3
"""Build a portable source bundle without macOS resource forks or caches."""
from pathlib import Path
import tarfile
root = Path(__file__).resolve().parents[1]
with tarfile.open('/private/tmp/robot_main_task_src.tgz', 'w:gz') as tar:
    for path in sorted(root.rglob('*')):
        rel = path.relative_to(root)
        if any(p.startswith('.') or p == '__pycache__' for p in rel.parts):
            continue
        if rel.parts[0] not in ('src', 'tests', 'scripts', 'console_service', 'CONTRACT.md', 'README.md', 'CONSOLE.txt'):
            continue
        if path.is_file():
            tar.add(path, arcname=str(rel), recursive=False)
