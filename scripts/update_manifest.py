"""Update hashes of project code, configuration and original scenario inputs."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    files = set((ROOT / '完整代码' / 'src').rglob('*.py'))
    files.update((ROOT / 'scripts').glob('*.py'))
    files.update((ROOT / '完整代码' / 'configs').rglob('*.json'))
    for name in ('run.py', 'launch.py', 'pyproject.toml', 'requirements.txt', '启动项目.cmd', '项目命令.cmd'):
        files.add(ROOT / '完整代码' / name)
    for scene in ('bridge', 'bridge_stl_20', 'other_scenes'):
        files.update(path for path in (ROOT / '数据集' / scene).rglob('*') if path.is_file())
    entries = []
    for path in sorted(files):
        data = path.read_bytes()
        normalization = 'binary' if path.suffix == '.stl' else 'lf'
        if normalization == 'lf':
            data = data.replace(b'\r\n', b'\n')
        entries.append({'path': path.relative_to(ROOT).as_posix(), 'normalization': normalization,
                        'sha256': hashlib.sha256(data).hexdigest()})
    output = ROOT / 'docs' / 'SOURCE_MANIFEST.json'
    output.write_text(json.dumps({'schema_version': 1, 'algorithm': 'sha256', 'files': entries},
                                 ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(f'Wrote {len(entries)} source and input hashes.')


if __name__ == '__main__':
    main()
