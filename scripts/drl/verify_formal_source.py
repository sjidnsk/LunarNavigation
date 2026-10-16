#!/usr/bin/env python3
"""Read-only byte verification of a recorded formal training package."""
import argparse
import hashlib
import json
from pathlib import Path
import re


def verify_source(package,manifest):
    package,manifest=Path(package),Path(manifest)
    record=json.loads(manifest.read_text(encoding='utf-8'))
    if record.get('schema')!='formal_drl_source_v1' or not isinstance(record.get('files'),dict):
        raise ValueError('unsupported formal source manifest')
    expected=record['files']
    for name,digest in expected.items():
        if (not isinstance(name,str) or Path(name).name!=name or '/' in name or '\\' in name
                or not name.endswith('.py') or not isinstance(digest,str)
                or not re.fullmatch(r'[0-9a-f]{64}',digest)):
            raise ValueError('invalid manifest filename or digest')
    actual={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in package.glob('*.py') if p.is_file()}
    missing=sorted(expected.keys()-actual.keys());extra=sorted(actual.keys()-expected.keys())
    changed=sorted(name for name in expected.keys()&actual.keys() if expected[name]!=actual[name])
    return dict(ok=not(missing or extra or changed),file_count=len(actual),missing=missing,extra=extra,changed=changed)


def main():
    root=Path(__file__).resolve().parents[2]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package',type=Path,default=root/'ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration')
    parser.add_argument('--manifest',type=Path,default=root/'docs/drl/formal-runtime-source-manifest.json')
    args=parser.parse_args()
    result=verify_source(args.package,args.manifest)
    print(json.dumps(result,sort_keys=True))
    return 0 if result['ok'] else 1


if __name__=='__main__':raise SystemExit(main())
