"""Read-only episode returns from admitted transition logs, outside the runtime.

Sum the recorded total and approach rewards separately for each scene episode.
Missing actions or invalid rewards never become a claimed complete episode return.
"""
from collections import OrderedDict
import json
import math
import os
from pathlib import Path
import re


TRANSITION = re.compile(rb'"event"\s*:\s*"transition"')


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class EpisodeRewards:
    """Incremental, bounded monitor cache; no checkpoint loads or training writes."""
    def __init__(self, path):
        self.path = Path(path)
        self.records = OrderedDict()
        self.latest = {}
        self.offset = 0
        self.anchor = b''
        self.file_id = None
        self.last_transition = -1
        self.semantics = None

    def _read(self, boundary):
        semantics = boundary
        if self.semantics != semantics:
            self.__init__(self.path)
            self.semantics = semantics
        try:
            with self.path.open('rb') as stream:
                before = os.fstat(stream.fileno())
                file_id = (before.st_dev, before.st_ino)
                offset = self.offset
                if file_id != self.file_id or offset > before.st_size:
                    offset = 0
                elif self.anchor:
                    stream.seek(offset-len(self.anchor))
                    if stream.read(len(self.anchor)) != self.anchor:
                        offset = 0  # In-place compaction can keep the same inode/size.
                # Only initial attachment/compaction reads the retained log again.
                start = max(offset, before.st_size-32*1024**2)
                stream.seek(start)
                if start > offset:
                    stream.readline()
                    start = stream.tell()
                data = stream.read(32*1024**2)
                end = data.rfind(b'\n')+1
                after = os.fstat(stream.fileno())
                if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                    return  # A concurrent in-place rewrite is not a valid snapshot.
                if end:
                    # Keep the whole last record: different steps may have identical
                    # reward/geometry suffixes, so a fixed-size suffix is insufficient.
                    last_line = data.rfind(b'\n', 0, end-1)+1
                    self.anchor = data[last_line:end][-1024**2:]
                elif start != self.offset:
                    self.anchor = b''
                self.offset = start+end
                self.file_id = file_id
            for line in data[:end].splitlines():
                if not TRANSITION.search(line):
                    continue
                try:
                    row = json.loads(line)
                except (ValueError, UnicodeError):
                    continue
                if isinstance(row, dict) and row.get('event') == 'transition':
                    self._observe(row, boundary)
        except OSError:
            pass

    def _observe(self, row, boundary):
        serial, step, env, episode = (row.get(k) for k in ('transitions','steps','env','episode_id'))
        if (type(serial) is not int or serial <= max(boundary, self.last_transition)
                or type(step) is not int or step < 1 or type(env) is not int or env < 0
                or not isinstance(episode, str) or not episode or len(episode) > 128):
            return
        self.last_transition = serial
        key = (env, episode)
        previous = self.records.get(self.latest.get(env))
        if previous and previous['episode_id'] != episode and not previous['ended']:
            previous.update(ended=True, outcome='interrupted')
        record = self.records.get(key)
        if record is None:
            record = dict(env=env, episode_id=episode, reward_sum=0., approach_reward=0., recorded_steps=0, steps=0,
                complete=True, ended=False, outcome='active')
            self.records[key] = record
        if step != record['steps']+1:
            record['complete'] = False
        total, auxiliary = row.get('reward'), row.get('approach_reward')
        if not finite(total) or not finite(auxiliary):
            record['complete'] = False
        else:
            record['reward_sum'] += total
            record['approach_reward'] += auxiliary
        record['recorded_steps'] += 1
        record.update({k: row.get(k) for k in ('steps', 'seed', 'curriculum_task',
            'reference_coverage', 'budget', 'sensor', 'scene_id', 'truncation_reason')})
        record['last_transition'] = serial
        if row.get('terminated') or row.get('truncated'):
            record.update(ended=True, outcome='success' if row.get('completed') else
                'truncated' if row.get('truncated') else 'terminated')
        self.latest[env] = key
        self.records.move_to_end(key)
        # Active episodes survive even when another environment finishes many cases.
        while len(self.records) > 512:
            oldest = next((k for k,v in self.records.items() if v['ended']), None)
            if oldest is None:
                break
            self.records.pop(oldest)

    def snapshot(self, progress, *, boundary=0):
        self._read(boundary)
        current, zero_ended = [], []
        for env, p in sorted(progress.get('environments', {}).items(), key=lambda item:int(item[0])):
            env = int(env)
            episode = p.get('episode_id')
            record = self.records.get((env, episode))
            latest = self.records.get(self.latest.get(env))
            # Transition logs can lead progress.json; never attach a newer return to
            # the previous scene still present in that older progress snapshot.
            if latest and latest['last_transition'] > progress.get('transitions', -1):
                record = latest
            elif latest and episode and latest['episode_id'] != episode and not latest['ended']:
                latest.update(ended=True, outcome='interrupted')
            if record is None:
                steps = p.get('steps', 0)
                zero = bool(episode and steps == 0 and p.get('state') in ('READY','EXECUTING'))
                record = dict(env=env, episode_id=episode, curriculum_task=p.get('curriculum_task'),
                    steps=steps, recorded_steps=0, complete=zero, reward_sum=0. if zero else None,
                    approach_reward=0. if zero else None,
                    reference_coverage=p.get('reference_coverage', p.get('initial_reference_coverage')),
                    ended=bool(zero and p.get('completed')), outcome='success' if p.get('completed') else 'active')
                if record['ended']:
                    zero_ended.append(record)
            if not record['ended']:
                record = dict(record)
                if record['episode_id'] == episode and p.get('steps', 0) > record['steps']:
                    record['complete'] = False
                current.append(record)
        recent = [dict(r) for r in reversed(self.records.values()) if r['ended']]
        return dict(current=current, recent=(zero_ended+recent)[:12])
