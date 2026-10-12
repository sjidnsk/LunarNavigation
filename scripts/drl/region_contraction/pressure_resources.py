"""Read-only process memory sampler for the frozen pressure dispatcher's workers.

Run this helper on the host (the sandbox may expose a different /proc). It never
signals, starts or changes a worker. RSS sums count shared pages repeatedly;
PSS sums apportion those pages across all system sharers. Sampled maxima are
lower bounds on transient peaks, and unavailable PSS is never replaced by RSS.
Only resource-samples.jsonl and maxima.json are written in the artifact root.
"""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time


SAMPLE_SCHEMA = 'pressure_resource_sample_v1'
MAXIMA_SCHEMA = 'pressure_resource_maxima_v1'
NOTE = ('KiB units. Summed RSS double-counts shared resident pages; summed PSS '
        'is proportional to all system sharers, including processes outside this '
        'case. Missing totals are null; observed partial sums are not full totals. '
        'Sampling every interval does not capture every transient peak.')


def parse_stat(text):
    # comm can contain whitespace and parentheses; fields after its LAST ')' are
    # state (field 3), ppid (4), ... starttime (22), in clock ticks after boot.
    end = text.rfind(')')
    fields = text[end + 1:].split()
    if end < 0 or len(fields) < 20:
        raise ValueError('invalid /proc PID stat')
    return dict(pid=int(text.split('(', 1)[0]), state=fields[0],
                ppid=int(fields[1]), start_ticks=int(fields[19]))


class ProcReader:
    def __init__(self, root='/proc', *, clock_ticks=None):
        self.root = Path(root)
        self.clock_ticks = clock_ticks or os.sysconf('SC_CLK_TCK')

    def stat(self, pid):
        value = parse_stat((self.root / str(pid) / 'stat').read_text())
        if value['pid'] != pid or value['state'] in ('Z', 'X', 'x'):
            raise ValueError('process is not live')
        return value

    def table(self):
        result = {}
        for path in self.root.iterdir():
            if path.name.isdecimal():
                try:
                    result[int(path.name)] = self.stat(int(path.name))
                except (OSError, ValueError):
                    pass  # Races in unrelated processes do not affect attribution.
        return result

    def command(self, pid):
        return [part.decode(errors='replace') for part in
                (self.root / str(pid) / 'cmdline').read_bytes().split(b'\0') if part]

    def boot_time(self):
        for line in (self.root / 'stat').read_text().splitlines():
            if line.startswith('btime '):
                return int(line.split()[1])
        raise ValueError('process start-time origin is unavailable')

    def memory(self, pid):
        path = self.root / str(pid)
        values, error = {}, None
        try:
            for line in (path / 'smaps_rollup').read_text().splitlines():
                key, _, value = line.partition(':')
                if key in ('Rss', 'Pss'):
                    values[key] = int(value.split()[0])
        except (OSError, ValueError) as exc:
            error = f'{type(exc).__name__}: smaps_rollup unavailable'
        rss_source = 'smaps_rollup'
        if 'Rss' not in values:
            rss_source = 'status.VmRSS'
            try:
                for line in (path / 'status').read_text().splitlines():
                    if line.startswith('VmRSS:'):
                        values['Rss'] = int(line.split()[1])
            except (OSError, ValueError):
                pass
        return dict(rss_kib=values.get('Rss'), pss_kib=values.get('Pss'),
                    rss_source=rss_source if 'Rss' in values else None,
                    pss_source='smaps_rollup' if 'Pss' in values else None,
                    memory_error=error)

    def host(self):
        memory = {}
        for line in (self.root / 'meminfo').read_text().splitlines():
            key, _, value = line.partition(':')
            if key in ('MemTotal', 'MemAvailable', 'SwapFree'):
                memory[key + '_kib'] = int(value.split()[0])
        try:
            load = list(map(float, (self.root / 'loadavg').read_text().split()[:3]))
        except OSError:
            load = list(os.getloadavg())
        return dict(loadavg=load, **memory)


def option(command, name):
    matches = [i for i, value in enumerate(command) if value == name]
    if len(matches) != 1 or matches[0] + 1 >= len(command):
        raise ValueError('missing or ambiguous command argument: ' + name)
    return command[matches[0] + 1]


def descendant(table, pid, ancestor):
    seen = set()
    while pid in table and pid not in seen:
        if pid == ancestor:
            return True
        seen.add(pid)
        child = table[pid]
        parent = table.get(child['ppid'])
        if parent is None or parent['start_ticks'] > child['start_ticks']:
            return False
        pid = child['ppid']
    return False


def same_process(first, second):
    # R/S/D scheduling-state changes are ordinary activity, not PID reuse.
    return all(first[key] == second[key] for key in ('pid', 'ppid', 'start_ticks'))


class Sampler:
    def __init__(self, root, *, reader=None, expected_dispatcher_pid=None):
        self.root = Path(root).resolve()
        self.reader = reader or ProcReader()
        self.expected_dispatcher_pid = expected_dispatcher_pid
        self.dispatcher_identity = None
        self.worker_identities = {}
        self.script = Path(__file__).resolve()
        self.collector_sha = hashlib.sha256(self.script.read_bytes()).hexdigest()

    def dispatcher(self, state, table):
        pid = state.get('pid')
        if type(pid) is not int or (self.expected_dispatcher_pid is not None
                                   and pid != self.expected_dispatcher_pid):
            raise ValueError('dispatcher PID does not match the expected owner')
        info = table.get(pid)
        if info is None:
            raise ValueError('dispatcher is not live or not visible in this /proc')
        command = self.reader.command(pid)
        legacy = str(self.script.with_name('pressure.py')) in command
        parallel = str(self.script.with_name('pressure_parallel.py')) in command
        if (legacy == parallel or Path(option(command, '--root')).resolve() != self.root):
            raise ValueError('dispatcher command/root identity mismatch')
        if parallel:
            # Auxiliary scheduling changes are explicit, hash registered and
            # owned by exactly one PID/start time. No descendant checks relax.
            override_path = self.root / 'execution-override.json'
            override = json.loads(override_path.read_text())
            receipt = json.loads((self.root / 'handoff.json').read_text())
            helper = self.script.with_name('pressure_parallel.py')
            digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
            if (override.get('helper_path') != str(helper)
                    or override.get('helper_sha') != digest(helper)
                    or override.get('protocol_sha') != digest(self.root / 'protocol.json')
                    or receipt.get('execution_override_sha') != digest(override_path)
                    or receipt.get('phase') not in ('dispatch_lock_acquired', 'parallel_running', 'parallel_finished')
                    or any(receipt.get('parallel_owner', {}).get(key) != info[key]
                           for key in ('pid', 'start_ticks'))):
                raise ValueError('parallel dispatcher source/owner registration mismatch')
        identity = (pid, info['start_ticks'])
        if self.dispatcher_identity not in (None, identity):
            raise ValueError('dispatcher PID/start time changed')
        self.dispatcher_identity = identity
        return pid

    def worker(self, active, dispatcher, table):
        pid, case, mode = active['pid'], active['case'], active['mode']
        if type(pid) is not int or table.get(pid) is None:
            raise ValueError('registered evaluation worker is not live/visible')
        if (not isinstance(case, str) or Path(case).name != case
                or mode not in ('baseline', 'remote', 'contracted')):
            raise ValueError('invalid active case/mode')
        command = self.reader.command(pid)
        expected_output = self.root / case / mode
        for args in (command, active['command']):
            if (str(self.script.with_name('evaluate.py')) not in args
                    or option(args, '--mode') != mode
                    or Path(option(args, '--output')).resolve() != expected_output):
                raise ValueError('evaluation command/case/output identity mismatch')
        if pid == dispatcher or not descendant(table, pid, dispatcher):
            raise ValueError('evaluation worker is not a dispatcher descendant')
        launch = float(active['launched_unix'])
        start = self.reader.boot_time() + table[pid]['start_ticks'] / self.reader.clock_ticks
        if not math.isfinite(launch) or abs(start - launch) > 5:
            raise ValueError('worker start time does not match its recorded launch')
        key, identity = (case, mode, pid), table[pid]['start_ticks']
        if self.worker_identities.get(key, identity) != identity:
            raise ValueError('registered worker PID was reused')
        self.worker_identities[key] = identity
        return pid

    def sample(self, state):
        started = time.monotonic()
        table, error = self.reader.table(), None
        try:
            dispatcher = self.dispatcher(state, table)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            dispatcher, error = None, str(exc)
        rows, attributed = [], set()
        for active in state.get('active', []):
            row = dict(case=active.get('case'), mode=active.get('mode'), pid=active.get('pid'),
                ownership_verified=False, complete=False, processes=[], process_count_discovered=0,
                process_count_sampled=0, rss_total_kib=None, pss_total_kib=None,
                rss_observed_kib=0, pss_observed_kib=0, pss_process_count=0, errors=[])
            rows.append(row)
            try:
                if dispatcher is None:
                    raise ValueError(error)
                root_pid = self.worker(active, dispatcher, table)
                members = sorted(pid for pid in table if descendant(table, pid, root_pid))
                if attributed.intersection(members):
                    raise ValueError('active worker trees overlap')
                attributed.update(members)
                row.update(ownership_verified=True, root_start_ticks=table[root_pid]['start_ticks'],
                           process_count_discovered=len(members))
                for pid in members:
                    try:
                        before = self.reader.stat(pid)
                        if not same_process(before, table[pid]):
                            raise ValueError('PID identity/parent changed during discovery')
                        # Recheck current parent links to prevent reparented/reused
                        # processes from being sampled after their owned root exits.
                        chain, current = {}, pid
                        while current not in chain:
                            chain[current] = self.reader.stat(current)
                            if current == root_pid:
                                break
                            current = chain[current]['ppid']
                        if (not descendant(chain, pid, root_pid)
                                or not same_process(chain[root_pid], table[root_pid])):
                            raise ValueError('owned descendant relationship changed')
                        command = self.reader.command(pid)
                        memory = self.reader.memory(pid)
                        if not same_process(self.reader.stat(pid), before):
                            raise ValueError('PID identity changed during memory read')
                        row['processes'].append(dict(before, **memory,
                            command=command, role='evaluation' if pid == root_pid else 'descendant'))
                    except (OSError, ValueError, KeyError) as exc:
                        row['errors'].append(dict(pid=pid, error=str(exc)))
                processes = row['processes']
                row['process_count_sampled'] = len(processes)
                row['rss_observed_kib'] = sum(p['rss_kib'] or 0 for p in processes)
                row['pss_observed_kib'] = sum(p['pss_kib'] or 0 for p in processes)
                row['pss_process_count'] = sum(p['pss_kib'] is not None for p in processes)
                all_sampled = len(processes) == len(members)
                if all_sampled and all(p['rss_kib'] is not None for p in processes):
                    row['rss_total_kib'] = row['rss_observed_kib']
                if all_sampled and row['pss_process_count'] == len(members):
                    row['pss_total_kib'] = row['pss_observed_kib']
                row['complete'] = row['rss_total_kib'] is not None and row['pss_total_kib'] is not None
            except (OSError, ValueError, KeyError, TypeError) as exc:
                row['errors'].append(dict(error=str(exc)))
        return dict(schema=SAMPLE_SCHEMA, root=str(self.root), time_unix=time.time(),
            dispatcher_pid=state.get('pid'), dispatcher_start_ticks=None if self.dispatcher_identity is None
                else self.dispatcher_identity[1], dispatcher_status=state.get('status'),
            dispatcher_phase=state.get('phase'), dispatcher_error=error,
            collector_pid=os.getpid(), collector_sha=self.collector_sha, note=NOTE,
            host=self.reader.host(), cases=rows, sampling_seconds=time.monotonic() - started)


def update_maxima(previous, sample):
    result = previous or dict(schema=MAXIMA_SCHEMA, root=sample['root'], samples=0, cases={}, note=NOTE)
    if result.get('schema') != MAXIMA_SCHEMA or result.get('root') != sample['root']:
        raise ValueError('existing maxima does not belong to this resource collector/root')
    result['samples'] += 1
    result['updated_unix'] = sample['time_unix']
    result['collector_sha'] = sample['collector_sha']
    available = sample['host'].get('MemAvailable_kib')
    if available is not None:
        result['minimum_host_available_kib'] = min(available, result.get('minimum_host_available_kib', available))
    for row in sample['cases']:
        key = f'{row["case"]}/{row["mode"]}'
        record = result['cases'].setdefault(key, dict(samples=0, complete_samples=0, rejected_samples=0,
            rss_max_kib=None, pss_max_kib=None, process_count_max=0))
        record['samples'] += 1
        record['complete_samples'] += int(row['complete'])
        record['rejected_samples'] += int(not row['ownership_verified'])
        record['process_count_max'] = max(record['process_count_max'], row['process_count_sampled'])
        for metric in ('rss', 'pss'):
            value = row[metric + '_total_kib']
            maximum = record[metric + '_max_kib']
            if value is not None and (maximum is None or value > maximum):
                record[metric + '_max_kib'] = value
                record[metric + '_max_time_unix'] = sample['time_unix']
                record[metric + '_max_process_count'] = row['process_count_sampled']
    return result


class SampleWriter:
    def __init__(self, root):
        self.root, self.stream, self.maxima = Path(root), None, None

    def __enter__(self):
        # Lock the append file itself: a second collector cannot mix sample
        # sequences or race the maxima replacement. No additional lock artifact.
        self.stream = (self.root / 'resource-samples.jsonl').open('a', encoding='utf-8')
        try:
            fcntl.flock(self.stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            path = self.root / 'maxima.json'
            if path.exists():
                self.maxima = json.loads(path.read_text())
                if self.maxima.get('schema') != MAXIMA_SCHEMA or self.maxima.get('root') != str(self.root.resolve()):
                    raise ValueError('existing maxima is not this resource collector output')
        except BaseException:
            self.stream.close()
            raise
        return self

    def write(self, sample):
        self.maxima = update_maxima(self.maxima, sample)
        self.stream.write(json.dumps(sample, allow_nan=False) + '\n')
        self.stream.flush()
        fd, temporary = tempfile.mkstemp(prefix='.maxima.', dir=self.root)
        try:
            with os.fdopen(fd, 'w') as stream:
                json.dump(self.maxima, stream, allow_nan=False, indent=2)
                stream.write('\n')
            os.replace(temporary, self.root / 'maxima.json')
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def __exit__(self, *args):
        self.stream.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--interval', type=float, default=10.)
    parser.add_argument('--expected-dispatcher-pid', type=int)
    parser.add_argument('--samples', type=int)
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--dry-run', action='store_true', help='print bounded samples; write no artifacts')
    args = parser.parse_args(argv)
    if (not math.isfinite(args.interval) or args.interval <= 0
            or args.samples is not None and args.samples <= 0):
        parser.error('interval and sample limit must be positive')
    limit = 1 if args.once else args.samples
    if args.dry_run and limit is None:
        parser.error('--dry-run requires --once or --samples')
    root = args.root.resolve()
    sampler = Sampler(root, expected_dispatcher_pid=args.expected_dispatcher_pid)
    writer = None if args.dry_run else SampleWriter(root)
    if writer is not None:
        writer.__enter__()
    try:
        count = 0
        while True:
            started = time.monotonic()
            state = json.loads((root / 'run-state.json').read_text())
            sample = sampler.sample(state)
            if writer is None:
                print(json.dumps(sample, allow_nan=False), flush=True)
            else:
                writer.write(sample)
            count += 1
            if (limit is not None and count >= limit
                    or state.get('status') in ('finished', 'finished_with_errors', 'error')):
                break
            time.sleep(max(0., args.interval - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass  # Interrupt only this sampler; it owns no worker processes.
    finally:
        if writer is not None:
            writer.__exit__()


if __name__ == '__main__':
    main()
