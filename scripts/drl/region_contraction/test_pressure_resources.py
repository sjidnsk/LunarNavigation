import importlib.util
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

SCRIPT = Path(__file__).with_name('pressure_resources.py')


class ResourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if SCRIPT.exists():
            spec = importlib.util.spec_from_file_location('pressure_resources', SCRIPT)
            cls.mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.mod)

    def setUp(self):
        self.assertTrue(SCRIPT.exists(), 'resource collector feature is missing')
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        self.proc = self.base / 'proc'; self.proc.mkdir()
        (self.proc / 'stat').write_text('btime 1000\n')
        (self.proc / 'meminfo').write_text('MemTotal: 300000 kB\nMemAvailable: 120000 kB\nSwapFree: 100 kB\n')
        self.root = self.base / 'artifacts'; self.root.mkdir()
        self.eval_script = SCRIPT.with_name('evaluate.py')
        self.dispatch_script = SCRIPT.with_name('pressure.py')
        self.reader = self.mod.ProcReader(self.proc, clock_ticks=100)

    def process(self, pid, ppid, start, args, rss=100, pss=60, comm='python (worker)'):
        d = self.proc / str(pid); d.mkdir(exist_ok=True)
        # /proc/stat: state is field 3; starttime is field 22.
        fields = ['S', str(ppid)] + ['0'] * 17 + [str(start)] + ['0'] * 3
        (d / 'stat').write_text(f'{pid} ({comm}) ' + ' '.join(fields))
        (d / 'cmdline').write_bytes(b'\0'.join(str(x).encode() for x in args) + b'\0')
        (d / 'status').write_text(f'VmRSS: {rss} kB\n')
        if pss is not None:
            (d / 'smaps_rollup').write_text(f'Rss: {rss} kB\nPss: {pss} kB\n')

    def fixture(self):
        self.process(10, 1, 100, ['python3', self.dispatch_script, '--root', self.root])
        cmd = ['python', str(self.eval_script), '--mode', 'baseline', '--output', str(self.root/'moon050-r1'/'baseline')]
        self.process(20, 10, 200, cmd, rss=100, pss=60)
        self.process(21, 20, 210, ['native_planner'], rss=200, pss=120)
        self.process(22, 21, 220, ['controller'], rss=300, pss=180)
        self.process(99, 10, 200, ['formal-training'], rss=10000, pss=8000)
        state = dict(status='running', pid=10, active=[dict(case='moon050-r1', mode='baseline', pid=20,
            command=cmd, launched_unix=1002.)])
        return state

    def sampler(self):
        return self.mod.Sampler(self.root, reader=self.reader, expected_dispatcher_pid=10)

    def test_owned_tree_excludes_training_and_keeps_rss_pss_separate(self):
        sample = self.sampler().sample(self.fixture())
        row = sample['cases'][0]
        self.assertTrue(row['ownership_verified'])
        self.assertTrue(row['complete'])
        self.assertEqual([x['pid'] for x in row['processes']], [20,21,22])
        self.assertEqual(row['rss_total_kib'], 600)
        self.assertEqual(row['pss_total_kib'], 360)
        self.assertEqual(row['process_count_sampled'], 3)
        self.assertEqual(sample['host']['MemAvailable_kib'], 120000)

    def test_missing_pss_stays_null_and_has_explicit_partial_sum(self):
        state = self.fixture(); (self.proc/'21'/'smaps_rollup').unlink()
        row = self.sampler().sample(state)['cases'][0]
        self.assertIsNone(row['pss_total_kib'])
        self.assertEqual(row['pss_observed_kib'], 240)
        self.assertEqual(row['pss_process_count'], 2)
        self.assertEqual(row['rss_total_kib'], 600)
        self.assertFalse(row['complete'])

    def test_wrong_parent_chain_rejects_registered_root(self):
        state = self.fixture()
        self.process(20, 1, 200, state['active'][0]['command'], rss=10000)
        row = self.sampler().sample(state)['cases'][0]
        self.assertFalse(row['ownership_verified'])
        self.assertIsNone(row['rss_total_kib'])
        self.assertEqual(row['processes'], [])

    def test_reused_pid_is_rejected_even_with_matching_command(self):
        state = self.fixture(); sampler = self.sampler()
        self.assertTrue(sampler.sample(state)['cases'][0]['ownership_verified'])
        self.process(20,10,600,state['active'][0]['command'])
        self.assertFalse(sampler.sample(state)['cases'][0]['ownership_verified'])

    def test_launch_time_mismatch_rejects_first_sample(self):
        state = self.fixture();state['active'][0]['launched_unix'] = 1010.
        self.assertFalse(self.sampler().sample(state)['cases'][0]['ownership_verified'])

    def test_missing_dispatcher_is_not_zero_memory(self):
        state = self.fixture(); (self.proc/'10'/'stat').unlink()
        row = self.sampler().sample(state)['cases'][0]
        self.assertFalse(row['ownership_verified'])
        self.assertIsNone(row['rss_total_kib'])
        self.assertIsNone(row['pss_total_kib'])

    def test_normal_running_sleeping_state_change_is_not_pid_reuse(self):
        state = self.fixture(); sampler=self.sampler(); original=self.reader.memory
        def memory(pid):
            result=original(pid)
            p=self.proc/str(pid)/'stat';p.write_text(p.read_text().replace(') S ', ') R '))
            return result
        self.reader.memory=memory
        row=sampler.sample(state)['cases'][0]
        self.assertTrue(row['complete'])
        self.assertEqual(row['rss_total_kib'],600)

    def test_wrong_case_output_cannot_be_attributed(self):
        state = self.fixture()
        self.process(20,10,200,['python',self.eval_script,'--mode','baseline','--output',self.root/'another'/'baseline'])
        self.assertFalse(self.sampler().sample(state)['cases'][0]['ownership_verified'])

    def parallel_fixture(self):
        state=self.fixture(); helper=SCRIPT.with_name('pressure_parallel.py')
        self.process(10,1,100,['python3',helper,'--root',self.root,'--workers','4'])
        (self.root/'protocol.json').write_text('{"frozen":"test"}')
        override=dict(helper_path=str(helper),helper_sha=hashlib.sha256(helper.read_bytes()).hexdigest(),
                      protocol_sha=hashlib.sha256((self.root/'protocol.json').read_bytes()).hexdigest())
        (self.root/'execution-override.json').write_text(json.dumps(override))
        (self.root/'handoff.json').write_text(json.dumps(dict(phase='parallel_running',
            parallel_owner=dict(pid=10,ppid=1,start_ticks=100),
            execution_override_sha=hashlib.sha256((self.root/'execution-override.json').read_bytes()).hexdigest())))
        return state

    def test_parallel_owner_requires_registered_source_and_start_identity(self):
        state=self.parallel_fixture()
        self.assertTrue(self.sampler().sample(state)['cases'][0]['ownership_verified'])
        receipt=json.loads((self.root/'handoff.json').read_text())
        receipt['parallel_owner']['start_ticks']=99
        (self.root/'handoff.json').write_text(json.dumps(receipt))
        self.assertFalse(self.sampler().sample(state)['cases'][0]['ownership_verified'])

    def test_unregistered_parallel_source_is_rejected(self):
        state=self.parallel_fixture()
        (self.root/'execution-override.json').write_text('{"helper_sha":"untrusted"}')
        self.assertFalse(self.sampler().sample(state)['cases'][0]['ownership_verified'])

    def test_parallel_owner_normal_reparent_keeps_owned_workers_attributed(self):
        state=self.parallel_fixture()
        self.process(10,2,100,['python3',SCRIPT.with_name('pressure_parallel.py'),'--root',self.root,'--workers','4'])
        row=self.sampler().sample(state)['cases'][0]
        self.assertTrue(row['ownership_verified'])
        self.assertEqual(row['process_count_discovered'],3)

    def test_maxima_does_not_substitute_partial_pss(self):
        state = self.fixture(); sampler = self.sampler()
        a = sampler.sample(state); maxima = self.mod.update_maxima(None,a)
        (self.proc/'21'/'smaps_rollup').unlink(); self.process(22,21,220,['controller'],rss=500,pss=180)
        b = sampler.sample(state); maxima = self.mod.update_maxima(maxima,b)
        row = maxima['cases']['moon050-r1/baseline']
        self.assertEqual(row['rss_max_kib'],800)
        self.assertEqual(row['pss_max_kib'],360)
        self.assertEqual(row['samples'],2)
        self.assertEqual(row['complete_samples'],1)

    def test_writer_appends_samples_and_replaces_only_own_maxima(self):
        sample = self.sampler().sample(self.fixture())
        with self.mod.SampleWriter(self.root) as writer:
            writer.write(sample); writer.write(sample)
        rows=[json.loads(x) for x in (self.root/'resource-samples.jsonl').read_text().splitlines()]
        self.assertEqual(len(rows),2)
        self.assertEqual(json.loads((self.root/'maxima.json').read_text())['samples'],2)
        (self.root/'maxima.json').write_text('{"schema":"someone-else"}')
        with self.assertRaises(ValueError):
            with self.mod.SampleWriter(self.root):
                pass


if __name__ == '__main__':
    unittest.main()
