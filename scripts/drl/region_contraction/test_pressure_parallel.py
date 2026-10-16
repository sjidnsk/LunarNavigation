"""Boundary handoff tests use temporary artifacts and owned fake processes only."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).with_name('pressure_parallel.py')


class ParallelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = None
        if SCRIPT.exists():
            spec = importlib.util.spec_from_file_location('pressure_parallel', SCRIPT)
            cls.mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.mod)

    def setUp(self):
        self.assertIsNotNone(self.mod, 'parallel boundary helper is missing')
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.reader = self.mod.ProcReader()
        self.case = dict(name='moon200-r1', family='moon', extent=200, seed=42,
                         budget=1536, repeat=0, mode_order=['baseline', 'remote'])
        self.proof = {key: 'test-' + key for key in self.mod.pressure.IDENTITY}
        self.protocol = dict(cases=[self.case], modes=['baseline', 'remote'], frozen=self.proof)
        (self.root/'protocol.json').write_text(json.dumps(self.protocol))

    def identity(self, pid):
        row = self.reader.stat(pid)
        return {key: row[key] for key in ('pid', 'ppid', 'start_ticks')}

    def process(self, code, *args):
        child = subprocess.Popen([sys.executable, '-c', code, *map(str, args)],
                                 stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL, start_new_session=True)
        def cleanup():
            if child.poll() is None:
                os.kill(child.pid, signal.SIGCONT)
                child.terminate()
                child.wait(timeout=3)
        self.addCleanup(cleanup)
        return child

    def wait_for(self, predicate, timeout=4):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.02)
        self.fail('owned test process did not reach expected state')

    def terminal(self, pid):
        output = self.root / self.case['name'] / 'baseline'
        output.mkdir(parents=True, exist_ok=True)
        (output / 'dispatch.json').write_text(json.dumps(dict(case=self.case, mode='baseline', frozen=self.proof)))
        row = dict(status='finished', ended=1, error=None, pid=pid, family='moon', extent_m=200,
                   seed=42, budget=1536, mode='baseline', provenance=self.proof)
        (output / 'summary.json').write_text(json.dumps(row))
        return output

    def test_pool_runs_two_cases_concurrently_but_each_pair_stays_ordered(self):
        # Removing the bounded case pool would reduce observed active work to one.
        active = 0; peak = 0; lock = threading.Lock(); rows = []
        class Dispatch:
            stop = threading.Event()
            def one_case(this, case):
                nonlocal active, peak
                with lock:
                    active += 1; peak = max(active, peak)
                for mode in case['mode_order']:
                    rows.append((case['name'], mode)); time.sleep(.05)
                with lock:
                    active -= 1
        cases = [dict(name=str(i), mode_order=['baseline', 'remote']) for i in range(4)]
        self.mod.run_case_pool(Dispatch(), cases, 2)
        self.assertEqual(peak, 2)
        for case in cases:
            self.assertLess(rows.index((case['name'], 'baseline')), rows.index((case['name'], 'remote')))

    def test_dispatcher_run_uses_parallel_pool_and_preserves_aggregate(self):
        active=0;peak=0;lock=threading.Lock()
        class FakeCases(self.mod.ParallelDispatcher):
            def one_case(this,case):
                nonlocal active,peak
                with lock: active+=1;peak=max(peak,active)
                time.sleep(.05)
                with lock: active-=1
        protocol=dict(self.protocol,large_serial_from_m=150,
            cases=[dict(self.case,name=str(i)) for i in range(4)])
        dispatch=FakeCases(self.root,protocol,4,6*1024*1024,'test-override')
        def aggregate(*args,**kwargs):
            self.mod.pressure.write_json(self.root/'comparison.json',dict(complete_pairing=True,paired_cases=['0','1','2','3']))
        with patch.object(self.mod.subprocess,'run',side_effect=aggregate):
            dispatch.run()
        self.assertEqual(peak,4)
        state=json.loads((self.root/'run-state.json').read_text())
        self.assertEqual(state['status'],'finished')
        self.assertEqual(state['paired_cases'],4)

    def test_first_new_state_preserves_existing_terminal_count(self):
        self.terminal(12345)
        dispatch=self.mod.ParallelDispatcher(self.root,self.protocol,4,6*1024*1024,'override')
        dispatch.state()
        state=json.loads((self.root/'run-state.json').read_text())
        self.assertEqual(state['terminal_episodes'],1)
        self.assertEqual(state['terminal'][0]['case'],'moon200-r1')

    def test_admission_allows_four_200m_cases_and_limits_300m_reservations(self):
        # Without reservation accounting all four 300m jobs would pass one host sample.
        for extent, want_peak in ((200, 4), (300, 3)):
            stop = threading.Event()
            admission = self.mod.MemoryAdmission(6*1024*1024, stop,
                host=lambda: dict(MemAvailable_kb=16*1024*1024), interval=.01)
            active=0; peak=0; lock=threading.Lock()
            def work():
                nonlocal active, peak
                with admission.reserve(dict(extent=extent)):
                    with lock:
                        active+=1; peak=max(peak,active)
                    time.sleep(.05)
                    with lock:
                        active-=1
            threads=[threading.Thread(target=work) for _ in range(4)]
            for thread in threads: thread.start()
            for thread in threads: thread.join(timeout=2)
            self.assertTrue(all(not thread.is_alive() for thread in threads))
            self.assertEqual(peak, want_peak)
            self.assertEqual(admission.reserved_kb, 0)

    def test_loaded_memory_is_not_subtracted_from_available_twice(self):
        host_memory=[16*1024*1024];stop=threading.Event()
        admission=self.mod.MemoryAdmission(6*1024*1024,stop,
            host=lambda:dict(MemAvailable_kb=host_memory[0]),interval=.01)
        first=admission.reserve(dict(extent=300));first.__enter__();host_memory[0]=13*1024*1024
        second=admission.reserve(dict(extent=300));second.__enter__();host_memory[0]=10*1024*1024
        entered=threading.Event()
        def third():
            try:
                with admission.reserve(dict(extent=300)): entered.set()
            except InterruptedError: pass
        thread=threading.Thread(target=third);thread.start()
        entered.wait(1);stop.set()
        with admission.condition: admission.condition.notify_all()
        thread.join(timeout=1);second.__exit__(None,None,None);first.__exit__(None,None,None)
        self.assertTrue(entered.is_set())

    def test_summary_pid_mismatch_rejects_handoff(self):
        child = self.process('import time; time.sleep(30)')
        self.terminal(child.pid + 1)
        active = dict(case=self.case['name'], mode='baseline', pid=child.pid)
        with self.assertRaisesRegex(ValueError, 'PID'):
            self.mod.terminal_closed(self.root, self.protocol, active, self.identity(child.pid), self.reader)
        self.assertIsNone(child.poll())

    def test_summary_with_live_native_descendant_does_not_retire_parent(self):
        marker = self.root / 'native.pid'
        child = self.process('import subprocess,sys,time; p=subprocess.Popen([sys.executable,"-c","import time;time.sleep(30)"]); '
                             'open(sys.argv[1],"w").write(str(p.pid)); time.sleep(30)', marker)
        self.wait_for(marker.exists)
        native_pid = int(marker.read_text())
        self.addCleanup(lambda: os.kill(native_pid, signal.SIGTERM) if Path(f'/proc/{native_pid}').exists() else None)
        self.terminal(child.pid)
        active = dict(case=self.case['name'], mode='baseline', pid=child.pid)
        self.assertFalse(self.mod.terminal_closed(self.root, self.protocol, active, self.identity(child.pid), self.reader))
        self.assertIsNone(child.poll())

    def test_reparented_native_is_still_live_cleanup_failure(self):
        marker=self.root/'orphan.pid'
        child=self.process('import subprocess,sys,time; p=subprocess.Popen([sys.executable,"-c","import time;time.sleep(30)"]); '
                           'open(sys.argv[1],"w").write(str(p.pid)); time.sleep(30)',marker)
        self.wait_for(marker.exists);native_pid=int(marker.read_text())
        known=[self.identity(native_pid)];worker=self.identity(child.pid)
        self.addCleanup(lambda: os.kill(native_pid,signal.SIGTERM) if Path(f'/proc/{native_pid}').exists() else None)
        child.terminate();child.wait(timeout=2);self.terminal(child.pid)
        self.assertFalse(self.mod.terminal_closed(self.root,self.protocol,
            dict(case=self.case['name'],mode='baseline',pid=child.pid),worker,self.reader,known))

    def test_dead_guardian_aborts_handoff_and_restores_old_owner(self):
        old=self.process('import time;time.sleep(30)',SCRIPT.with_name('pressure.py'),'--root',self.root)
        self.mod.pressure.write_json(self.root/'run-state.json',dict(status='running',pid=old.pid,active=[]))
        dead_guard=self.process('pass');dead_guard.wait(timeout=2)
        transition=self.mod.Transition(self.root,self.protocol,old.pid,timeout=2,interval=.02)
        with patch.object(transition,'start_guardian',return_value=dead_guard):
            with self.assertRaisesRegex(RuntimeError,'guardian'):
                transition.handoff()
        self.assertIsNone(old.poll())
        self.assertNotEqual(self.reader.stat(old.pid)['state'],'T')

    def test_signal_refuses_reused_identity(self):
        child = self.process('import time; time.sleep(30)')
        identity = self.identity(child.pid); identity['start_ticks'] += 1
        with self.assertRaisesRegex(ValueError, 'identity'):
            self.mod.signal_exact(self.reader, identity, signal.SIGSTOP)
        self.assertNotEqual(self.reader.stat(child.pid)['state'], 'T')

    def test_guard_resumes_stopped_old_owner_when_helper_dies(self):
        old = self.process('import time; time.sleep(30)')
        owner = self.process('import time; time.sleep(30)')
        old_identity, owner_identity = self.identity(old.pid), self.identity(owner.pid)
        self.mod.pressure.write_json(self.root/'handoff.json', dict(phase='terminal_wait', old_owner=old_identity,
            helper_owner=owner_identity))
        os.kill(old.pid, signal.SIGSTOP)
        thread = threading.Thread(target=self.mod.guardian, args=(self.root, owner_identity, old_identity),
                                  kwargs=dict(interval=.02, timeout=3))
        thread.start(); owner.kill(); owner.wait()
        thread.join(timeout=3)
        self.assertFalse(thread.is_alive())
        self.assertNotEqual(self.reader.stat(old.pid)['state'], 'T')
        self.assertTrue(json.loads((self.root/'guardian-recovery.json').read_text())['restored'])

    def test_independent_guard_cli_survives_killed_helper(self):
        old=self.process('import time; time.sleep(30)')
        owner=self.process('import time; time.sleep(30)')
        old_identity, owner_identity=self.identity(old.pid),self.identity(owner.pid)
        self.mod.pressure.write_json(self.root/'handoff.json',dict(phase='terminal_wait',
            old_owner=old_identity,helper_owner=owner_identity))
        guard=subprocess.Popen([sys.executable,str(SCRIPT),'--root',str(self.root),'--guard',
            '--guard-helper',json.dumps(owner_identity),'--guard-old',json.dumps(old_identity),
            '--handoff-timeout','3'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,
            start_new_session=True)
        self.addCleanup(lambda: guard.terminate() if guard.poll() is None else None)
        self.wait_for((self.root/'parallel-guardian.json').exists)
        os.kill(old.pid,signal.SIGSTOP)
        owner.kill();owner.wait()
        self.assertEqual(guard.wait(timeout=4),0)
        self.assertNotEqual(self.reader.stat(old.pid)['state'],'T')

    def test_guard_owner_reparent_is_not_mistaken_for_helper_exit(self):
        old=self.process('import time;time.sleep(30)');owner=self.process('import time;time.sleep(30)')
        old_identity,owner_identity=self.identity(old.pid),self.identity(owner.pid)
        owner_identity['ppid']+=100  # Registration before launcher exits; live starttime stays equal.
        self.mod.pressure.write_json(self.root/'handoff.json',dict(phase='terminal_wait',
            old_owner=old_identity,helper_owner=owner_identity))
        os.kill(old.pid,signal.SIGSTOP)
        thread=threading.Thread(target=self.mod.guardian,args=(self.root,owner_identity,old_identity),
                                kwargs=dict(interval=.02,timeout=3))
        thread.start();time.sleep(.1)
        self.assertEqual(self.reader.stat(old.pid)['state'],'T')
        owner.kill();owner.wait();thread.join(timeout=3)
        self.assertFalse(thread.is_alive())
        self.assertNotEqual(self.reader.stat(old.pid)['state'],'T')

    def test_monitor_identity_rejects_wrong_root_without_signaling(self):
        monitor=self.process('import time;time.sleep(30)',SCRIPT.with_name('pressure_resources.py'),
                             '--root',self.root/'wrong')
        with self.assertRaisesRegex(ValueError,'command/root'):
            self.mod.inspect_service(self.root,monitor.pid,'resources',self.reader)
        self.assertIsNone(monitor.poll())

    def test_stopped_dispatcher_child_finishes_before_owner_is_interrupted(self):
        # A regression that stops/kills the native child would prevent its terminal.
        output = self.root / self.case['name'] / 'baseline'; output.mkdir(parents=True)
        (output/'dispatch.json').write_text(json.dumps(dict(case=self.case, mode='baseline', frozen=self.proof)))
        child_code = ('import json,os,sys,time; time.sleep(.4); '
            'row=json.loads(sys.argv[1]); row["pid"]=os.getpid(); '
            'open(sys.argv[2],"w").write(json.dumps(row))')
        row = dict(status='finished', ended=1, error=None, family='moon', extent_m=200,
                   seed=42, budget=1536, mode='baseline', provenance=self.proof)
        parent_code = ('import fcntl,json,os,signal,subprocess,sys,time; '
            'lock=open(sys.argv[1]+"/.dispatch.lock","a"); fcntl.flock(lock,fcntl.LOCK_EX); '
            'p=subprocess.Popen([sys.executable,"-c",sys.argv[2],sys.argv[3],sys.argv[4],'
            'sys.argv[5],"--mode","baseline","--output",sys.argv[6]]); '
            's={"status":"running","pid":os.getpid(),"active":[{"case":"moon200-r1",'
            '"mode":"baseline","pid":p.pid,"command":[sys.argv[5],"--mode","baseline",'
            '"--output",sys.argv[6]],"launched_unix":time.time()}]}; '
            'open(sys.argv[1]+"/run-state.json","w").write(json.dumps(s)); '
            'signal.signal(signal.SIGINT,lambda *a:sys.exit(0)); time.sleep(30)')
        old = self.process(parent_code, self.root, child_code, json.dumps(row), output/'summary.json',
                           SCRIPT.with_name('evaluate.py'), output, SCRIPT.with_name('pressure.py'), '--root', self.root)
        self.wait_for((self.root/'run-state.json').exists)
        transition = self.mod.Transition(self.root, self.protocol, old.pid, timeout=3, interval=.02)
        transition.handoff()
        old.wait(timeout=3)
        self.assertEqual(old.returncode, 0)
        self.assertEqual(json.loads((output/'summary.json').read_text())['status'], 'finished')
        self.assertFalse((self.root/'comparison.json').exists())
        state = json.loads((self.root/'run-state.json').read_text())
        self.assertEqual(state['pid'], old.pid)  # No new main state before acquiring dispatch lock.

    def test_partial_unregistered_output_is_rejected_without_overwrite(self):
        partial = self.root/'moon200-r1'/'remote'; partial.mkdir(parents=True)
        (partial/'preserve.txt').write_text('keep')
        with self.assertRaisesRegex(ValueError, 'partial'):
            self.mod.audit_outputs(self.root, self.protocol, [])
        self.assertEqual((partial/'preserve.txt').read_text(), 'keep')

    def test_orphan_mode_log_is_rejected_before_output_creation(self):
        parent=self.root/self.case['name'];parent.mkdir()
        (parent/'remote.log').write_text('retain original')
        with self.assertRaisesRegex(ValueError,'log'):
            self.mod.audit_outputs(self.root,self.protocol,[])
        self.assertFalse((parent/'remote').exists())


if __name__ == '__main__':
    unittest.main()
