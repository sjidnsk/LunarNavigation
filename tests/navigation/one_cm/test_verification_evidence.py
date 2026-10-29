"""最终证据门禁负例：不让终点或路线事件替代原始证据。"""
import copy
import importlib
import json
from pathlib import Path
import sys
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3] / 'scripts/navigation/one_cm'
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'ros2/lunar_nav1cm_verification'))
import benchmark_async_navigation as benchmark


def evidence():
    assert importlib.util.find_spec('nav1cm.verification_evidence') is not None, 'shared evidence contract missing'
    return importlib.import_module('nav1cm.verification_evidence')


def global_rows():
    return [dict(kind='TICK', current_goal=[1.4, 0.], current_epoch=2, route_version=None,
                 events=[dict(kind='GLOBAL_SUBMITTED', version=3)]),
            dict(kind='TICK', current_goal=[1.4, 0.], current_epoch=2, route_version=3,
                 global_result=dict(planning_outcome=0, reason_code='PLAN_FOUND', has_reference=True,
                    requested_goal=[1.4, 0.], requested_epoch=2, requested_version=3,
                    route=dict(version=3, map_epoch=2, goal=[1.4, 0.], points=[[0., 0.], [1.4, 0.]])),
                 events=[dict(kind='ROUTE_ACCEPTED', version=3, map_epoch=2, points=[[0., 0.], [1.4, 0.]])])]


def samples():
    vectors = [[x, 0., 0., 0., 0., x] for x in (0., .01, .02, .03)]
    segments = [dict(start_s=i*.05, end_s=(i+1)*.05, before=list(vectors[i]), after=list(vectors[i+1]), source='test') for i in range(3)]
    return [dict(kind='READY', sample_s=0., vector=vectors[0], segments=[]),
            dict(kind='FEEDBACK', sample_s=.15, vector=vectors[-1], segments=segments)]


def audits(rows):
    return [dict(start_s=s['start_s'], end_s=s['end_s'], source=s['source'], valid=True) for s in rows[-1]['segments']]


def test_missing_segments_cannot_turn_terminal_audit_into_continuity():
    rows=[dict(kind='FEEDBACK', sample_s=0., vector=[0,0,0,0,0,0], segments=[]),
          dict(kind='FEEDBACK', sample_s=.25, vector=[1.4,0,0,0,0,0], segments=[])]
    result=benchmark.session_outcome(rows,[dict(kind='TERMINAL_STATE', start_s=.25,end_s=.25,valid=True)],
        (1.4,0),[dict(feedback_count=2,displacement_m=1.4)],False)
    assert result['navigation_arrived']
    assert not result['all_executed_actions_audited']
    assert not result['continuity_pass']
    assert result['status']=='FAILED_EXECUTION_COVERAGE'


def test_result_serializer_keeps_route_numpy_failure_and_none_without_inference():
    e=evidence()
    from nav1cm.contracts import Route
    result=global_rows()[1]['global_result']
    result['route']=Route(3,2,np.array([[0.,0.],[1.4,0.]]),(1.4,0.))
    result['elapsed_ms']=np.float64(12.5)
    result['diagnostic']=np.array([1,2])
    encoded=json.loads(json.dumps(e.serialize_global_result(result)))
    assert encoded['route']['points']==[[0.,0.],[1.4,0.]]
    assert encoded['elapsed_ms']==12.5 and encoded['diagnostic']==[1,2]
    assert e.serialize_global_result(None) is None
    assert e.serialize_global_result({'reason_code':'NO_PATH','route':None})=={'reason_code':'NO_PATH','route':None}


@pytest.mark.parametrize('mutation', ['planning_outcome','reason_code','has_reference','bad_triple','goal','epoch','version','route_goal','event_version','stale','request_missing','request_goal'])
def test_formal_proof_requires_triple_and_matching_request_acceptance(mutation):
    e=evidence(); rows=global_rows()
    assert e.formal_global_evidence(rows)['pass']
    result=rows[1]['global_result']
    if mutation in ('planning_outcome','reason_code','has_reference'): result.pop(mutation)
    elif mutation=='bad_triple': result['has_reference']=False
    elif mutation=='goal': result['requested_goal']=[2.,0.]
    elif mutation=='epoch': result['requested_epoch']=1
    elif mutation=='version': result['requested_version']=9
    elif mutation=='route_goal': result['route']['goal']=[2.,0.]
    elif mutation=='event_version': rows[1]['events'][0]['version']=9
    elif mutation=='stale': rows[1]['events']=[dict(kind='STALE_GLOBAL_RESULT',version=3)]
    elif mutation=='request_missing': rows.pop(0)
    else: rows[0]['current_goal']=[2.,0.]
    assert not e.formal_global_evidence(rows)['pass']


def test_periodic_result_records_computed_version_without_fabricating_acceptance():
    e=evidence(); rows=global_rows()
    rows[1]['route_version']=1
    rows[1]['events']=[dict(kind='ROUTE_CONTINUITY_RETAINED',version=1,computed_version=3)]
    result=e.formal_global_evidence(rows)
    assert not result['pass'] and result['accepted_successes']==0
    assert result['results'][0]['disposition']=='RETAINED'
    rows[1]['events'][0]['computed_version']=8
    assert e.formal_global_evidence(rows)['results'][0]['disposition']=='IDENTITY_MISMATCH'
    assert e.formal_global_evidence([dict(kind='TICK',events=[dict(kind='ROUTE_ACCEPTED',version=1)])])['status']=='NOT_RECORDED'


@pytest.mark.parametrize('mutation',['empty','middle','duplicate','vector','time','last','audit_missing','terminal_only'])
def test_shared_coverage_rejects_incomplete_execution_or_action_audit(mutation):
    e=evidence(); rows=samples(); audit=audits(rows)
    assert e.execution_coverage(rows,audit)['complete']
    if mutation=='empty': rows[-1]['segments']=[]
    elif mutation=='middle': rows[-1]['segments'].pop(1)
    elif mutation=='duplicate': rows[-1]['segments'].insert(0,copy.deepcopy(rows[-1]['segments'][0]))
    elif mutation=='vector': rows[-1]['segments'][1]['before'][0]=9
    elif mutation=='time': rows[-1]['segments'][1]['start_s']=.06
    elif mutation=='last': rows[-1]['segments'].pop()
    elif mutation=='audit_missing': audit.pop(1)
    else: audit=[dict(kind='TERMINAL_STATE',start_s=.15,end_s=.15,valid=True)]
    assert not e.execution_coverage(rows,audit)['complete']
    offline=importlib.import_module('lunar_nav1cm_verification.offline')
    assert offline.execution_coverage(rows,audit)==e.execution_coverage(rows,audit)
    outcome=benchmark.session_outcome(rows,audit,(.03,0),[dict(feedback_count=2,displacement_m=.03)],False)
    assert outcome['navigation_arrived'] and not outcome['all_executed_actions_audited'] and not outcome['continuity_pass']


def test_explicit_config_anchor_and_readonly_reanalysis_keep_identity_and_input_hashes():
    e=evidence(); rows=samples(); audit=audits(rows)
    anchor=dict(sample_s=0.,vector=rows[0]['vector'],source='configured scenario.start; not emitted feedback')
    actual=e.execution_coverage(rows,audit,initial_anchor=anchor)
    assert actual['complete'] and actual['initial_anchor_source']=='READY'
    raw='\n'.join(json.dumps(r) for r in rows[1:])
    assert not e.execution_coverage(rows[1:],audit)['complete']
    prior=json.dumps({'cases':{'case':{'status':'ARRIVED','worker_busy_evidence':[dict(feedback_count=2,displacement_m=.03)]}}})
    result=benchmark.reanalyze_session('{"source_sha256":"old"}',raw,json.dumps(audit),prior,'case',goal=(.03,0),analysis_sha256='new',initial_anchor=anchor)
    assert result['identity']=={'source_sha256':'old'} and result['original_status']=='ARRIVED'
    case=result['cases']['case']
    assert case['all_executed_actions_audited'] and case['continuity_pass']
    assert case['formal_global_evidence']['status']=='NOT_RECORDED' and not case['pass']
    assert result['analysis_initial_anchor']==anchor and result['original_summary']==json.loads(prior)
    assert len(result['input_sha256']['raw_jsonl'])==64 and result['analysis_sha256']=='new'


def test_probe_formal_witness_checks_complete_message_and_missing_delivery():
    e=evidence()
    c=importlib.import_module('lunar_nav1cm_verification.contract')
    offline=importlib.import_module('lunar_nav1cm_verification.offline')
    streams={role:[dict(direction='identity',pid=pid)] for role,pid in [('fixture',11),('navigator',22),('probe',33)]}
    for index,tick in enumerate(global_rows()):
        payload={k:v for k,v in tick.items() if k!='kind'}
        msg=c.envelope('s','navigator','TICK',payload,10.+index,7,pid=22)
        streams['navigator'].append(dict(direction='send',channel='diagnostic',message=msg,receiver_pid=22,received_monotonic_s=10.+index))
        streams['probe'].append(dict(direction='receive',channel='diagnostic',message=copy.deepcopy(msg),receiver_pid=33,received_monotonic_s=10.01+index))
    assert offline.transport_evidence(streams)['formal_global_probe_delivery']
    streams['probe'][-1]['message']['payload']['global_result']['elapsed_ms']=99
    assert not offline.transport_evidence(streams)['formal_global_probe_delivery']
    streams['probe'].pop()
    assert not offline.transport_evidence(streams)['formal_global_probe_delivery']

def test_continuity_helper_requires_execution_coverage_even_with_motion_and_valid_audit():
    result=benchmark.continuity_assessment([dict(feedback_count=2,displacement_m=1.)],
        [dict(valid=True)],[],False)
    assert not result['continuity_pass']


def test_tick_evidence_preserves_failed_and_stale_results_with_current_context():
    from types import SimpleNamespace
    e=evidence()
    raw={'planning_outcome':1,'reason_code':'NO_PATH','has_reference':False,'route':None,
         'requested_epoch':1,'requested_goal':[0.,0.],'requested_version':8}
    tick=e.tick_global_evidence(SimpleNamespace(global_result=raw),(1.4,0.),2,SimpleNamespace(version=3))
    assert tick['global_result']==raw
    assert tick['current_goal']==[1.4,0.] and tick['current_epoch']==2 and tick['route_version']==3
    assert e.tick_global_evidence(SimpleNamespace(global_result=None),(1.4,0.),2,None)['global_result'] is None

def test_real_global_worker_result_has_same_native_and_json_formal_proof():
    from nav1cm.global_worker import make_request, run_request, restore_result
    from nav1cm.fixtures import scenarios
    from nav1cm.contracts import WheelLimits
    from types import SimpleNamespace
    e=evidence()
    scenario=next(s for s in scenarios() if s.name=='straight')
    request=make_request(scenario.world,(scenario.start.x,scenario.start.y),scenario.goal,1,WheelLimits(),budget_s=2.)
    restored=restore_result(run_request(request))
    assert restored['planning_outcome']==0 and restored['reason_code']=='PLAN_FOUND' and restored['has_reference'] is True
    route=restored['route']
    result=SimpleNamespace(global_result=restored)
    rows=[dict(kind='TICK',events=[dict(kind='GLOBAL_SUBMITTED',version=1)],
               **e.tick_global_evidence(SimpleNamespace(global_result=None),scenario.goal,scenario.world.epoch,None)),
          dict(kind='TICK',events=[dict(kind='ROUTE_ACCEPTED',version=route.version,map_epoch=route.map_epoch,points=route.points.tolist())],
               **e.tick_global_evidence(result,scenario.goal,scenario.world.epoch,route))]
    assert type(rows[1]['global_result']['planning_outcome']) is int
    native=e.formal_global_evidence(rows)
    assert native['pass'] and native['accepted_successes']==1
    assert e.formal_global_evidence(json.loads(json.dumps(rows)))==native
