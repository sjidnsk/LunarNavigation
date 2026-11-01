"""离线原型的时间因果、角度处理与非线性传播验证。"""
import numpy as np
import pytest
from scripts.analysis.compare_delayed_pose_estimation import (
    DelayedEKF, Observation, transition, evaluate,
)


@pytest.mark.parametrize('w',[0.,1e-7,-1e-7,.1,-.2])
def test_exact_transition_jacobian_matches_finite_difference(w):
    state=np.array([1.,2.,.4,.2,w]);dt=2.3
    _,jacobian=transition(state,dt)
    for k in range(5):
        step=np.zeros(5);step[k]=1e-8
        numerical=(transition(state+step,dt)[0]-transition(state-step,dt)[0])/2e-8
        np.testing.assert_allclose(jacobian[:,k],numerical,atol=2e-7)


def test_delayed_prediction_does_not_write_future_back_into_source_posterior():
    f=DelayedEKF();f.correct(Observation(0.,2.,np.zeros(5)))
    f.correct(Observation(1.,3.,np.array([.1,0.,.1,0.,0.])))
    before=f.state.copy();cov=f.covariance.copy()
    a=f.predict(4.);f.predict(8.);b=f.predict(4.)
    np.testing.assert_array_equal(a,b)
    np.testing.assert_array_equal(f.state,before)
    np.testing.assert_array_equal(f.covariance,cov)
    assert f.stamp==1.
    assert not f.correct(Observation(.5,5.,np.ones(5)))
    assert np.linalg.eigvalsh(cov).min()>0


def test_yaw_update_crossing_pi_uses_shortest_innovation():
    f=DelayedEKF();f.correct(Observation(0.,2.,np.array([0.,0.,np.pi-.01,0.,0.])))
    f.correct(Observation(1.,3.,np.array([0.,0.,-np.pi+.01,0.,0.])))
    assert abs(f.state[4])<.05


def test_future_label_pose_never_enters_earlier_prediction():
    obs=[Observation(float(t),float(t+2),np.array([.1*t,0.,.01*t,0.,0.])) for t in range(12)]
    estimates=[dict(t=float(t),stamp=max(0.,t-2),online=np.zeros(5)) for t in range(12)]
    params=dict(linear_noise=.03,angular_noise=.03)
    before=evaluate(estimates,obs,[],params)
    altered=list(obs);altered[8]=Observation(8.,10.,np.array([99.,50.,1.,0.,0.]))
    after=evaluate(estimates,altered,[],params)
    a=next(r for r in before if r['t']==8.)
    b=next(r for r in after if r['t']==8.)
    assert a['label_received']>a['known_until']
    assert a['latest_visible_received']<=a['known_until']
    for method in a['errors']:
        np.testing.assert_array_equal(a['errors'][method]['state'],b['errors'][method]['state'])
    assert a['errors']['delayed_ekf']['position_m'] != b['errors']['delayed_ekf']['position_m']