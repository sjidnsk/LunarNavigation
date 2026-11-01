"""Continuous follower certificates on footprint-inflated navigation maps."""
from dataclasses import replace
import math
import numpy as np
import pytest
from lunar_pure_wheeled_controller.mppi import MppiPolicy
from lunar_pure_wheeled_controller.mppi_map import GridMap
from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
from lunar_pure_wheeled_controller.tracking import TrackingPolicy, TrackingState


def follower(*, clearance=.000001, steps=1, response=.001):
    e=MppiPathFollower(replace(TrackingPolicy(),execution_clearance_m=clearance,
        control_response_time_s=response),MppiPolicy(time_steps=steps,nominal_response_time_s=0.))
    e.set_path(((0.,0.,0.),(1.,0.,0.)))
    return e


def fine_grid(*, obstacle=None, value=2, assumed=False, revision=1):
    states=np.ones((200,1200),np.uint8)
    mask=np.zeros_like(states,bool)
    if obstacle is not None:
        x,y=obstacle;states[y,x]=value;mask[y,x]=assumed
    return GridMap(states,.001,(-.2,-.1),assumed=mask,revision=revision)


def test_missing_map_stops_follower_before_tracking_or_completion():
    e=follower()
    for state in (TrackingState(0,0,0),TrackingState(1,0,0)):
        r=e.update(state,.05)
        assert e.requires_map and r.phase=='WAITING'
        assert r.command.linear_x_mps==r.command.angular_z_radps==0.


@pytest.mark.parametrize('value,assumed',[(2,False),(0,False),(0,True)])
def test_thin_cell_between_free_rollout_knots_is_rejected(value,assumed):
    e=follower();e.note_command(.2,0.)
    e.set_map(fine_grid(obstacle=(205,100),value=value,assumed=assumed))
    state=TrackingState(0,.0005,0.,.2)
    sequence=np.array([[[.2,0.]]])
    poses,_=e._rollout(state,sequence,.05)
    assert np.all(e.map.sample(poses[:,:,:2])[0]==1), 'only the connecting segment crosses the cell'
    valid,_=e._certify_sequences(state,sequence,.05)
    assert not valid[0]


def test_closed_corner_contact_and_millimeter_clearance_are_certified():
    e=follower();e.note_command(.2,0.)
    e.set_map(fine_grid(obstacle=(205,100)))
    sequence=np.array([[[.2,0.]]])
    assert not e._certify_sequences(TrackingState(0,0.,0.,.2),sequence,.05)[0][0]
    assert e._certify_sequences(TrackingState(0,-.001,0.,.2),sequence,.05)[0][0]
    e.policy=replace(e.policy,execution_clearance_m=.0011)
    assert not e._certify_sequences(TrackingState(0,-.001,0.,.2),sequence,.05)[0][0]


def test_actual_arc_is_checked_even_when_endpoint_chord_misses_cell():
    e=follower();e.note_command(.2,.2)
    # Upper edge of the row lies between the arc midpoint and chord midpoint.
    xmid=math.sin(.005);ymid=1.-math.cos(.005)
    resolution=.0001
    origin=(-.001,ymid+.000005-11*resolution)
    cell=int((xmid-origin[0])/resolution)
    states=np.ones((100,1200),np.uint8);states[10,cell]=2
    e.set_map(GridMap(states,resolution,origin))
    state=TrackingState(0.,0.,0.,.2,.2)
    sequence=np.array([[[.2,.2]]]);poses,velocity=e._rollout(state,sequence,.05)
    assert e.map.valid_segments(poses[:,:-1,:2],poses[:,1:,:2],0.).all()
    assert not e._certify_sequences(state,sequence,.05)[0][0]
    e.set_map(GridMap(np.ones_like(states),resolution,origin))
    assert e._certify_sequences(state,sequence,.05)[0][0]


@pytest.mark.parametrize('residual',[.2,-.2])
def test_full_braking_tail_covers_residual_velocity_beyond_short_horizon(residual):
    e=follower(response=.15);e.note_command(residual,0.,stamp_s=-1.)
    # The one-step nominal rollout stays near x=0; delayed braking reaches wall.
    x=240 if residual>0 else 159
    states=np.ones((200,1200),np.uint8);states[:,x]=2
    e.set_map(GridMap(states,.001,(-.2,-.1)))
    state=TrackingState(0.,.0005,0.,residual)
    sequence=np.zeros((1,1,2));poses,velocity=e._rollout(state,sequence,.05)
    assert e.map.valid_motion(poses,velocity,.05,0.)[0]
    assert not e._certify_sequences(state,sequence,.05)[0][0]
    assert not e.certify_publication(state,0.,0.,.05)


@pytest.mark.parametrize('assumed',[False,True])
def test_unknown_and_assumed_support_only_native_stationary_rotation(assumed):
    e=follower(steps=8)
    states=np.zeros((20,20),np.uint8)
    source=GridMap(states,.1,(-1.05,-1.05),assumed=np.full_like(states,assumed,dtype=bool),stationary_anchor=(0.,0.))
    e.set_map(source);e.set_path(((0.,0.,math.pi/2),))
    r=e.update(TrackingState(0,0,0),.05)
    assert r.command.linear_x_mps==0. and r.command.angular_z_radps>0.
    assert not e.certify_publication(TrackingState(0,0,0,.001),0.,0.,0.)
    e.set_path(((0.,0.,0.),(.1,0.,0.)))
    r=e.update(TrackingState(0,0,0),.05)
    assert r.command.linear_x_mps==r.command.angular_z_radps==0.
    assert r.phase!='COMPLETED'
    np.testing.assert_array_equal(source.assumed,np.full_like(states,assumed,dtype=bool))


def test_new_map_same_revision_revokes_previous_certificate_and_assumed_halo():
    e=follower(clearance=.002);e.note_command(.2,0.)
    state=TrackingState(0.,.0005,0.,.2);sequence=np.array([[[.2,0.]]])
    e.set_map(fine_grid())
    assert e._certify_sequences(state,sequence,.05)[0][0]
    # Assumed cell is only in the clearance halo, never at the sampled center.
    e.set_map(fine_grid(obstacle=(205,101),value=0,assumed=True))
    assert not e._certify_sequences(state,sequence,.05)[0][0]


def test_weighted_collision_rejects_average_but_certifies_safe_sample():
    e=follower(steps=40);e.note_command(.2,0.)
    e.set_map(fine_grid(obstacle=(355,100)))
    state=TrackingState(0.,.0005,0.,.2)
    samples=np.zeros((2,40,2));samples[:,:,0]=.2
    samples[0,:,1]=.15;samples[1,:,1]=-.15
    # Safe mirrored arcs have an unsafe mean control: certify the actual
    # nonlinear rollout rather than averaged sample poses or safe endpoints.
    average=samples.mean(axis=0,keepdims=True)
    assert not e._certify_sequences(state,average,.05)[0][0]
    valid,poses=e._certify_sequences(state,samples,.05,costs=np.array([1.,2.]))
    assert np.flatnonzero(valid).tolist()==[0]
    first=e._first_commands(state,samples[:1,0],.05)[0]
    assert e.certify_publication(state,*first,0.)
    assert poses[0,-1,1]>0. and poses[1,-1,1]<0.
