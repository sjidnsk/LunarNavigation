import os
from pathlib import Path
import pytest
from lunar_drl_exploration.domain_lease import DomainLease, DomainBusy


def test_same_domain_exclusive_and_released(tmp_path):
    first=DomainLease.acquire(230,root=tmp_path,proc_root=tmp_path/'proc')
    with pytest.raises(DomainBusy):DomainLease.acquire(230,root=tmp_path,proc_root=tmp_path/'proc')
    first.close()
    second=DomainLease.acquire(230,root=tmp_path,proc_root=tmp_path/'proc');second.close()


def test_detects_existing_unleased_native_process_and_auto_skips(tmp_path):
    proc=tmp_path/'proc';native=proc/'12345';native.mkdir(parents=True)
    (native/'environ').write_bytes(b'ROS_DOMAIN_ID=232\0OTHER=unused\0')
    with pytest.raises(DomainBusy,match='12345'):
        DomainLease.acquire(232,root=tmp_path/'locks',proc_root=proc)
    lease=DomainLease.acquire(None,root=tmp_path/'locks',proc_root=proc)
    assert lease.domain_id==231
    lease.close()


def test_lease_survives_child_exit_and_does_not_unlink_lock(tmp_path):
    lease=DomainLease.acquire(230,root=tmp_path,proc_root=tmp_path/'proc')
    assert not os.get_inheritable(lease.fd)
    lease.close();lease.close()
    assert (tmp_path/'domain-230.lock').exists()


def test_environment_failed_startup_releases_domain(tmp_path,monkeypatch):
    import numpy as np
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.contracts import Pose,TaskSpec
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    from lunar_drl_exploration.scene import TerrainGrid
    config=TrainingConfig();env=RosExplorationEnv(config,0,domain_base=230)
    def fail_start():
        env._domain_lease=DomainLease.acquire(230,root=tmp_path,proc_root=tmp_path/'proc')
        raise RuntimeError('ROS startup failure')
    monkeypatch.setattr(env,'_start_ros',fail_start)
    terrain=TerrainGrid.from_heights(np.zeros((30,30),np.float32),.2,(0.,0.),config.platform)
    task=TaskSpec('test','map',np.array([[1,1],[5,1],[5,5],[1,5]],np.float32))
    with pytest.raises(RuntimeError,match='ROS startup failure'):
        env.reset_scene(terrain,Pose(3.,3.,0.),task)
    assert env._closed and env._domain_lease is None
    lease=DomainLease.acquire(230,root=tmp_path,proc_root=tmp_path/'proc');lease.close()
