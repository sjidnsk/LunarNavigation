"""共享的纯观测策略输入准备；不读取真值、训练器或权重。"""
import time
from .contracts import SensorSpec


def prepare_policy_state(native, *, snapshot=None, architecture_version='region_relation_v2',
                         mode='fine', cap=32, task_id='', sensor=None):
    sensor = sensor or SensorSpec()
    started=time.perf_counter()
    if architecture_version == 'candidate_geometry_v3':
        from .candidate_observation import prepare_candidate_observation
        if snapshot is None:
            raise ValueError('v5 input requires the frozen build snapshot')
        result=prepare_candidate_observation(native,snapshot.pose,contract=mode=='a2',
            max_remote_positions=cap if mode=='a2' else 0,
            metadata=dict(platform_profile_hash=snapshot.profile_hash,
                map_resolution_m=float(snapshot.resolution_m),task_id=task_id,
                sensor_mount=dict(offset_x_m=sensor.offset_x_m,
                    offset_y_m=sensor.offset_y_m,offset_yaw_rad=sensor.offset_yaw_rad)))
        policy,fine=result.observation,result.fine_observation
        diagnostics=result.diagnostics
        virtual_nodes=int(result.virtual_mask.sum())
    elif mode=='a2':
        from .policy_graph import prepare_policy_graph
        result=prepare_policy_graph(native,max_remote_positions=cap)
        policy,fine=result.observation,result.fine_observation
        diagnostics=result.diagnostics
        virtual_nodes=int(result.virtual_mask.sum())
    elif mode=='fine':
        policy=fine=native;diagnostics={};virtual_nodes=0
    else:
        raise ValueError('unknown policy graph mode')
    stats=dict(mode=mode,max_remote_positions=cap,
        native_nodes=len(native.node_ids),fine_nodes=len(fine.node_ids),
        coarse_nodes=len(policy.node_ids),native_edges=len(native.edges),
        fine_edges=len(fine.edges),coarse_edges=len(policy.edges),
        native_actions=len(native.goals),actions=len(policy.goals),virtual_nodes=virtual_nodes,
        preparation_wall_s=time.perf_counter()-started)
    return policy,fine,diagnostics,stats
