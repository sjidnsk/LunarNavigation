"""隔离域重放真实CDR，验证地图节点默认20cm；无控制/导航/探索Action。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import deserialize_message
    from rcl_interfaces.srv import GetParameters
    from grid_map_msgs.msg import GridMap
    from nav_msgs.msg import Odometry, OccupancyGrid
    from tf2_msgs.msg import TFMessage
    parser=argparse.ArgumentParser();parser.add_argument('--binary',required=True);parser.add_argument('--frame',required=True);parser.add_argument('--wheel',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();out=Path(args.output);out.mkdir(exist_ok=False,parents=True);frame=Path(args.frame)
    if os.environ.get('ROS_DOMAIN_ID')!='88':raise RuntimeError('replay verification requires isolated domain 88')
    ns='/p4_resolution_20cm';rclpy.init();node=rclpy.create_node('resolution_verifier');latest=[]
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pubs=[node.create_publisher(t,ns+'/'+name,qos) for t,name in [(GridMap,'input_map'),(Odometry,'input_odom'),(TFMessage,'input_tf')]]
    values=[deserialize_message((frame/(name+'.cdr')).read_bytes(),t) for name,t in [('map',GridMap),('odom',Odometry),('tf',TFMessage)]]
    node.create_subscription(OccupancyGrid,ns+'/exploration_map',latest.append,qos)
    params=dict(platform_config=args.wheel,map_topic=ns+'/input_map',odometry_topic=ns+'/input_odom',tf_topic=ns+'/unused_tf',tf_static_topic=ns+'/input_tf',exploration_map_topic=ns+'/exploration_map',policy_map_service=ns+'/get_policy_map',map_qos_durability='transient_local')
    command=[args.binary,'--ros-args','--log-level','warn','-r','__ns:='+ns,'-r','__node:=map']
    for k,v in params.items():command+=['-p',k+':='+v]
    log=(out/'node.log').open('w');process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    client=node.create_client(GetParameters,ns+'/map/get_parameters');result=dict(scope='isolated replay, no controller; source timestamps preserved',domain=88,source_capture=json.loads((frame/'capture.json').read_text()))
    try:
        deadline=time.monotonic()+25.;last=0.
        while time.monotonic()<deadline and not latest:
            rclpy.spin_once(node,timeout_sec=.05)
            if process.poll() is not None:raise RuntimeError('map node exited')
            if time.monotonic()-last>1.:
                for pub,value in zip(pubs,values):pub.publish(value)
                last=time.monotonic()
        if not latest:raise RuntimeError('no projected map within bounded replay')
        future=client.call_async(GetParameters.Request(names=['coarse_resolution_m','policy_resolution_m']))
        while time.monotonic()<deadline and not future.done():rclpy.spin_once(node,timeout_sec=.05)
        if not future.done():raise RuntimeError('default parameter query timed out')
        result['parameters']={k:v.double_value for k,v in zip(['coarse_resolution_m','policy_resolution_m'],future.result().values)}
        m=latest[-1];result.update(output_resolution_m=m.info.resolution,width=m.info.width,height=m.info.height,free_cells=list(m.data).count(0),occupied_cells=list(m.data).count(100))
        result['ok']=abs(m.info.resolution-.2)<1e-7 and result['parameters']==dict(coarse_resolution_m=.2,policy_resolution_m=.2) and result['free_cells']>0
    except BaseException as exc:result['ok']=False;result['error']=repr(exc)
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=12.)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5.);result['forced_shutdown']=True
        result['node_returncode']=process.returncode;result['ok']=bool(result['ok'] and process.returncode==0 and not result.get('forced_shutdown'))
        result['vehicle_command_publishers']=[x.node_name for x in node.get_publishers_info_by_topic('/Car/T5/Car_Cmd_Vel')]
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));node.destroy_node();rclpy.shutdown();log.close()
    return 0 if result['ok'] else 1
if __name__=='__main__':raise SystemExit(main())
