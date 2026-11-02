"""真实P4组件的合成高程输入负载测试；不启动ROS节点或发布控制。"""
import argparse
from array import array
from dataclasses import asdict
import json
from pathlib import Path
import resource
import statistics
import time
import numpy as np
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray
from rclpy.serialization import serialize_message,deserialize_message
from nav1cm.p3_map import P3MapStore
from nav1cm.terrain_native import NativeTerrain,load_wheel_capability
from nav1cm.contracts import WheelLimits,VehicleState,LocalReference
from nav1cm.maps import extract_window,build_distance_field
from nav1cm.field_native import NativeDistanceField
from nav1cm.geometry_native import NativeGeometry
from nav1cm.local_mpcc import MpccPlanner
from nav1cm.certification import certify
from nav1cm.global_worker import make_request,run_request


def timed(fn):
 t=time.perf_counter();v=fn();return v,(time.perf_counter()-t)*1000


def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--skip-local',action='store_true')
 p.add_argument('--three-layers',action='store_true')
 for k in ('length','resolution'):p.add_argument('--'+k,type=float,required=True)
 for k in ('output','library-dir','wheel','build-dir'):p.add_argument('--'+k,required=True)
 a=p.parse_args();N=round(a.length/a.resolution)
 xy=-a.length/2+(np.arange(N)+.5)*a.resolution;x,y=np.meshgrid(xy,xy)
 h=(.025*x).astype(np.float32)
 h[((x-2.3)**2+(y-1.2)**2<.85**2)|((x+3.1)**2+(y+2.1)**2<1.05**2)]+=.5
 h[(abs(x)>a.length*.42)|(abs(y)>a.length*.42)]=np.nan
 msg=GridMap();msg.header.frame_id='map';msg.info.resolution=a.resolution;msg.info.length_x=a.length;msg.info.length_y=a.length;msg.info.pose.orientation.w=1.
 msg.layers=['elevation'];layer=Float32MultiArray();layer.data=array('f',h[::-1,::-1].ravel());msg.data=[layer]
 if a.three_layers:
  for name,values in [('elevation_variance',np.zeros_like(h)),('observation_count',np.isfinite(h).astype(np.float32))]:
   extra=Float32MultiArray();extra.data=array('f',values.ravel());msg.layers.append(name);msg.data.append(extra)
 wire,serialize_cold_ms=timed(lambda:serialize_message(msg))
 serialize_ms=statistics.median(timed(lambda:serialize_message(msg))[1] for _ in range(3))
 decoded,deserialize_ms=timed(lambda:deserialize_message(wire,GridMap))
 lib=Path(a.library_dir);terrain=NativeTerrain(lib/'libnav1cm_terrain.so',capability=load_wheel_capability(a.wheel))
 native=[];original=terrain.apply
 def observed(*args,**kwargs):
  value,elapsed=timed(lambda:original(*args,**kwargs));native.append(elapsed);return value
 terrain.apply=observed
 store=P3MapStore(terrain=terrain)
 limits=WheelLimits(motion_model='differential_drive',minimum_clearance_m=.0312775222660697)
 rows=[]
 for trial in range(2):
  if trial:
   h[N//2,N//2]+=.001;decoded.data[0].data=array('f',h[::-1,::-1].ravel())
  world,apply_ms=timed(lambda:store.apply(decoded))
  state=VehicleState(-a.length*.3,0.,0.,.1,0.)
  grid,window_ms=timed(lambda:extract_window(world,state,True))
  backend=NativeDistanceField(lib/'libnav1cm_distance_field.so')
  field,field_ms=timed(lambda:build_distance_field(grid,backend=backend))
  global_result=run_request(make_request(world,(state.x,0.),(a.length*.3,0.),1,limits,.75,1.,reference_smoothing=True,terminal_yaw=0.))
  rows.append(dict(trial=trial,apply_ms=apply_ms,native_terrain_ms=native[-1],python_merge_ms=apply_ms-native[-1],blocked_rectangles=len(world.blocked),free_rectangles=len(world.known_free),window_ms=window_ms,field_ms=field_ms,window_shape=list(grid.cells.shape),global_compute_ms=global_result['worker_compute_ms'],global_reason=global_result['reason_code'],global_timing=global_result['timing'],smoothing_ms=global_result['smoothing_ms']))
 if a.skip_local:
  output=dict(length_m=a.length,resolution_m=a.resolution,source_cells=N*N,layers=list(msg.layers),cdr_bytes=len(wire),serialize_cold_ms=serialize_cold_ms,serialize_ms=serialize_ms,deserialize_ms=deserialize_ms,rows=rows,rss_peak_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
  Path(a.output).write_text(json.dumps(output,indent=2));terrain.close();print(json.dumps(output));return
 ref=LocalReference(1,grid.epoch,[[state.x,0.],[state.x+1.5,0.]],[0.,1.5])
 planner,prepare_ms=timed(lambda:MpccPlanner(limits,a.build_dir))
 local=[];geometry=NativeGeometry(lib/'libnav1cm_geometry.so')
 for _ in range(3):
  result,elapsed=timed(lambda:planner.plan(state,grid,ref,distance_field=field,field_grid_digest=grid.digest()))
  row=dict(plan_ms=elapsed,reason=result.reason,timings_ms=result.timings_ms,has_candidate=result.candidate is not None)
  if result.candidate is not None:
   cert,cert_ms=timed(lambda:certify(result.candidate,grid,limits,1,backend=geometry,check_braking_tail=False))
   row.update(cert_ms=cert_ms,cert_valid=cert.valid,cert_reason=cert.reason.value)
  local.append(row)
 terrain.close()
 output=dict(length_m=a.length,resolution_m=a.resolution,source_cells=N*N,finite_cells=int(np.isfinite(h).sum()),cdr_bytes=len(wire),serialize_cold_ms=serialize_cold_ms,serialize_ms=serialize_ms,deserialize_ms=deserialize_ms,rows=rows,mpcc_prepare_ms=prepare_ms,local=local,rss_peak_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,scope='synthetic physical height scene, real production components, no P3 generation/transport/control')
 Path(a.output).write_text(json.dumps(output,indent=2));print(json.dumps(output))
if __name__=='__main__':main()
