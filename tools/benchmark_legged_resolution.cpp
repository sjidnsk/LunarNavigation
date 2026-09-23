// Offline synthetic benchmark only. Args: side_m clearance_weight repetitions scenes only_resolution.
// Production defaults: 16 m, clearance 2, 10 repetitions, 3 scenes, all resolutions.
// Each scene/resolution gets one excluded warmup. No ROS or controller output.
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <numbers>
#include <vector>
#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"
#include "lunar_incremental_navigation_core/legged_local_planner.hpp"
using namespace lunar::incremental_navigation;
using Clock=std::chrono::steady_clock;
double ms(Clock::time_point a,Clock::time_point b){return std::chrono::duration<double,std::milli>(b-a).count();}
int main(int argc, char** argv){
 double side=argc>1?std::atof(argv[1]):16.; double clearance=argc>2?std::atof(argv[2]):2.; int reps=argc>3?std::atoi(argv[3]):10; int scenes=argc>4?std::atoi(argv[4]):3; double only_r=argc>5?std::atof(argv[5]):0.;
 LeggedCapability c; c.body_extent_m={.68,.33,.35}; c.maximum_slope_rad=std::numbers::pi/6; c.maximum_step_height_m=.5; c.maximum_gap_width_m=.3; c.minimum_body_clearance_m=.3;
 c.motion_primitives={{.primitive_id="forward",.kind=LeggedPrimitiveKind::kForward,.body_frame_displacement_m={.2,0,0}}, {.primitive_id="backward",.kind=LeggedPrimitiveKind::kBackward,.body_frame_displacement_m={-.2,0,0}}, {.primitive_id="left",.kind=LeggedPrimitiveKind::kLateralLeft,.body_frame_displacement_m={0,.2,0}}, {.primitive_id="right",.kind=LeggedPrimitiveKind::kLateralRight,.body_frame_displacement_m={0,-.2,0}}, {.primitive_id="spin-left",.kind=LeggedPrimitiveKind::kSpin,.yaw_change_rad=std::numbers::pi/16}, {.primitive_id="spin-right",.kind=LeggedPrimitiveKind::kSpin,.yaw_change_rad=-std::numbers::pi/16}};
 TraversabilityProfile p; p.planar_envelope_xy_m={{-.34,-.165},{.34,-.165},{.34,.165},{-.34,.165}}; p.preferred_clearance_m=.3;p.slope_weight=1.;p.relief_weight=1.;p.clearance_weight=clearance;
 std::cout<<"scene,resolution,iteration,apply_ms,derive_ms,plan_ms,update_apply_ms,update_derive_ms,status,expanded,transitions,path_length_m\n";
 for(int rep=-1;rep<reps;++rep) for(int scene=0;scene<scenes;++scene) for(double r: (rep%2==0 ? std::vector<double>{.2,.1,.05}:std::vector<double>{.05,.1,.2})){
  if(only_r>0 && std::abs(r-only_r)>1e-9)continue;
  int n=std::lround(side/r); GridGeometry g{.frame_id="map",.width=(unsigned)n,.height=(unsigned)n,.resolution_m=r}; std::vector<float> z(n*n,0);
  for(int y=0;y<n;++y)for(int x=0;x<n;++x){double X=(x+.5)*r,Y=(y+.5)*r;
   if(scene==1 && X>=7.2 && X<8 && Y<10)z[y*n+x]=1.;
   if(scene==2 && X>=7.2 && X<8 && Y>=7 && Y<9)z[y*n+x]=.4;
  }
  PersistentElevationMap map; auto a=Clock::now();
  (void)map.Apply({.geometry=g,.elevation_m=z,.map_from_source={.parent_frame="map",.child_frame="map"}});auto b=Clock::now();
  FineTraversabilityBuilder builder;auto fine=builder.Derive(map.Snapshot(),c,p);auto d=Clock::now();
  Pose2 start{.position_m={2.1,8.1}}; LocalTarget goal{.center={13.9,8.1},.is_final_goal=true};
  RequestLocalPlanningView view(fine,start,0.,{}); LeggedLocalPlanner planner(c,{.terminal_yaw_tolerance_rad=.025});
  auto result=planner.Plan(view,start,goal,Clock::now()+std::chrono::seconds(3),{});auto e=Clock::now();
  double len=0;for(size_t i=1;i<result.path.size();++i)len+=std::hypot(result.path[i].pose.position_m.x-result.path[i-1].pose.position_m.x,result.path[i].pose.position_m.y-result.path[i-1].pose.position_m.y);
  // Equal physical 0.4 x 0.4 m off-route update, not one cell at each resolution.
  for(int y=0;y<n;++y)for(int x=0;x<n;++x)if((x+.5)*r>=3.2&&(x+.5)*r<3.6&&(y+.5)*r>=3.2&&(y+.5)*r<3.6)z[y*n+x]=.05;
  auto f=Clock::now();(void)map.Apply({.geometry=g,.elevation_m=z,.map_from_source={.parent_frame="map",.child_frame="map"}});auto h=Clock::now();
  auto updated=builder.Derive(map.Snapshot(),c,p,fine);auto j=Clock::now();
  if(rep>=0)std::cout<<scene<<','<<r<<','<<rep<<','<<ms(a,b)<<','<<ms(b,d)<<','<<ms(d,e)<<','<<ms(f,h)<<','<<ms(h,j)<<','<<int(result.status)<<','<<result.statistics.expanded_states<<','<<result.statistics.evaluated_transitions<<','<<len<<std::endl;
 }
}
