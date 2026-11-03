#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <sys/resource.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include "lunar_incremental_navigation_ros/exploration_map.hpp"
#include "lunar_incremental_navigation_ros/platform_config.hpp"
#include "lunar_pure_exploration_core/candidate_generator.hpp"
#include "lunar_pure_exploration_core/information_gain.hpp"
namespace core=lunar::incremental_navigation;
namespace bridge=lunar::incremental_navigation_ros;
namespace ex=lunar::pure_exploration;
using Clock=std::chrono::steady_clock;
double ms(Clock::time_point t){return std::chrono::duration<double,std::milli>(Clock::now()-t).count();}
int main(int argc,char**argv){try{
 if(argc!=5 && argc!=6)throw std::runtime_error("usage: audit wheel.yaml length source_resolution output_resolution");
 double L=std::stod(argv[2]),r=std::stod(argv[3]),out=std::stod(argv[4]);
 auto loaded=bridge::LoadPlatformConfig(argv[1],"wheel");
 if(!loaded.capability)throw std::runtime_error(loaded.error_detail);
 auto cap=*loaded.capability;
 auto wheel=std::get<core::WheeledCapability>(cap);
 core::TraversabilityProfile profile{.planar_envelope_xy_m=wheel.footprint_xy_m,.preferred_clearance_m=wheel.minimum_clearance_m,.slope_weight=1.,.relief_weight=1.,.clearance_weight=1.,.goal_position_tolerance_m=.1,.goal_yaw_tolerance_rad=.0872664626};
 double radius=core::CircumscribedRadius(wheel.footprint_xy_m);
 for(auto&p:profile.planar_envelope_xy_m){p.x*=1.+wheel.minimum_clearance_m/radius;p.y*=1.+wheel.minimum_clearance_m/radius;}
 bridge::OwnedElevationEvidence source;
 size_t N=std::llround(L/r);
 source.geometry={"map",N,N,r,{-L/2,-L/2,0.}};
 source.map_from_source={.parent_frame="map",.child_frame="map"};
 source.elevation_m.resize(N*N);
 for(size_t y=0;y<N;++y)for(size_t x=0;x<N;++x){double px=-L/2+(x+.5)*r,py=-L/2+(y+.5)*r;
  float h=.025*px;
  if(std::abs(px)>L*.42 || std::abs(py)>L*.42)h=std::numeric_limits<float>::quiet_NaN();
  else if((px-2.3)*(px-2.3)+(py-1.2)*(py-1.2)<.85*.85 || (px+3.1)*(px+3.1)+(py+2.1)*(py+2.1)<1.05*1.05)h+=.5;
  source.elevation_m[y*N+x]=h;
 }
 if(argc==6 && std::string(argv[5])=="raster-limit"){
  ex::GridGeometry geometry{(uint32_t)N,(uint32_t)N,r,-L/2,-L/2,0.};
  std::vector<int8_t> data(N*N,0);
  ex::Polygon2 boundary{{{-L/2,-L/2},{L/2,-L/2},{L/2,L/2},{-L/2,L/2}}};
  try{auto task=ex::TaskRaster::Build(ex::OccupancyGridView(geometry,data,100),boundary);std::cout<<"raster_ok cells="<<task.task_cells().size()<<std::endl;}
  catch(const std::exception&e){std::cout<<"raster_rejected: "<<e.what()<<std::endl;}
  return 0;
 }
 bridge::ExplorationMap map(cap,profile,out,out);
 ex::PlatformGeometry platform{.platform_id="wheeled-lunar-explorer",.platform_type="WHEELED",.base_frame_id="base_footprint",.footprint_vertices={},.minimum_clearance_m=wheel.minimum_clearance_m};
 for(auto p:wheel.footprint_xy_m)platform.footprint_vertices.push_back({p.x,p.y});
 ex::CandidateGenerator generator(platform,{{-M_PI/4,-M_PI/8,0,M_PI/8,M_PI/4}},{100000,10000,1000000});
 ex::InformationGainEvaluator gain({10.,M_PI/2},{1000000});
 ex::Polygon2 polygon{{{-L/2,-L/2},{L/2,-L/2},{L/2,L/2},{-L/2,L/2}}};
 for(int trial=0;trial<(argc==6?1:2);++trial){
  if(trial)source.elevation_m[(N/2)*N+N/2]+=.001;
  auto begin=Clock::now();bool applied=map.Apply(source,trial+1);double apply_ms=ms(begin);
  begin=Clock::now();auto capture=map.Capture();double capture_ms=ms(begin);
  if(argc==6){
   capture.map.anchor=core::Pose2{0.,0.,0.};capture.map.local_window_size_m=5.;capture.map.epoch="resolution-audit";
   begin=Clock::now();auto response=bridge::PolicyMapExporter(cap,profile).ExportObserved(capture,0);double export_ms=ms(begin);
   rclcpp::SerializedMessage wire;
   begin=Clock::now();rclcpp::Serialization<lunar_planning_msgs::srv::GetPolicyMap::Response>{}.serialize_message(&response,&wire);double serialize_ms=ms(begin);
   struct rusage usage{};getrusage(RUSAGE_SELF,&usage);
   std::cout<<"{\"length_m\":"<<L<<",\"source_resolution_m\":"<<r<<",\"output_resolution_m\":"<<out<<",\"apply_ms\":"<<apply_ms<<",\"export_ms\":"<<export_ms<<",\"serialize_ms\":"<<serialize_ms<<",\"tiles\":"<<response.tiles.size()<<",\"wire_bytes\":"<<wire.size()<<",\"ready\":"<<response.ready<<",\"rss_peak_kib\":"<<usage.ru_maxrss<<"}"<<std::endl;
   continue;
  }
  begin=Clock::now();auto projection=map.Project();double project_ms=ms(begin);
  auto g=projection.geometry;size_t free=0,blocked=0;for(auto c:projection.data){free+=c==0;blocked+=c==100;}
  ex::GridGeometry eg{(uint32_t)g.width(),(uint32_t)g.height(),g.resolution_m(),g.origin_m().x+g.min_inclusive().x*out,g.origin_m().y+g.min_inclusive().y*out,0.};
  begin=Clock::now();auto task=ex::TaskRaster::Build(ex::OccupancyGridView(eg,projection.data,100),polygon);double raster_ms=ms(begin);
  begin=Clock::now();auto frontiers=ex::FrontierDetector({1.}).DetectAll(task);double frontier_ms=ms(begin);
  begin=Clock::now();auto candidates=generator.Generate(task,frontiers.clusters);double candidates_ms=ms(begin);
  begin=Clock::now();size_t scored=0;std::string gain_error;double area=0.;
  try{for(auto&c:candidates){area+=gain.Evaluate(task,c).visible_unknown_area_m2;++scored;}}catch(const std::exception&e){gain_error=e.what();}
  double gain_ms=ms(begin);auto stats=map.StorageStats();struct rusage usage{};getrusage(RUSAGE_SELF,&usage);
  std::cout<<"{\"length_m\":"<<L<<",\"source_resolution_m\":"<<r<<",\"output_resolution_m\":"<<out<<",\"trial\":"<<trial
   <<",\"applied\":"<<applied<<",\"source_cells\":"<<N*N<<",\"apply_ms\":"<<apply_ms<<",\"capture_ms\":"<<capture_ms<<",\"project_ms\":"<<project_ms
   <<",\"raster_ms\":"<<raster_ms<<",\"frontier_ms\":"<<frontier_ms<<",\"candidates_ms\":"<<candidates_ms<<",\"gain_ms\":"<<gain_ms
   <<",\"policy_cells\":"<<stats.policy_cells<<",\"fragments\":"<<stats.fragments<<",\"output_cells\":"<<projection.data.size()<<",\"free_cells\":"<<free<<",\"blocked_cells\":"<<blocked
   <<",\"frontiers\":"<<frontiers.clusters.size()<<",\"candidates\":"<<candidates.size()<<",\"scored\":"<<scored<<",\"gain_error\":\""<<gain_error<<"\",\"rss_peak_kib\":"<<usage.ru_maxrss<<"}"<<std::endl;
 }
}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}}
