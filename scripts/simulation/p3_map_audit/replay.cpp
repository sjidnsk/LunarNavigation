// 同一原始CDR经生产适配器、分类器和探索投影；无ROS节点或发布者。
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <filesystem>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include "lunar_incremental_navigation_ros/exploration_map.hpp"
#include "lunar_incremental_navigation_ros/platform_config.hpp"
#include "lunar_pure_exploration_core/candidate_generator.hpp"
#include <chrono>
namespace explore=lunar::pure_exploration;
namespace core=lunar::incremental_navigation;
namespace bridge=lunar::incremental_navigation_ros;
template<class T> std::shared_ptr<T> Read(const std::filesystem::path& path){
  std::ifstream in(path,std::ios::binary);if(!in)throw std::runtime_error("cannot read "+path.string());
  std::vector<char> bytes((std::istreambuf_iterator<char>(in)),{});
  rclcpp::SerializedMessage wire(bytes.size());auto& buffer=wire.get_rcl_serialized_message();
  std::memcpy(buffer.buffer,bytes.data(),bytes.size());buffer.buffer_length=bytes.size();
  auto value=std::make_shared<T>();rclcpp::Serialization<T>{}.deserialize_message(&wire,value.get());return value;
}
class SourceView final:public core::ElevationRangeView{
 const bridge::OwnedElevationEvidence& source_;core::SparseGridGeometry geometry_;
 public:explicit SourceView(const bridge::OwnedElevationEvidence& s):source_(s),geometry_(s.geometry.frame_id,s.geometry.resolution_m,s.geometry.origin_m,{0,0},{(long)s.geometry.width,(long)s.geometry.height}){}
 const core::SparseGridGeometry& geometry()const noexcept override{return geometry_;}
 std::optional<core::ElevationRange> ElevationRangeAt(core::GridIndex i)const noexcept override{
  if(!geometry_.Contains(i))return {};auto h=source_.elevation_m[i.y*source_.geometry.width+i.x];
  return std::isfinite(h)?std::optional<core::ElevationRange>({h,h}):std::nullopt;
 }
 std::optional<core::LocalTerrainMeasurements> TerrainMeasurementsAt(core::GridIndex i)const noexcept override{
  if(!geometry_.Contains(i)||source_.terrain_measurements.empty())return {};
  auto value=source_.terrain_measurements[i.y*source_.geometry.width+i.x];
  return value.center_known?std::optional<core::LocalTerrainMeasurements>(value):std::nullopt;
 }
};
core::Vec3 Transform(core::Vec3 p,const core::RigidTransform&t){
 auto q=t.rotation;core::Vec3 c{2*(q.y*p.z-q.z*p.y),2*(q.z*p.x-q.x*p.z),2*(q.x*p.y-q.y*p.x)};
 return {t.translation_m.x+p.x+q.w*c.x+q.y*c.z-q.z*c.y,t.translation_m.y+p.y+q.w*c.y+q.z*c.x-q.x*c.z,t.translation_m.z+p.z+q.w*c.z+q.x*c.y-q.y*c.x};
}
int main(int argc,char**argv){try{
 if(argc!=3)throw std::runtime_error("usage: replay capture_dir wheel.yaml");std::filesystem::path out=argv[1];
 bridge::InputSnapshot input;input.local_map=Read<grid_map_msgs::msg::GridMap>(out/"map.cdr");
 input.odometry=Read<nav_msgs::msg::Odometry>(out/"odom.cdr");
 auto tf=Read<tf2_msgs::msg::TFMessage>(out/"tf.cdr");input.map_from_odom=tf->transforms.at(0);
 auto adapted=bridge::AdaptExplorationElevation(input);if(!adapted.value)throw std::runtime_error(adapted.reason_code);
 const auto& source=*adapted.value;SourceView view(source);
 auto loaded=bridge::LoadPlatformConfig(argv[2],"wheel");if(!loaded.capability)throw std::runtime_error(loaded.error_detail);
 auto cap=*loaded.capability;const auto& wheel=std::get<core::WheeledCapability>(cap);
 core::TraversabilityProfile profile{.planar_envelope_xy_m=wheel.footprint_xy_m,.preferred_clearance_m=wheel.minimum_clearance_m,.slope_weight=1.,.relief_weight=1.,.clearance_weight=1.,.goal_position_tolerance_m=.1,.goal_yaw_tolerance_rad=.05};
 double radius=core::CircumscribedRadius(wheel.footprint_xy_m);
 for(auto&p:profile.planar_envelope_xy_m){p.x*=1.+wheel.minimum_clearance_m/radius;p.y*=1.+wheel.minimum_clearance_m/radius;}
 bridge::ExplorationMap map(cap,profile,.2,1.);
 auto stamp=input.local_map->header.stamp;auto stamp_ns=int64_t(stamp.sec)*1000000000+stamp.nanosec;
 if(!map.Apply(source,stamp_ns))throw std::runtime_error("map apply rejected");
 auto capture=map.Capture();if(!capture.map.fine)throw std::runtime_error("no finite map");
 auto fine=capture.map.fine;auto intrinsic=core::MakePlatformElevationEvaluator(cap);
 std::ofstream rows(out/"source.csv");rows<<std::setprecision(17)<<"x,y,resolution,finite,intrinsic,complete,slope,relief,rise\n";
 const auto& g=source.geometry;double minx=1e100,miny=1e100,maxx=-1e100,maxy=-1e100;
 for(double y:{g.origin_m.y,g.origin_m.y+g.height*g.resolution_m})for(double x:{g.origin_m.x,g.origin_m.x+g.width*g.resolution_m}){
 auto p=Transform({x,y,g.origin_m.z},source.map_from_source);minx=std::min(minx,p.x);maxx=std::max(maxx,p.x);miny=std::min(miny,p.y);maxy=std::max(maxy,p.y);}
 for(long y=0;y<(long)g.height;++y)for(long x=0;x<(long)g.width;++x){
 core::GridIndex i{x,y};auto p=Transform({g.origin_m.x+(x+.5)*g.resolution_m,g.origin_m.y+(y+.5)*g.resolution_m,g.origin_m.z},source.map_from_source);
 auto m=core::MeasureLocalTerrain(view,i);auto e=intrinsic->Evaluate(view,i);
 rows<<p.x<<','<<p.y<<','<<g.resolution_m<<','<<bool(view.ElevationRangeAt(i))<<','<<int(e.state)<<','<<m.neighborhood_complete<<','<<m.slope_rad<<','<<m.relief_m<<','<<m.positive_rise_m<<'\n';}
 rows.close();const auto& fg=fine->geometry();double r=fg.resolution_m();auto origin=fg.origin_m();
 core::GridIndex lo{(long)std::floor((minx-origin.x)/r+1e-8),(long)std::floor((miny-origin.y)/r+1e-8)},hi{(long)std::ceil((maxx-origin.x)/r-1e-8),(long)std::ceil((maxy-origin.y)/r-1e-8)};
 std::ofstream cells(out/"fine.csv");cells<<std::setprecision(17)<<"x,y,resolution,finite,intrinsic,state\n";
 for(long y=lo.y;y<hi.y;++y)for(long x=lo.x;x<hi.x;++x){core::GridIndex i{x,y};auto ev=capture.intrinsic->Evaluate(*fine->elevation(),i);
 cells<<origin.x+(x+.5)*r<<','<<origin.y+(y+.5)*r<<','<<r<<','<<bool(fine->elevation()->ElevationRangeAt(i))<<','<<int(ev.state)<<','<<int(fine->State(i))<<'\n';}
 std::ofstream exploration(out/"exploration.csv");exploration<<"resolution,free_cells,frontier_clusters,candidate_views,robot_cell_free,reachable_free_cells,elapsed_ms\n";
 explore::PlatformGeometry platform{.platform_id="wheeled-lunar-explorer",.platform_type="WHEELED",.base_frame_id="base_footprint",.footprint_vertices={},.minimum_clearance_m=wheel.minimum_clearance_m};
 for(auto p:wheel.footprint_xy_m)platform.footprint_vertices.push_back({p.x,p.y});
 explore::CandidateGenerator generator(platform,{{-M_PI/4,-M_PI/8,0,M_PI/8,M_PI/4}},{100000,10000,1000000});
 auto robot=input.odometry->pose.pose.position;
 explore::Polygon2 boundary{{{robot.x-16,robot.y-16},{robot.x+16,robot.y-16},{robot.x+16,robot.y+16},{robot.x-16,robot.y+16}}};
 for(double coarse:{1.,.4,.2}){
 core::SparseGridGeometry cg(fg.frame_id(),coarse,{},
 {(long)std::floor(minx/coarse+1e-8),(long)std::floor(miny/coarse+1e-8)},
 {(long)std::ceil(maxx/coarse-1e-8),(long)std::ceil(maxy/coarse-1e-8)});
 auto projection=bridge::ExplorationMapProjector{}.ProjectConservative(*fine,cg);
 std::ofstream c(out/("coarse-"+std::to_string(coarse)+".csv"));c<<std::setprecision(17)<<"x,y,resolution,state\n";
 auto begin=std::chrono::steady_clock::now();
 explore::GridGeometry eg{(uint32_t)cg.width(),(uint32_t)cg.height(),coarse,cg.min_inclusive().x*coarse,cg.min_inclusive().y*coarse,0.};
 explore::OccupancyGridView ev(eg,projection.data,100);
 auto task=explore::TaskRaster::Build(ev,boundary);
 auto frontiers=explore::FrontierDetector({1.}).DetectAll(task);
 auto candidates=generator.Generate(task,frontiers.clusters);
 auto robot_cell=task.WorldToCell({robot.x,robot.y});
 if(!robot_cell)throw std::runtime_error("robot outside task geometry");
 auto reachable=explore::FrontierDetector({1.}).Detect(task,*robot_cell);
 double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
 exploration<<coarse<<','<<frontiers.map_backed_free_cell_count<<','<<frontiers.clusters.size()<<','<<candidates.size()<<','<<(task.Classify(*robot_cell)==explore::CellState::kFree)<<','<<reachable.reachable_free_cell_count<<','<<elapsed<<'\n';
 std::ofstream candidate_rows(out/("candidates-"+std::to_string(coarse)+".csv"));candidate_rows<<"x,y,yaw\n";
 for(auto&v:candidates)candidate_rows<<v.pose.x<<','<<v.pose.y<<','<<v.pose.yaw<<'\n';
 size_t n=0;for(long y=cg.min_inclusive().y;y<cg.max_exclusive().y;++y)for(long x=cg.min_inclusive().x;x<cg.max_exclusive().x;++x)
 c<<(x+.5)*coarse<<','<<(y+.5)*coarse<<','<<coarse<<','<<int(projection.data[n++])<<'\n';
 }
 std::cout<<"stamp_ns="<<stamp_ns<<" source="<<g.width<<"x"<<g.height<<" fine_resolution="<<r<<" hard_radius="<<core::CircumscribedRadius(profile.planar_envelope_xy_m)<<"\n";
 return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
