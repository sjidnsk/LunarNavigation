#include "lunar_incremental_navigation_ros/exploration_map.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include "lunar_incremental_navigation_ros/elevation_pipeline.hpp"

namespace lunar::incremental_navigation_ros {
namespace core=lunar::incremental_navigation;
namespace {
using Polygon=std::vector<core::Point2>;
using Values=std::map<core::GridIndex,core::IntrinsicTraversalEvaluation>;
double Cross(core::Point2 a,core::Point2 b,core::Point2 p) {
  return (b.x-a.x)*(p.y-a.y)-(b.y-a.y)*(p.x-a.x);
}
double Area(const Polygon& p) {
  double sum=0.;
  for(std::size_t i=0;i<p.size();++i) {
    const auto a=p[i], b=p[(i+1)%p.size()]; sum+=a.x*b.y-a.y*b.x;
  }
  return std::abs(sum)*.5;
}
Polygon Clip(const Polygon& p,core::Point2 a,core::Point2 b,bool inside=true) {
  Polygon result;
  if(p.empty()) return result;
  auto previous=p.back();
  double d0=Cross(a,b,previous)*(inside?1.:-1.);
  for(const auto current:p) {
    double d1=Cross(a,b,current)*(inside?1.:-1.);
    if((d0>=0.)!=(d1>=0.)) {
      const double t=d0/(d0-d1);
      result.push_back({previous.x+t*(current.x-previous.x),previous.y+t*(current.y-previous.y)});
    }
    if(d1>=0.) result.push_back(current);
    previous=current; d0=d1;
  }
  return result;
}
Polygon Intersect(Polygon a,const Polygon& b) {
  for(std::size_t i=0;i<b.size();++i) a=Clip(a,b[i],b[(i+1)%b.size()]);
  return a;
}
std::vector<Polygon> Subtract(Polygon a,const Polygon& b,double epsilon) {
  std::vector<Polygon> result;
  for(std::size_t i=0;i<b.size()&&!a.empty();++i) {
    auto outside=Clip(a,b[i],b[(i+1)%b.size()],false);
    if(Area(outside)>epsilon) result.push_back(std::move(outside));
    a=Clip(a,b[i],b[(i+1)%b.size()]);
  }
  return result;
}
Polygon Square(core::GridIndex i,double r) {
  const double x=i.x*r,y=i.y*r;
  return {{x,y},{x+r,y},{x+r,y+r},{x,y+r}};
}
struct Fragment { Polygon polygon; core::IntrinsicTraversalEvaluation value; float height; };
class SourceView final:public core::ElevationRangeView {
 public:
  explicit SourceView(const OwnedElevationEvidence& source):source_(source),
      geometry_(source.geometry.frame_id,source.geometry.resolution_m,source.geometry.origin_m,
      {0,0},{static_cast<std::int64_t>(source.geometry.width),static_cast<std::int64_t>(source.geometry.height)}) {}
  const core::SparseGridGeometry& geometry() const noexcept override {return geometry_;}
  std::optional<core::ElevationRange> ElevationRangeAt(core::GridIndex i) const noexcept override {
    if(!geometry_.Contains(i)) return std::nullopt;
    const auto h=source_.elevation_m[i.y*source_.geometry.width+i.x];
    if(!std::isfinite(h)) return std::nullopt;
    return core::ElevationRange{h,h};
  }
  std::optional<core::LocalTerrainMeasurements> TerrainMeasurementsAt(core::GridIndex i) const noexcept override {
    if(!geometry_.Contains(i)||source_.terrain_measurements.empty()) return std::nullopt;
    const auto m=source_.terrain_measurements[i.y*source_.geometry.width+i.x];
    return m.center_known?std::optional(m):std::nullopt;
  }
 private:
  const OwnedElevationEvidence& source_;
  core::SparseGridGeometry geometry_;
};
class ProjectedEvaluator final:public core::PlatformElevationEvaluator {
 public:
  explicit ProjectedEvaluator(std::shared_ptr<const Values> values):values_(std::move(values)) {}
  core::IntrinsicTraversalEvaluation Evaluate(const core::ElevationRangeView& view,core::GridIndex i) const override {
    i.x+=static_cast<std::int64_t>(std::llround(view.geometry().origin_m().x/view.geometry().resolution_m()));
    i.y+=static_cast<std::int64_t>(std::llround(view.geometry().origin_m().y/view.geometry().resolution_m()));
    const auto it=values_->find(i);return it==values_->end()?core::IntrinsicTraversalEvaluation{}:it->second;
  }
 private:
  std::shared_ptr<const Values> values_;
};
core::Vec3 Transform(const core::RigidTransform& t,core::Vec3 p) {
  auto q=t.rotation;
  const double n=std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
  q.w/=n;q.x/=n;q.y/=n;q.z/=n;
  const core::Vec3 c{2*(q.y*p.z-q.z*p.y),2*(q.z*p.x-q.x*p.z),2*(q.x*p.y-q.y*p.x)};
  return {p.x+q.w*c.x+q.y*c.z-q.z*c.y+t.translation_m.x,
          p.y+q.w*c.y+q.z*c.x-q.x*c.z+t.translation_m.y,
          p.z+q.w*c.z+q.x*c.y-q.y*c.x+t.translation_m.z};
}
bool Same(const core::IntrinsicTraversalEvaluation& a,const core::IntrinsicTraversalEvaluation& b) {
  return a.state==b.state&&a.slope_rad==b.slope_rad&&a.relief_m==b.relief_m;
}
}
struct ExplorationMap::Impl {
  core::PlatformCapability capability;
  core::TraversabilityProfile profile;
  double resolution,coarse;
  std::map<core::GridIndex,std::vector<Fragment>> history;
  std::shared_ptr<const Values> values=std::make_shared<const Values>();
  core::PersistentElevationMap elevation;
  std::shared_ptr<const core::FineTraversabilitySnapshot> fine;
  std::int64_t stamp{};
  std::deque<std::pair<std::uint64_t,std::vector<core::TileIndex>>> journal;
  Impl(core::PlatformCapability c,core::TraversabilityProfile p,double r,double coarse_value):
    capability(std::move(c)),profile(std::move(p)),resolution(r),coarse(coarse_value) {
    if(!std::isfinite(r)||r<=0||!std::isfinite(coarse)||coarse<=0) throw std::invalid_argument("invalid map resolution");
  }
};
ExplorationMap::ExplorationMap(core::PlatformCapability c,core::TraversabilityProfile p,double r,double coarse):
  impl_(std::make_unique<Impl>(std::move(c),std::move(p),r,coarse)) {}
ExplorationMap::~ExplorationMap()=default;

bool ExplorationMap::Apply(const OwnedElevationEvidence& source,std::int64_t stamp) {
  auto& m=*impl_;
  const auto& g=source.geometry;
  // 原生 Apply 仅用于输入验证及非水平变换的保守物理辅助，绝不复用其源分辨率锁。
  core::PersistentElevationMap validation;
  if(validation.Apply(source.View()).status==core::ElevationUpdateResult::Status::kRejected) return false;
  if(m.fine && source.map_from_source.parent_frame!=m.fine->geometry().frame_id()) return false;
  SourceView view(source);
  const auto evaluator=core::MakePlatformElevationEvaluator(m.capability);
  const double cell_area=m.resolution*m.resolution, epsilon=cell_area*1e-10;
  std::map<core::GridIndex,std::vector<Fragment>> incoming;
  const auto q=source.map_from_source.rotation;
  const bool upright=std::abs(q.x)+std::abs(q.y)<1e-9;
  for(std::size_t y=0;y<g.height;++y) for(std::size_t x=0;x<g.width;++x) {
    const auto h=source.elevation_m[y*g.width+x];
    if(!std::isfinite(h)) continue;
    auto classified=evaluator->Evaluate(view,{static_cast<std::int64_t>(x),static_cast<std::int64_t>(y)});
    // 倾斜坐标系的源格标量不能冒充世界水平邻域；未知保持未知。
    if(!upright && classified.state==core::IntrinsicCellState::kFree) classified.state=core::IntrinsicCellState::kUnknown;
    Polygon polygon;
    float height=0;
    for(const auto d:std::vector<core::Point2>{{0,0},{1,0},{1,1},{0,1}}) {
      auto p=Transform(source.map_from_source,{g.origin_m.x+(x+d.x)*g.resolution_m,
        g.origin_m.y+(y+d.y)*g.resolution_m,g.origin_m.z+h});
      polygon.push_back({p.x,p.y});height+=static_cast<float>(p.z*.25);
    }
    double signed_area=0.;for(std::size_t i=0;i<polygon.size();++i) signed_area+=Cross({0,0},polygon[i],polygon[(i+1)%polygon.size()]);
    if(signed_area<0) std::reverse(polygon.begin(),polygon.end());
    double minx=polygon[0].x,maxx=minx,miny=polygon[0].y,maxy=miny;
    for(auto p:polygon) {minx=std::min(minx,p.x);maxx=std::max(maxx,p.x);miny=std::min(miny,p.y);maxy=std::max(maxy,p.y);}
    for(auto iy=static_cast<std::int64_t>(std::floor(miny/m.resolution));iy<static_cast<std::int64_t>(std::ceil(maxy/m.resolution));++iy)
      for(auto ix=static_cast<std::int64_t>(std::floor(minx/m.resolution));ix<static_cast<std::int64_t>(std::ceil(maxx/m.resolution));++ix) {
        const core::GridIndex i{ix,iy};auto clipped=Intersect(polygon,Square(i,m.resolution));
        if(Area(clipped)>epsilon) incoming[i].push_back({std::move(clipped),classified,height});
      }
  }
  auto values=std::make_shared<Values>(*m.values);
  bool changed=false;
  std::set<core::TileIndex> dirty;
  std::map<core::GridIndex,float> heights;
  for(auto& [index,fragments]:incoming) {
    double incoming_area=0.;for(const auto& f:fragments) incoming_area+=Area(f.polygon);
    auto& old=m.history[index];
    // 完整覆盖替换旧源格足迹；部分覆盖只裁掉真正收到的有限观测。
    if(upright && incoming_area>=cell_area-epsilon) old.clear();
    else for(const auto& f:fragments) {
      std::vector<Fragment> retained;
      for(const auto& previous:old) for(auto polygon:Subtract(previous.polygon,f.polygon,epsilon))
        retained.push_back({std::move(polygon),previous.value,previous.height});
      old=std::move(retained);
    }
    old.insert(old.end(),std::make_move_iterator(fragments.begin()),std::make_move_iterator(fragments.end()));
    core::IntrinsicTraversalEvaluation value{.state=core::IntrinsicCellState::kFree};
    double area=0.,height=0.;
    for(const auto& f:old) {
      const auto a=Area(f.polygon);area+=a;height+=a*f.height;
      value.slope_rad=std::max(value.slope_rad,f.value.slope_rad);
      value.relief_m=std::max(value.relief_m,f.value.relief_m);
      if(f.value.state==core::IntrinsicCellState::kBlocked) value.state=core::IntrinsicCellState::kBlocked;
      else if(f.value.state==core::IntrinsicCellState::kUnknown&&value.state!=core::IntrinsicCellState::kBlocked) value.state=core::IntrinsicCellState::kUnknown;
    }
    if(area<cell_area-epsilon&&value.state!=core::IntrinsicCellState::kBlocked) value.state=core::IntrinsicCellState::kUnknown;
    const auto previous=values->find(index);
    if(previous==values->end()||!Same(previous->second,value)) {changed=true;dirty.insert(core::TileForCell(index));}
    (*values)[index]=value;
    if(area>epsilon) heights[index]=static_cast<float>(height/area);
  }
  // 高程仅作独立显示/传输字段；物理分类由上述源格证据提供。
  if(!heights.empty()) {
    auto minimum=heights.begin()->first,maximum=minimum;
    for(const auto& [i,h]:heights) {minimum.x=std::min(minimum.x,i.x);minimum.y=std::min(minimum.y,i.y);
      maximum.x=std::max(maximum.x,i.x);maximum.y=std::max(maximum.y,i.y);}
    const std::size_t width=maximum.x-minimum.x+1,height=maximum.y-minimum.y+1;
    std::vector<float> h(width*height,std::numeric_limits<float>::quiet_NaN());
    for(const auto& [i,v]:heights) h[(i.y-minimum.y)*width+i.x-minimum.x]=v;
    const core::GridGeometry canonical{.frame_id=source.map_from_source.parent_frame,.width=width,.height=height,
      .resolution_m=m.resolution,.origin_m={minimum.x*m.resolution,minimum.y*m.resolution,0.}};
    const auto result=m.elevation.Apply({.geometry=canonical,.elevation_m=h,
      .map_from_source={.parent_frame=canonical.frame_id,.child_frame=canonical.frame_id}});
    if(result.status==core::ElevationUpdateResult::Status::kApplied) changed=true;
  }
  m.stamp=stamp;
  m.values=std::move(values);
  if(!changed) return true;
  auto next=core::FineTraversabilityBuilder{}.DeriveProjected(m.elevation.Snapshot(),m.capability,m.profile,
      std::make_unique<const ProjectedEvaluator>(m.values),m.fine);
  // 全量派生的脏tile并集可安全重放；不假设原始高程revision等同分类revision。
  for(auto tile:next->tile_indices()) dirty.insert(tile);
  if(m.fine) for(auto tile:m.fine->tile_indices()) dirty.insert(tile);
  m.fine=std::move(next);
  m.journal.push_back({m.fine->fine_traversability_revision(),{dirty.begin(),dirty.end()}});
  if(m.journal.size()>128) m.journal.pop_front();
  return true;
}
ObservedPolicyMapExportInput ExplorationMap::Capture(std::uint64_t since) const {
  const auto& m=*impl_;
  ObservedPolicyMapExportInput result{.map={.fine=m.fine,.processed_stamp_ns=m.stamp},
    .intrinsic=std::make_shared<const ProjectedEvaluator>(m.values)};
  auto& out=result.map;
  if(!m.fine||since==0||since>m.fine->fine_traversability_revision()) return result;
  if(since!=m.fine->fine_traversability_revision() && (m.journal.empty()||since+1<m.journal.front().first)) return result;
  out.full_snapshot=false;out.base_revision=since;
  std::set<core::TileIndex> dirty;
  for(const auto& [revision,tiles]:m.journal) if(revision>since) dirty.insert(tiles.begin(),tiles.end());
  out.dirty_tiles={dirty.begin(),dirty.end()};return result;
}
ExplorationMapProjection ExplorationMap::Project() const {
  const auto& m=*impl_;
  if(!m.fine) return {};
  const auto& g=m.fine->geometry();const auto o=g.origin_m();
  const core::SparseGridGeometry coarse(g.frame_id(),m.coarse,{},
      {static_cast<std::int64_t>(std::floor((o.x+g.min_inclusive().x*g.resolution_m())/m.coarse)),
       static_cast<std::int64_t>(std::floor((o.y+g.min_inclusive().y*g.resolution_m())/m.coarse))},
      {static_cast<std::int64_t>(std::ceil((o.x+g.max_exclusive().x*g.resolution_m())/m.coarse)),
       static_cast<std::int64_t>(std::ceil((o.y+g.max_exclusive().y*g.resolution_m())/m.coarse))});
  return ExplorationMapProjector{}.ProjectConservative(*m.fine,coarse);
}
}  // namespace lunar::incremental_navigation_ros
