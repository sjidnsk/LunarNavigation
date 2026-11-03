#include "lunar_incremental_navigation_core/platform_terrain_window.hpp"
#include "map/terrain_window_statistics.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <type_traits>
namespace lunar::incremental_navigation {
namespace {
using MedianQueue=map::SlidingTerrainMedian;
using CountQueue=map::SlidingTerrainCount;
std::optional<ElevationRange> Raw(const ElevationRangeView& v,GridIndex i){
 if(!v.geometry().Contains(i))return {};auto h=v.ElevationRangeAt(i);
 if(!h||!std::isfinite(h->min_m)||!std::isfinite(h->max_m)||h->min_m>h->max_m)return {};return h;
}
// 先验证相对位移再构造全局索引，跨出int64可表达域与跨出观测域一致处理。
std::optional<ElevationRange> RawOffset(const ElevationRangeView& v,GridIndex base,std::int64_t dx,std::int64_t dy){
 constexpr auto min=std::numeric_limits<std::int64_t>::min(),max=std::numeric_limits<std::int64_t>::max();
 if((dx>0&&base.x>max-dx)||(dx<0&&base.x<min-dx)||(dy>0&&base.y>max-dy)||(dy<0&&base.y<min-dy))return {};
 return Raw(v,{base.x+dx,base.y+dy});
}
// 仅本批输出块和物理halo的真实高程快照；不补齐未知、不读取旧3x3标量。
// 共享geometry仍是原始源格几何，索引转换保持相对lo以支持int64边缘。
class HaloElevationView final:public ElevationRangeView {
 const ElevationRangeView& source_;GridIndex base_;std::size_t pad_,cols_,rows_;
 std::vector<std::optional<ElevationRange>> values_;
 std::optional<std::size_t> Axis(std::int64_t i,std::int64_t base,std::size_t extent)const noexcept{
  if(i>=base){auto delta=static_cast<std::uint64_t>(i)-static_cast<std::uint64_t>(base);if(delta>=extent-pad_)return {};return static_cast<std::size_t>(delta)+pad_;}
  auto delta=static_cast<std::uint64_t>(base)-static_cast<std::uint64_t>(i);if(delta>pad_)return {};return pad_-static_cast<std::size_t>(delta);
 }
 public:
 HaloElevationView(const ElevationRangeView& source,GridIndex base,std::size_t width,std::size_t height,std::int64_t radius):source_(source),base_(base),pad_(static_cast<std::size_t>(radius)){
  if(width>std::numeric_limits<std::size_t>::max()-2*pad_||height>std::numeric_limits<std::size_t>::max()-2*pad_)throw std::overflow_error("terrain halo cache extent overflow");
  cols_=width+2*pad_;rows_=height+2*pad_;
  if(cols_&&rows_>std::numeric_limits<std::size_t>::max()/cols_)throw std::overflow_error("terrain halo cache storage overflow");
  values_.resize(cols_*rows_);
  auto relative=[this](std::size_t offset){return offset<pad_?-static_cast<std::int64_t>(pad_-offset):static_cast<std::int64_t>(offset-pad_);};
  for(std::size_t y=0;y<rows_;++y)for(std::size_t x=0;x<cols_;++x)values_[y*cols_+x]=RawOffset(source_,base_,relative(x),relative(y));
 }
 // 内部批量循环的相对坐标已由halo范围保证，无需逐样本重做全局几何/溢出检查。
 const std::optional<ElevationRange>& Relative(std::int64_t x,std::int64_t y)const noexcept{
  const auto col=static_cast<std::size_t>(static_cast<std::uint64_t>(x)+pad_);
  const auto row=static_cast<std::size_t>(static_cast<std::uint64_t>(y)+pad_);
  return values_[row*cols_+col];
 }
 const SparseGridGeometry& geometry()const noexcept override{return source_.geometry();}
 std::optional<ElevationRange> ElevationRangeAt(GridIndex i)const noexcept override{
  auto x=Axis(i.x,base_.x,cols_),y=Axis(i.y,base_.y,rows_);if(!x||!y)return {};return values_[*y*cols_+*x];
 }
};
void LocalRelief(const HaloElevationView& view,std::int64_t center_x,std::int64_t center_y,std::int64_t radius,LocalTerrainMeasurements& m,double a,double b){
 const auto& h0=view.Relative(center_x,center_y);if(!h0)return;const auto res=view.geometry().resolution_m();
 double lo=std::numeric_limits<double>::infinity(),hi=-lo;
 std::uint64_t known=0;GridIndex first{},second{};bool noncollinear=false;
 // 半径只受int64几何表达能力约束：差分<=INT32_MAX-1，
 // 两乘积之差仍严格落在int64范围，不引入任何物理窗口上限。
 constexpr std::int64_t bound=std::numeric_limits<std::int32_t>::max()-1;
 static_assert(bound*bound<=std::numeric_limits<std::int64_t>::max()/2);
 // 台阶不再使用局部OLS：复用L尺度的稳健背景梯度，避免局部3格
 // 恰落在双台阶上时把两次高度跳变误当作一段连续坡面。
 for(std::int64_t y=-radius;y<=radius;++y)for(std::int64_t x=-radius;x<=radius;++x){
  const auto& h=view.Relative(center_x+x,center_y+y);if(!h)continue;
  ++known;
  if(known==1)first={x,y};
  else if(known==2)second={x,y};
  else if(!noncollinear){
   const auto dx=second.x-first.x,dy=second.y-first.y;
   const auto tx=x-first.x,ty=y-first.y;
   noncollinear=dx*ty-dy*tx!=0;
  }
  const auto plane=a*x*res+b*y*res;lo=std::min(lo,double(h->min_m)-plane);hi=std::max(hi,double(h->max_m)-plane);
 }
 m.neighborhood_complete=m.neighborhood_complete&&known>=3&&noncollinear;
 m.relief_m=std::max(0.,hi-lo);m.positive_rise_m=std::max(0.,hi-.5*(double(h0->min_m)+double(h0->max_m)));
}
class MeasurementOverlay final:public ElevationRangeView {
 const ElevationRangeView& raw_;GridIndex index_;LocalTerrainMeasurements m_;
 public:MeasurementOverlay(const ElevationRangeView& raw,GridIndex i,LocalTerrainMeasurements m):raw_(raw),index_(i),m_(m){}
 const SparseGridGeometry& geometry()const noexcept override{return raw_.geometry();}
 std::optional<ElevationRange> ElevationRangeAt(GridIndex i)const noexcept override{return raw_.ElevationRangeAt(i);}
 std::optional<LocalTerrainMeasurements> TerrainMeasurementsAt(GridIndex i)const noexcept override{return i==index_?std::optional(m_):std::nullopt;}
};
class WindowEvaluator final:public PlatformElevationEvaluator {
 const ElevationRangeView& view_;double length_;std::unique_ptr<const PlatformElevationEvaluator> threshold_;
 mutable std::map<GridIndex,PlatformTerrainMeasurementBatch> blocks_;
 public:WindowEvaluator(const ElevationRangeView& v,const PlatformCapability& c):view_(v),length_(PlatformTerrainLongSide(c)),threshold_(MakePlatformElevationEvaluator(c)){}
 IntrinsicTraversalEvaluation Evaluate(const ElevationRangeView& v,GridIndex i)const override{
  if(&v!=&view_)throw std::invalid_argument("platform terrain evaluator view mismatch");if(!v.geometry().Contains(i))return {};
  auto floor=[](std::int64_t x){auto q=x/32;return x%32<0?q-1:q;};GridIndex key{floor(i.x)*32,floor(i.y)*32};
  auto End=[](std::int64_t start){constexpr auto max=std::numeric_limits<std::int64_t>::max();return start>max-32?max:start+32;};
  auto it=blocks_.find(key);if(it==blocks_.end()){auto lo=v.geometry().min_inclusive(),hi=v.geometry().max_exclusive();it=blocks_.emplace(key,MeasurePlatformTerrainBatch(v,length_,{std::max(lo.x,key.x),std::max(lo.y,key.y)},{std::min(hi.x,End(key.x)),std::min(hi.y,End(key.y))})).first;}
  auto m=it->second.At(i);if(!m)throw std::logic_error("platform terrain block mismatch");MeasurementOverlay overlay(v,i,*m);return threshold_->Evaluate(overlay,i);
 }
};
}
double PlatformTerrainLongSide(const PlatformCapability& capability){
 return std::visit([](const auto& c){double L=std::max(c.body_extent_m.x,c.body_extent_m.y);
  if constexpr(std::is_same_v<std::decay_t<decltype(c)>,WheeledCapability>){if(!(L>0)&&!c.footprint_xy_m.empty()){double xmin=std::numeric_limits<double>::infinity(),xmax=-xmin,ymin=xmin,ymax=-xmin;for(auto p:c.footprint_xy_m){xmin=std::min(xmin,p.x);xmax=std::max(xmax,p.x);ymin=std::min(ymin,p.y);ymax=std::max(ymax,p.y);}L=std::max(xmax-xmin,ymax-ymin);}}
  if(!std::isfinite(L)||L<=0)throw std::invalid_argument("platform body long side must be positive");return L;},capability);
}
std::int64_t PlatformTerrainWindowRadiusCells(double L,double res,bool local){
 if(!std::isfinite(L)||L<=0||!std::isfinite(res)||res<=0)throw std::invalid_argument("invalid platform terrain scale");
 double r=std::ceil((local?L/3.:L)/(2*res));if(!std::isfinite(r)||r>double(std::numeric_limits<std::int32_t>::max()/2))throw std::overflow_error("platform terrain radius overflow");return std::max<std::int64_t>(1,static_cast<std::int64_t>(r));
}
const LocalTerrainMeasurements* PlatformTerrainMeasurementBatch::At(GridIndex i)const noexcept{
 if(i.x<min_inclusive.x||i.y<min_inclusive.y)return nullptr;auto x=static_cast<std::uint64_t>(i.x)-static_cast<std::uint64_t>(min_inclusive.x),y=static_cast<std::uint64_t>(i.y)-static_cast<std::uint64_t>(min_inclusive.y);return x<width&&y<height?&measurements[y*width+x]:nullptr;
}
PlatformTerrainMeasurementBatch MeasurePlatformTerrainBatch(const ElevationRangeView& view,double L,GridIndex lo,GridIndex hi){
 auto R=PlatformTerrainWindowRadiusCells(L,view.geometry().resolution_m()),local=PlatformTerrainWindowRadiusCells(L,view.geometry().resolution_m(),true);
 if(hi.x<lo.x||hi.y<lo.y)throw std::invalid_argument("inverted terrain batch");
 const auto width=static_cast<std::uint64_t>(hi.x)-static_cast<std::uint64_t>(lo.x),height=static_cast<std::uint64_t>(hi.y)-static_cast<std::uint64_t>(lo.y);
 if(width>static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())||height>static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())||width>std::numeric_limits<std::size_t>::max()||height>std::numeric_limits<std::size_t>::max())throw std::overflow_error("terrain batch span overflow");
 PlatformTerrainMeasurementBatch out{lo,static_cast<std::size_t>(width),static_cast<std::size_t>(height),{}};
 if(out.width && out.height>std::numeric_limits<std::size_t>::max()/out.width)throw std::overflow_error("terrain batch overflow");
 // 输出跨度必须可迭代；实际lo/hi靠近int64端点不限制物理窗口。
 if(width>static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()-R)||height>static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()-R))throw std::overflow_error("terrain relative iteration span overflow");
 out.measurements.resize(out.width*out.height);if(!out.width||!out.height)return out;
 HaloElevationView halo(view,lo,out.width,out.height,R);
 auto res=view.geometry().resolution_m();
 std::vector<CountQueue> counts(out.width);std::vector<MedianQueue> vx(out.width),vy(out.width);
 auto mid=[](const ElevationRange& h){return .5*(double(h.min_m)+double(h.max_m));};
 // gx窗口包含L方窗内部2R条横向边、2R+1行；gy相反。
 // 横/纵分离中位使用全部行列支撑，避免用两条中心轴线漏过窄对角地形。
 // 每条边入/出平衡树常数次：O(N log R)，临时内存O(width*R)。
 for(std::int64_t y=-R;y<static_cast<std::int64_t>(height)+R;++y){
  CountQueue horizontal;MedianQueue hx,hy;
  for(std::int64_t x=-R;x<static_cast<std::int64_t>(width)+R;++x){
   const auto& current=halo.Relative(x,y);horizontal.Push(current?1:0);
   if(x>-R){const auto& left=halo.Relative(x-1,y);hx.Push(current&&left?std::optional<double>((mid(*current)-mid(*left))/res):std::nullopt);}
   if(y>-R){const auto& below=halo.Relative(x,y-1);hy.Push(current&&below?std::optional<double>((mid(*current)-mid(*below))/res):std::nullopt);}
   if(x>=R){auto col=static_cast<std::size_t>(x-R);counts[col].Push(horizontal.Total());vx[col].Push(hx.Median());if(y>-R)vy[col].Push(hy.Median());horizontal.Pop();hx.Pop();if(y>-R)hy.Pop();}
  }
  if(y<R)continue;auto row=static_cast<std::size_t>(y-R);
  for(std::size_t col=0;col<out.width;++col){auto count=counts[col].Total();auto a=vx[col].Median(),b=vy[col].Median();counts[col].Pop();vx[col].Pop();vy[col].Pop();GridIndex i{lo.x+static_cast<std::int64_t>(col),lo.y+static_cast<std::int64_t>(row)};auto& m=out.measurements[row*out.width+col];
   if(!halo.Relative(static_cast<std::int64_t>(col),static_cast<std::int64_t>(row)))continue;m.center_known=true;m.neighborhood_complete=count>=3;if(a&&b)m.slope_rad=std::atan(std::hypot(*a,*b));else m.neighborhood_complete=false;LocalRelief(halo,static_cast<std::int64_t>(col),static_cast<std::int64_t>(row),local,m,a.value_or(0.),b.value_or(0.));
  }
 }
 return out;
}
LocalTerrainMeasurements MeasurePlatformTerrain(const ElevationRangeView& view,GridIndex i,double L){if(i.x==std::numeric_limits<std::int64_t>::max()||i.y==std::numeric_limits<std::int64_t>::max())return {};auto batch=MeasurePlatformTerrainBatch(view,L,i,{i.x+1,i.y+1});return *batch.At(i);}
std::unique_ptr<const PlatformElevationEvaluator> MakePlatformWindowElevationEvaluator(const ElevationRangeView& v,const PlatformCapability& c){return std::make_unique<WindowEvaluator>(v,c);}
}
