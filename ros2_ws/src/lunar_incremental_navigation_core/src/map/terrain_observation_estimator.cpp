#include "map/terrain_observation_estimator.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
#include "map/grid_bounds.hpp"
#include "map/terrain_surface_fit.hpp"

namespace lunar::incremental_navigation::map {
int ObservationBackgroundRadiusCells(double half_width_m, double resolution_m) {
  if (!std::isfinite(half_width_m) || half_width_m < 0.0 ||
      !std::isfinite(resolution_m) || resolution_m <= 0.0)
    throw std::invalid_argument("invalid terrain background geometry");
  const double cells = std::ceil(std::nextafter(half_width_m / resolution_m, 0.0));
  if (cells > 32.0)
    throw std::invalid_argument("terrain background exceeds 32-cell half-width");
  return std::max(2, static_cast<int>(cells));
}

TerrainObservationEstimator::TerrainObservationEstimator(
    const ElevationRangeView& view, TerrainObservationParameters parameters)
    : view_(view), parameters_(parameters) {
  const double resolution = view.geometry().resolution_m();
  if (TerrainFitMethodName(parameters.fit_method) == "invalid" ||
      !std::isfinite(parameters.support_axis_distance_m) ||
      parameters.support_axis_distance_m <= 0. ||
      !std::isfinite(parameters.maximum_plane_residual_m) ||
      parameters.maximum_plane_residual_m <= 0. ||
      !std::isfinite(parameters.minimum_confidence) ||
      parameters.minimum_confidence <= 0. || parameters.minimum_confidence > 1. ||
      !std::isfinite(resolution) || resolution <= 0.)
    throw std::invalid_argument("invalid terrain observation parameters");
  const double cells = std::floor((1.5 * parameters.support_axis_distance_m + 1e-12) / resolution);
  if (cells > 32.) throw std::invalid_argument("terrain observation radius exceeds 32 cells");
  radius_ = static_cast<int>(cells);
}
const std::optional<GroundEstimate>& TerrainObservationEstimator::Estimate(GridIndex index) {
  static const std::optional<GroundEstimate> empty;
  if (!view_.geometry().Contains(index)) return empty;
  const auto floor_tile = [](std::int64_t value) {
    return value / kCacheWidth - (value % kCacheWidth < 0 ? 1 : 0);
  };
  const TileIndex key{floor_tile(index.x), floor_tile(index.y)};
  if (!last_tile_ || key != last_tile_index_) {
    auto& entry = tiles_[key];
    if (!entry) entry = std::make_unique<Tile>();
    last_tile_ = entry.get();
    last_tile_index_ = key;
  }
  auto* tile = last_tile_;
  const auto modulo = [](std::int64_t value) {
    const auto remainder = value % kCacheWidth;
    return remainder < 0 ? remainder + kCacheWidth : remainder;
  };
  const auto offset = static_cast<std::size_t>(modulo(index.y) * kCacheWidth + modulo(index.x));
  if (!tile->populated.test(offset)) {
    tile->values[offset] = Compute(index);
    tile->populated.set(offset);
    ++evaluated_;
  }
  return tile->values[offset];
}
std::size_t TerrainObservationEstimator::allocated_bytes() const noexcept {
  return tiles_.size() * sizeof(Tile);
}
std::optional<GroundEstimate> TerrainObservationEstimator::Compute(GridIndex index) const {
  const double resolution = view_.geometry().resolution_m();
  const bool measured = view_.ElevationRangeAt(index).has_value();
  if (radius_ < 1 || (!measured && resolution > .10 + 1e-12)) return {};
  std::vector<SurfaceSample> points;
  points.reserve((2*radius_+1)*(2*radius_+1));
  const int preferred = std::max(1, static_cast<int>(std::floor(
      (parameters_.support_axis_distance_m + 1e-12) / resolution)));
  std::vector<int> radii;
  for(int r=preferred;r>=1;--r) radii.push_back(r);
  for(int r=preferred+1;r<=radius_;++r) radii.push_back(r);
  std::optional<GroundEstimate> remote_jump_fallback;
  // Every candidate uses only the immutable input, never previous predictions.
  for (int r : radii) {
    points.clear();
    for(int y=-r;y<=r;++y) for(int x=-r;x<=r;++x) {
      const auto cell=detail::OffsetWithinGeometry(view_.geometry(),index,x,y);
      const auto range=cell?view_.ElevationRangeAt(*cell):std::nullopt;
      if(range) points.push_back({x*resolution,y*resolution,range->min_m,
          range->max_m,.5*(double(range->min_m)+double(range->max_m))});
    }
    if(points.size()<4) continue;
    std::vector<double> weights(points.size(),1.);
    double a=0.,b=0.,c=0.;
    const auto fit = [&]() {
      double n=0.,sx=0.,sy=0.,sz=0.,xx=0.,yy=0.,xy=0.,xz=0.,yz=0.;
      for(std::size_t i=0;i<points.size();++i) {
        const auto& p=points[i];const double w=weights[i];
        n+=w;sx+=w*p.x;sy+=w*p.y;sz+=w*p.z;
        xx+=w*p.x*p.x;yy+=w*p.y*p.y;xy+=w*p.x*p.y;
        xz+=w*p.x*p.z;yz+=w*p.y*p.z;
      }
      if(n<3.)return false;
      xx-=sx*sx/n;yy-=sy*sy/n;xy-=sx*sy/n;xz-=sx*sz/n;yz-=sy*sz/n;
      const double det=xx*yy-xy*xy;
      if(!(xx>0. && yy>0. && det>1e-4*(xx+yy)*(xx+yy)))return false;
      a=(xz*yy-yz*xy)/det;b=(yz*xx-xz*xy)/det;c=(sz-a*sx-b*sy)/n;
      return std::isfinite(a)&&std::isfinite(b)&&std::isfinite(c);
    };
    std::optional<SurfaceFit> surface;
    std::size_t inliers=0;
    if(parameters_.fit_method == TerrainFitMethod::kBaseline) {
    bool valid=fit();
    for(int iteration=0;valid && iteration<4;++iteration) {
      double maximum_residual=0.;
      for(const auto& p:points)maximum_residual=std::max(maximum_residual,std::abs(p.z-c-a*p.x-b*p.y));
      if(maximum_residual<1e-10)break;
      for(std::size_t i=0;i<points.size();++i) {
        const auto& p=points[i];
        weights[i]=std::min(1.,.5*parameters_.maximum_plane_residual_m/
            std::max(1e-12,std::abs(p.z-c-a*p.x-b*p.y)));
      }
      valid=fit();
    }
    if(!valid)continue;
    for(std::size_t i=0;i<points.size();++i) {
      const auto& p=points[i];
      weights[i]=std::abs(p.z-c-a*p.x-b*p.y)<=parameters_.maximum_plane_residual_m?1.:0.;
      inliers+=weights[i]>0.;
    }
    if(inliers<4 || double(inliers)/points.size()<.7 || !fit())continue;
    } else {
      surface=FitTerrainSurface(parameters_.fit_method,points,
          {.support_distance_m=parameters_.support_axis_distance_m,
           .resolution_m=resolution,.residual_limit_m=parameters_.maximum_plane_residual_m});
      if(!surface) continue;
      c=surface->Height(0,0); const auto gradient=surface->Gradient(0,0);
      a=gradient.x; b=gradient.y;
      if(!std::isfinite(c)||!std::isfinite(a)||!std::isfinite(b)) continue;
      for(std::size_t i=0;i<points.size();++i) {
        weights[i]=surface->inliers[i]?1.:0.; inliers+=surface->inliers[i];
      }
      if(inliers<4 || double(inliers)/points.size()<.7) continue;
      // Rejected points cannot supply the missing second spatial dimension.
      // Apply the baseline's final-consensus rank check without refitting a curve.
      double sx=0,sy=0,xx=0,yy=0,xy=0;
      for(std::size_t i=0;i<points.size();++i) if(weights[i]>0) {
        const auto& p=points[i];sx+=p.x;sy+=p.y;xx+=p.x*p.x;yy+=p.y*p.y;xy+=p.x*p.y;
      }
      xx-=sx*sx/inliers;yy-=sy*sy/inliers;xy-=sx*sy/inliers;
      if(!(xx>0 && yy>0 && xx*yy-xy*xy>1e-4*(xx+yy)*(xx+yy))) continue;
    }
    const auto height=[&](double x,double y) {
      return surface ? surface->Height(x,y) : c+a*x+b*y;
    };
    std::vector<double> angles;
    double nearest=std::numeric_limits<double>::infinity(),squared=0.;
    double low=std::numeric_limits<double>::infinity(),high=-low;
    for(std::size_t i=0;i<points.size();++i) {
      const auto& p=points[i];const double plane=height(p.x,p.y);
      low=std::min(low,p.low-plane);high=std::max(high,p.high-plane);
      if(weights[i]==0.)continue;
      squared+=(p.z-plane)*(p.z-plane);
      nearest=std::min(nearest,std::hypot(p.x,p.y));
      if(p.x!=0. || p.y!=0.)angles.push_back(std::atan2(p.y,p.x));
    }
    // Consensus only defines ground. Raw interval extremes remain a hard guard
    // and are separately consumed by platform obstacle/step evaluation.
    if(std::max(std::abs(low),std::abs(high))>.10 ||
       nearest>parameters_.support_axis_distance_m+1e-12 || angles.size()<3)continue;
    bool discontinuous=false, remote_discontinuity=false, remote_consensus_conflict=false;
    const auto record_jump = [&](bool local, bool all_on_ground) {
      if(local)discontinuous=true;
      else {
        remote_discontinuity=true;
        remote_consensus_conflict |= all_on_ground;
      }
    };
    // With every endpoint within 2 cm of the plane, an adjacent detrended
    // jump is <=4 cm and a second difference <=8 cm: all guards below pass.
    const bool curved=parameters_.fit_method==TerrainFitMethod::kMovingLeastSquares ||
        parameters_.fit_method==TerrainFitMethod::kGaussianProcess;
    if(curved || std::max(std::abs(low),std::abs(high))>.02) for(const auto& p:points) {
      // Only evidence inside the target's own central patch can veto its
      // estimate. Remote returns still affect fit consensus/confidence and
      // remain raw obstacles when their own cells are evaluated.
      const int px=static_cast<int>(std::lround(p.x/resolution));
      const int py=static_cast<int>(std::lround(p.y/resolution));
      const bool local_point=std::abs(px)<=1 && std::abs(py)<=1;
      const double plane=height(p.x,p.y);
      const bool p_on_ground=std::abs(p.z-plane)<=parameters_.maximum_plane_residual_m;
      if(p.high-p.low>parameters_.maximum_plane_residual_m &&
         std::max(std::abs(p.low-plane),std::abs(p.high-plane))>
             parameters_.maximum_plane_residual_m)
        record_jump(local_point,p_on_ground);
      // A raw adjacent jump cannot be interpreted as a smooth interpolated
      // ramp. Subtract the fitted trend so genuine steep planes still pass.
      for(const auto offset: {GridIndex{1,0},GridIndex{0,1}}) {
        const int qx=px+static_cast<int>(offset.x),qy=py+static_cast<int>(offset.y);
        if(qx>r || qy>r)continue;
        const bool local_pair=local_point && std::abs(qx)<=1 && std::abs(qy)<=1;
        const auto cell=detail::OffsetWithinGeometry(view_.geometry(),index,qx,qy);
        const auto range=cell?view_.ElevationRangeAt(*cell):std::nullopt;
        const double qz=range?.5*(double(range->min_m)+range->max_m):0.;
        const bool q_on_ground=range && std::abs(surface?qz-height(resolution*qx,resolution*qy):
            qz-c-resolution*(a*qx+b*qy))<=parameters_.maximum_plane_residual_m;
        const int lx=px-static_cast<int>(offset.x),ly=py-static_cast<int>(offset.y);
        if(lx>=-r && ly>=-r && range) {
          const auto left=detail::OffsetWithinGeometry(view_.geometry(),index,lx,ly);
          const auto lrange=left?view_.ElevationRangeAt(*left):std::nullopt;
          if(lrange && std::abs(.5*(double(lrange->min_m)+lrange->max_m)+
              .5*(double(range->min_m)+range->max_m)-2.*p.z)>.08)
            record_jump(local_pair && std::abs(lx)<=1 && std::abs(ly)<=1,
                p_on_ground && q_on_ground &&
                std::abs(surface?.5*(double(lrange->min_m)+lrange->max_m)-height(resolution*lx,resolution*ly):
                    .5*(double(lrange->min_m)+lrange->max_m)-c-
                    resolution*(a*lx+b*ly))<=parameters_.maximum_plane_residual_m);
        }
        if(range && std::abs(.5*(double(range->min_m)+range->max_m)-p.z-
            resolution*(a*offset.x+b*offset.y))>2.*parameters_.maximum_plane_residual_m)
          record_jump(local_pair,p_on_ground && q_on_ground);
      }
    }
    // A jump explained only by rejected outliers must not erase supported
    // ground elsewhere. A jump among the fitted ground samples means the
    // plane itself bridges distinct surfaces: never use that as fallback.
    if(remote_discontinuity && !remote_consensus_conflict) {
      struct RejectedPoint { double x,y,residual; };
      std::vector<RejectedPoint> rejected;
      for(const auto& p:points) {
        const double residual=surface?p.z-height(p.x,p.y):p.z-c-a*p.x-b*p.y;
        if(std::abs(residual)>parameters_.maximum_plane_residual_m)
          rejected.push_back({p.x,p.y,residual});
      }
      for(std::size_t i=0;i<rejected.size();++i)
        for(std::size_t j=i+1;j<rejected.size();++j) {
          const auto& p=rejected[i];const auto& q=rejected[j];
          if(p.residual*q.residual<=0. ||
             std::abs(p.residual-q.residual)>parameters_.maximum_plane_residual_m)continue;
          const double dx=q.x-p.x,dy=q.y-p.y,length2=dx*dx+dy*dy;
          const double t=length2>0.?std::clamp(-(p.x*dx+p.y*dy)/length2,0.,1.):0.;
          // Consistent alternative-surface evidence bracketing the target
          // cannot be discarded as isolated noise. Keep a central-patch-sized
          // exclusion; a coherent obstacle wholly elsewhere is still local
          // to its own cells, not a veto over the complete fitting window.
          const double x=p.x+t*dx,y=p.y+t*dy;
          if(x*x+y*y<=2.*resolution*resolution+1e-12)
            remote_consensus_conflict=true;
        }
    }
    if(discontinuous || (remote_discontinuity && remote_consensus_conflict))continue;
    const auto center=view_.ElevationRangeAt(index);
    if(center && std::abs(.5*(double(center->min_m)+center->max_m)-c)>.04)continue;
    std::sort(angles.begin(),angles.end());
    double gap=angles.front()+2.*std::acos(-1.)-angles.back();
    for(std::size_t i=1;i<angles.size();++i)gap=std::max(gap,angles[i]-angles[i-1]);
    if(gap>std::acos(-1.)+1e-10)continue;
    const double rms=std::sqrt(squared/inliers);
    const double uncertainty_factor=surface ?
        1./(1.+std::sqrt(surface->predictive_variance_m2)/.02) : 1.;
    const double confidence=uncertainty_factor*(measured?1.:.85)*(double(inliers)/points.size())*
        (.5+.5*std::min(1.,double(inliers)/8.))*
        (1.-.5*nearest/parameters_.support_axis_distance_m)/(1.+rms/.02);
    if(confidence<parameters_.minimum_confidence)continue;
    GroundEstimate candidate{c,{a,b},rms,low,high,confidence,
        surface?surface->predictive_variance_m2:0.,surface&&surface->used_fallback};
    if(remote_discontinuity) {
      // Prefer a window confined to the target's surface. If no such window
      // has enough observations, remote evidence must not veto valid ground
      // here: preserve the first supported estimate with its reduced score.
      if(!remote_jump_fallback)remote_jump_fallback=candidate;
      continue;
    }
    return candidate;
  }
  return remote_jump_fallback;
}
namespace {
// Shared terrain measurement: physical window for a robust background plane,
// unsmoothed 3x3 residual extrema for local discontinuities. Platform policies
// below keep wheel clearance and legged support/step limits independent.


template <typename Container>
double PatchMedian(Container& values, std::size_t count) {
  // Preserve the upper median for even counts without fully sorting.
  std::nth_element(values.begin(), values.begin() + count / 2,
                   values.begin() + count);
  return values[count / 2];
}

// Shared observation policy, independent of chassis size and step capability.
// These limits describe missing samples, not a robot's traversable gap size.
constexpr std::size_t kMinimumBackgroundObservedSamples = 4;


}
std::optional<TerrainPatchObservation> MeasureTerrainPatch(
    const ElevationRangeView& elevation, GridIndex center, double half_width_m,
    TerrainObservationEstimator& observations) {
  if (!elevation.geometry().Contains(center)) return {};
  const double resolution = elevation.geometry().resolution_m();
  const int radius = ObservationBackgroundRadiusCells(half_width_m, resolution);
  const int side = 2 * radius + 1;
  const std::size_t area = side * side;
  std::vector<TerrainPatchPoint> points(area);
  std::vector<double> heights(area);
  std::vector<int> indices(area, -1);
  std::size_t count = 0;
  int min_x = radius, max_x = -radius, min_y = radius, max_y = -radius;
  for (int y = -radius; y <= radius; ++y)
    for (int x = -radius; x <= radius; ++x) {
      const auto cell =
          detail::OffsetWithinGeometry(elevation.geometry(), center, x, y);
      const auto range =
          cell ? elevation.ElevationRangeAt(*cell) : std::nullopt;
      if (!range) continue;
      indices[(y + radius) * side + x + radius] = static_cast<int>(count);
      const double z = .5 * (double(range->min_m) + double(range->max_m));
      points[count++] = {x, y, range->min_m, range->max_m, z, 0.};
      min_x = std::min(min_x, x);
      max_x = std::max(max_x, x);
      min_y = std::min(min_y, y);
      max_y = std::max(max_y, y);
    }
  // A physical background is a sampling extent, not a completeness quota.
  // Keep enough measured samples and two-sided geometry; the covariance and
  // directional derivatives below still reject degenerate fits.
  if (count < kMinimumBackgroundObservedSamples || min_x >= 0 || max_x <= 0 || min_y >= 0 || max_y <= 0)
    return {};
  const std::size_t observed_count = count;
  double confidence = 1.;

  // Consume one deterministic estimate per missing cell. Only raw samples
  // remain eligible for real high/low surface support below.
  for (int y=-1;y<=1;++y) for(int x=-1;x<=1;++x) {
    const int slot=(y+radius)*side+x+radius;
    if(indices[slot]>=0) continue;
    const auto cell=detail::OffsetWithinGeometry(elevation.geometry(),center,x,y);
    if(!cell) continue;
    const auto& estimate=observations.Estimate(*cell);
    if(!estimate) continue;
    confidence=std::min(confidence,estimate->confidence);
    indices[slot]=static_cast<int>(count);
    const double z=estimate->surface_height_m;
    points[count++]={x,y,z,z,z,0.};
  }
  const bool center_available = indices[radius * side + radius] >= 0;
  bool central_complete = true, central_present = false;
  for (int y = radius - 1; y <= radius + 1; ++y)
    for (int x = radius - 1; x <= radius + 1; ++x) {
      const bool present = indices[y * side + x] >= 0;
      central_complete &= present;
      central_present |= present;
    }
  // No local evidence means no local obstacle certificate. In particular,
  // a distant steep background cannot classify an entirely missing patch.
  if (!central_present) return {};

  // Median adjacent derivatives initialize the background: a sharp step must
  // not tilt the initial fit into an artificial ramp joining its two surfaces.
  std::vector<double> gx(side * (side - 1)), gy(side * (side - 1));
  std::size_t nx = 0, ny = 0;
  for (int y = 0; y < side; ++y)
    for (int x = 0; x < side; ++x) {
      const int i = indices[y * side + x];
      if (i < 0) continue;
      if (x < side - 1 && indices[y * side + x + 1] >= 0)
        gx[nx++] =
            (points[indices[y * side + x + 1]].z - points[i].z) / resolution;
      if (y < side - 1 && indices[(y + 1) * side + x] >= 0)
        gy[ny++] =
            (points[indices[(y + 1) * side + x]].z - points[i].z) / resolution;
    }
  if (nx < 2 ||
      ny < 2) return {};
  double a = PatchMedian(gx, nx), b = PatchMedian(gy, ny);
  for (std::size_t i = 0; i < count; ++i)
    heights[i] = points[i].z - resolution * (a * points[i].x + b * points[i].y);
  double c = PatchMedian(heights, count);
  const double background_a = a, background_b = b, background_c = c;
  for (std::size_t i = 0; i < count; ++i) heights[i] = std::abs(heights[i] - c);
  // A 5 mm robust scale floor prevents nearly exact flats from producing
  // singular weights. It is a numerical/noise scale, not an obstacle threshold.
  const double cutoff =
      2.5 * std::max(.005, 1.4826 * PatchMedian(heights, count));
  double sw = 0., sx = 0., sy = 0., sz = 0., sxx = 0., syy = 0., sxy = 0.,
         sxz = 0., syz = 0.;
  for (std::size_t i = 0; i < count; ++i) {
    const auto& p = points[i];
    const double x = p.x * resolution, y = p.y * resolution;
    const double residual = p.z - (a * x + b * y + c);
    const double w = std::min(1., cutoff / std::max(std::abs(residual), 1e-12));
    sw += w;
    sx += w * x;
    sy += w * y;
    sz += w * p.z;
    sxx += w * x * x;
    syy += w * y * y;
    sxy += w * x * y;
    sxz += w * x * p.z;
    syz += w * y * p.z;
  }
  sxx -= sx * sx / sw;
  syy -= sy * sy / sw;
  sxy -= sx * sy / sw;
  sxz -= sx * sz / sw;
  syz -= sy * sz / sw;
  const double det = sxx * syy - sxy * sxy;
  if (!(sxx > 0. && syy > 0. && det > 1e-8 * sxx * syy)) return {};
  a = (sxz * syy - syz * sxy) / det;
  b = (syz * sxx - sxz * sxy) / det;
  c = (sz - a * sx - b * sy) / sw;
  double refined_slope = std::atan(std::hypot(a, b));
  double median_slope = std::atan(std::hypot(background_a, background_b));
  if (!std::isfinite(refined_slope) || !std::isfinite(median_slope) || !std::isfinite(c)) return {};
  double low = std::numeric_limits<double>::infinity(), high = -low,
         center_residual = 0.;
  for (std::size_t i = 0; i < count; ++i) {
    auto& p = points[i];
    // Use the robust background before least-squares refinement for
    // discontinuities: even a small fitted tilt must not shave a 21 cm
    // step below the existing 20 cm limit.
    const double plane =
        resolution * (background_a * p.x + background_b * p.y) + background_c;
    p.residual = p.z - plane;
    p.low -= plane;
    p.high -= plane;
    if (p.x == 0 && p.y == 0) center_residual = p.residual;
    if (std::abs(p.x) <= 1 && std::abs(p.y) <= 1) {
      low = std::min(low, p.low);
      high = std::max(high, p.high);
    }
  }
  const auto& ground = observations.Estimate(center);
  if (ground) {
    confidence = std::min(confidence, ground->confidence);
    refined_slope = median_slope = std::atan(std::hypot(
        ground->gradient.x, ground->gradient.y));
  }
  points.resize(count);
  return TerrainPatchObservation{std::move(points), observed_count,
      refined_slope, median_slope, low, high, center_residual,
      confidence, central_complete, ground.has_value(), center_available};
}
}  // namespace lunar::incremental_navigation::map
