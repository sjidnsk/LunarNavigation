#include "map/terrain_surface_fit.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace lunar::incremental_navigation::map {
namespace {
double Kernel(const SurfaceFit& model, double x, double y, const SurfaceSample& p) {
  const double dx=x-p.x,dy=y-p.y;
  return model.gp_signal_variance*std::exp(-(dx*dx+dy*dy)/(2*model.gp_length_m*model.gp_length_m));
}
bool Cholesky(std::vector<double>& a, std::size_t n) {
  for(std::size_t i=0;i<n;++i) for(std::size_t j=0;j<=i;++j) {
    double value=a[i*n+j];
    for(std::size_t k=0;k<j;++k) value-=a[i*n+k]*a[j*n+k];
    if(i==j) {
      if(!(value>1e-15) || !std::isfinite(value)) return false;
      a[i*n+j]=std::sqrt(value);
    } else a[i*n+j]=value/a[j*n+j];
  }
  return true;
}
std::vector<double> Solve(const std::vector<double>& l, std::vector<double> b) {
  const auto n=b.size();
  for(std::size_t i=0;i<n;++i) {
    for(std::size_t j=0;j<i;++j) b[i]-=l[i*n+j]*b[j];
    b[i]/=l[i*n+i];
  }
  for(std::size_t i=n;i-->0;) {
    for(std::size_t j=i+1;j<n;++j) b[i]-=l[j*n+i]*b[j];
    b[i]/=l[i*n+i];
  }
  return b;
}
}
double SurfaceFit::Height(double x, double y) const {
  const auto& c=coefficients;
  double z=c[0]+c[1]*x+c[2]*y+c[3]*x*x+c[4]*x*y+c[5]*y*y;
  for(std::size_t i=0;i<gp_samples.size();++i) z+=gp_alpha[i]*Kernel(*this,x,y,gp_samples[i]);
  return z;
}
Vec2 SurfaceFit::Gradient(double x, double y) const {
  const auto& c=coefficients;
  Vec2 gradient{c[1]+2*c[3]*x+c[4]*y,c[2]+c[4]*x+2*c[5]*y};
  for(std::size_t i=0;i<gp_samples.size();++i) {
    const double scale=-gp_alpha[i]*Kernel(*this,x,y,gp_samples[i])/(gp_length_m*gp_length_m);
    gradient.x+=scale*(x-gp_samples[i].x); gradient.y+=scale*(y-gp_samples[i].y);
  }
  return gradient;
}
std::optional<SurfaceFit> FitGaussianProcess(const std::vector<SurfaceSample>& points,
                                            const SurfaceFitOptions& options) {
  // Robust mean avoids fitting isolated spikes with a flexible kernel. Kernel
  // uncertainty is conditional on this model and says nothing about hidden voids.
  auto mean=FitWeightedPlane(points,options);
  if(!mean) return {};
  SurfaceFit model=*mean;
  model.gp_length_m=.6*options.support_distance_m;
  std::vector<std::size_t> candidates;
  double sum2=0;
  for(std::size_t i=0;i<points.size();++i) if(mean->inliers[i]) {
    candidates.push_back(i); const auto& p=points[i];
    const double residual=p.z-mean->Height(p.x,p.y); sum2+=residual*residual;
  }
  if(candidates.size()<4) return {};
  model.gp_signal_variance=std::max(.0004,sum2/candidates.size());
  // Deterministic farthest-point subset, starting at the closest sample.
  // At most 32 observations keeps the dense solve bounded on each local cell.
  std::vector<double> distances(candidates.size(),std::numeric_limits<double>::infinity());
  auto first=std::min_element(candidates.begin(),candidates.end(),[&](auto a,auto b){
    return std::hypot(points[a].x,points[a].y)<std::hypot(points[b].x,points[b].y);});
  std::size_t next=static_cast<std::size_t>(first-candidates.begin());
  for(std::size_t count=0;count<std::min<std::size_t>(32,candidates.size());++count) {
    const auto& chosen=points[candidates[next]]; model.gp_samples.push_back(chosen);
    distances[next]=-1;
    for(std::size_t i=0;i<candidates.size();++i) if(distances[i]>=0) {
      const auto& p=points[candidates[i]];
      distances[i]=std::min(distances[i],(p.x-chosen.x)*(p.x-chosen.x)+(p.y-chosen.y)*(p.y-chosen.y));
    }
    next=static_cast<std::size_t>(std::max_element(distances.begin(),distances.end())-distances.begin());
  }
  const auto n=model.gp_samples.size(); std::vector<double> matrix(n*n), residual(n), k(n);
  for(std::size_t i=0;i<n;++i) {
    const auto& p=model.gp_samples[i]; residual[i]=p.z-mean->Height(p.x,p.y);
    k[i]=Kernel(model,0,0,p);
    for(std::size_t j=0;j<n;++j) matrix[i*n+j]=Kernel(model,p.x,p.y,model.gp_samples[j]);
    const double width=p.high-p.low;
    matrix[i*n+i]+=.0001+width*width/12.+1e-10;
  }
  if(!Cholesky(matrix,n)) return {};
  model.gp_alpha=Solve(matrix,residual);
  for(double alpha:model.gp_alpha) if(!std::isfinite(alpha)) return {};
  const auto v=Solve(matrix,k); double variance=model.gp_signal_variance;
  for(std::size_t i=0;i<n;++i) variance-=k[i]*v[i];
  if(!std::isfinite(variance)) return {};
  model.predictive_variance_m2=std::max(0.,variance);
  model.inliers.assign(points.size(),false);
  for(std::size_t i=0;i<points.size();++i)
    model.inliers[i]=std::abs(points[i].z-model.Height(points[i].x,points[i].y))<=options.residual_limit_m;
  return model;
}
std::optional<SurfaceFit> FitTerrainSurface(TerrainFitMethod method,
    const std::vector<SurfaceSample>& points,const SurfaceFitOptions& options) {
  switch(method) {
    case TerrainFitMethod::kWeightedPlane: return FitWeightedPlane(points,options);
    case TerrainFitMethod::kMovingLeastSquares: return FitMovingLeastSquares(points,options);
    case TerrainFitMethod::kGroupedRansac: return FitGroupedRansac(points,options);
    case TerrainFitMethod::kGaussianProcess: return FitGaussianProcess(points,options);
    case TerrainFitMethod::kBaseline: return {};
  }
  return {};
}
}  // namespace lunar::incremental_navigation::map
