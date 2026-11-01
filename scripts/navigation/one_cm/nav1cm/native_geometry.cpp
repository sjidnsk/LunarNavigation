// 仅实现原始栅格上的统一圆包络或原型矩形欧氏距离；动力学、采样上界、停车和身份仍由 Python 认证负责。
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

extern "C" int nav1cm_geometry_abi() { return 3; }

namespace {
double rectangle_distance_squared(double dx,double dy,double r,double c,double s,double hl,double hw) {
    const double ex=std::abs(c)*hl+std::abs(s)*hw,ey=std::abs(s)*hl+std::abs(c)*hw;
    const double projection=r*(std::abs(c)+std::abs(s));
    if (std::abs(dx)<=ex+r+1e-12 && std::abs(dy)<=ey+r+1e-12 &&
        std::abs(c*dx+s*dy)<=hl+projection+1e-12 && std::abs(-s*dx+c*dy)<=hw+projection+1e-12) return 0.;
    double distance=std::numeric_limits<double>::infinity();
    for (int a=-1;a<=1;a+=2) for (int b=-1;b<=1;b+=2) {
        const double x=c*a*hl-s*b*hw,y=s*a*hl+c*b*hw;
        const double qx=std::max(std::abs(x-dx)-r,0.),qy=std::max(std::abs(y-dy)-r,0.);
        distance=std::min(distance,qx*qx+qy*qy);
        const double bx=dx+a*r,by=dy+b*r;
        const double px=std::max(std::abs(c*bx+s*by)-hl,0.),py=std::max(std::abs(-s*bx+c*by)-hw,0.);
        distance=std::min(distance,px*px+py*py);
    }
    return distance;
}
}

extern "C" int nav1cm_check_poses(
    const double* poses, int n, double margin, const std::int8_t* cells,
    const std::int32_t* prefix, int height, int width, double ox, double oy,
    double resolution, int allow_unknown, double length, double body_width, double circle_radius, int* checked) {
    *checked = 0;
    for (int i=0; i<n; ++i) {
        *checked = i+1;
        const double x=poses[i*3], y=poses[i*3+1], yaw=poses[i*3+2];
        const double c=std::cos(yaw), s=std::sin(yaw);
        const double hl=length/2, hw=body_width/2;
        const bool circle=circle_radius>0.;
        const double ex=circle ? circle_radius : std::abs(c)*hl+std::abs(s)*hw+margin;
        const double ey=circle ? circle_radius : std::abs(s)*hl+std::abs(c)*hw+margin;
        if (x-ex<ox-1e-12 || x+ex>ox+width*resolution+1e-12 ||
            y-ey<oy-1e-12 || y+ey>oy+height*resolution+1e-12) return 3;
        const int x0=std::max(0,int(std::floor(std::nextafter(x-ex-ox,-std::numeric_limits<double>::infinity())/resolution)));
        const int x1=std::min(width,int(std::floor((x+ex-ox)/resolution))+1);
        const int y0=std::max(0,int(std::floor(std::nextafter(y-ey-oy,-std::numeric_limits<double>::infinity())/resolution)));
        const int y1=std::min(height,int(std::floor((y+ey-oy)/resolution))+1);
        const int stride=width+1;
        const int count=prefix[y1*stride+x1]-prefix[y0*stride+x1]-prefix[y1*stride+x0]+prefix[y0*stride+x0];
        if (!count) continue;
        bool unknown=false;
        for (int row=y0; row<y1; ++row) {
            // 每行前缀剪枝只跳过无危险格区域，不改变格面积判定。
            const int row_count=prefix[(row+1)*stride+x1]-prefix[row*stride+x1]-prefix[(row+1)*stride+x0]+prefix[row*stride+x0];
            if (!row_count) continue;
            for (int col=x0; col<x1; ++col) {
                const int value=cells[row*width+col];
                if (value==0 || (allow_unknown && value==-1)) continue;
                const double dx=ox+(col+.5)*resolution-x, dy=oy+(row+.5)*resolution-y;
                const double qx=std::max(std::abs(dx)-resolution/2,0.);
                const double qy=std::max(std::abs(dy)-resolution/2,0.);
                const double distance2=circle ? qx*qx+qy*qy : rectangle_distance_squared(dx,dy,resolution/2,c,s,hl,hw);
                const double threshold=(circle ? circle_radius : margin)+1e-12;
                if (distance2<=threshold*threshold) {
                    if (value==1) return 1;
                    unknown=true;
                }
            }
        }
        if (unknown) return 2;
    }
    return 0;
}

// 可选偏好积分；不改变ABI3碰撞接口，也不认证碰撞。单次工作量有界，超限返回NaN由Python预算回退。
#include <vector>
#include <utility>
extern "C" double nav1cm_soft_path_cost(const double* points,int count,const double* boxes,int n,
                                        double soft,double weight,double radius) {
    const double nan=std::numeric_limits<double>::quiet_NaN();
    if(count<0 || n<0 || !std::isfinite(soft) || !std::isfinite(weight) || soft<0 || weight<0 || radius<0) return nan;
    double total=0.; std::uint64_t work=0;
    struct Box { double v[4]; };
    for(int edge=0;edge+1<count;++edge) {
        if(++work>2000000) return nan;
        const double* a=points+2*edge; const double* b=a+2;
        const double dx=b[0]-a[0],dy=b[1]-a[1],length=std::hypot(dx,dy);
        total+=length;
        if(!length || !soft || !weight || !n) continue;
        std::vector<Box> near; std::vector<std::pair<double,double>> intervals;
        for(int j=0;j<n;++j) {
            if(++work>2000000) return nan;
            const double* box=boxes+4*j;
            if(!(box[0]-soft<std::max(a[0],b[0]) && box[2]+soft>std::min(a[0],b[0]) &&
                 box[1]-soft<std::max(a[1],b[1]) && box[3]+soft>std::min(a[1],b[1]))) continue;
            Box raw{{box[0]+radius,box[1]+radius,box[2]-radius,box[3]-radius}};
            near.push_back(raw);
            double low=0.,high=1.; bool possible=true;
            for(int axis=0;axis<2;++axis) {
                const double delta=b[axis]-a[axis];
                if(delta==0) possible &= a[axis]>box[axis]-soft && a[axis]<box[axis+2]+soft;
                else {
                    const double t0=(box[axis]-soft-a[axis])/delta,t1=(box[axis+2]+soft-a[axis])/delta;
                    low=std::max(low,std::min(t0,t1));high=std::min(high,std::max(t0,t1));
                }
            }
            if(possible && low<high) intervals.emplace_back(low,high);
        }
        if(near.empty()) continue;
        std::sort(intervals.begin(),intervals.end());
        std::vector<std::pair<double,double>> merged;
        for(auto interval:intervals) {
            if(!merged.empty() && interval.first<=merged.back().second) merged.back().second=std::max(merged.back().second,interval.second);
            else merged.push_back(interval);
        }
        const int axis=dy==0 ? 0 : (dx==0 ? 1 : -1);
        if(axis>=0) {
            std::vector<double> cuts;
            for(const auto& box:near) for(int k:{axis,axis+2}) cuts.push_back((box.v[k]-a[axis])/(b[axis]-a[axis]));
            std::sort(cuts.begin(),cuts.end());cuts.erase(std::unique(cuts.begin(),cuts.end()),cuts.end());
            std::vector<std::pair<double,double>> split;
            for(auto interval:merged) {
                if(++work>2000000) return nan;
                double low=interval.first;
                for(double value:cuts) {
                    if(++work>2000000) return nan;
                    if(interval.first<value && value<interval.second) {split.emplace_back(low,value);low=value;}
                }
                split.emplace_back(low,interval.second);
            }
            merged=std::move(split);
        }
        for(auto interval:merged) {
            if(++work>2000000) return nan;
            const double low=interval.first,high=interval.second;
            if(axis>=0) {
                const double q0=a[axis]+low*(b[axis]-a[axis]),q1=a[axis]+high*(b[axis]-a[axis]);
                const double p0=std::min(q0,q1),p1=std::max(q0,q1);
                double constant2=std::numeric_limits<double>::infinity(),minimum2=constant2;
                const int other=1-axis;
                for(const auto& box:near) {
                    if(++work>2000000) return nan;
                    const double orth=std::max({box.v[other]-a[other],a[other]-box.v[other+2],0.});
                    const double axial=std::max({box.v[axis]-p1,p0-box.v[axis+2],0.});
                    minimum2=std::min(minimum2,orth*orth+axial*axial);
                    if(box.v[axis]<=p0 && box.v[axis+2]>=p1) constant2=std::min(constant2,orth*orth);
                }
                if(std::isfinite(constant2) && constant2<=minimum2) {
                    const double penalty=std::max(0.,1-std::max(0.,std::sqrt(constant2)-radius)/soft);
                    total+=weight*(high-low)*length*penalty*penalty;continue;
                }
            }
            const double wanted=std::ceil((high-low)*length/(soft/8));
            if(!std::isfinite(wanted) || wanted>2000000.) return nan;
            const int samples=std::max(1,int(wanted));
            if(work+std::uint64_t(samples)*near.size()>2000000) return nan;
            work+=std::uint64_t(samples)*near.size();
            double sum=0.;
            for(int sample=0;sample<samples;++sample) {
                const double t=low+(sample+.5)*(high-low)/samples;
                const double x=a[0]+t*dx,y=a[1]+t*dy;
                double nearest2=std::numeric_limits<double>::infinity();
                for(const auto& box:near) {
                    const double ex=std::max({box.v[0]-x,x-box.v[2],0.}),ey=std::max({box.v[1]-y,y-box.v[3],0.});
                    nearest2=std::min(nearest2,ex*ex+ey*ey);
                }
                const double penalty=std::max(0.,1-std::max(0.,std::sqrt(nearest2)-radius)/soft);
                sum+=penalty*penalty;
            }
            total+=weight*(high-low)*length/samples*sum;
        }
    }
    return total;
}
