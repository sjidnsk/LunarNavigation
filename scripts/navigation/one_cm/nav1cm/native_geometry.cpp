// 仅实现原始栅格上的矩形 SAT；动力学、采样上界、停车和身份仍由 Python 认证负责。
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

extern "C" int nav1cm_geometry_abi() { return 1; }

extern "C" int nav1cm_check_poses(
    const double* poses, int n, double margin, const std::int8_t* cells,
    const std::int32_t* prefix, int height, int width, double ox, double oy,
    double resolution, int allow_unknown, double length, double body_width, int* checked) {
    *checked = 0;
    for (int i=0; i<n; ++i) {
        *checked = i+1;
        const double x=poses[i*3], y=poses[i*3+1], yaw=poses[i*3+2];
        const double c=std::cos(yaw), s=std::sin(yaw);
        const double hl=length/2+margin, hw=body_width/2+margin;
        const double ex=std::abs(c)*hl+std::abs(s)*hw, ey=std::abs(s)*hl+std::abs(c)*hw;
        if (x-ex<ox-1e-12 || x+ex>ox+width*resolution+1e-12 ||
            y-ey<oy-1e-12 || y+ey>oy+height*resolution+1e-12) return 3;
        const int x0=std::max(0,int(std::floor(std::nextafter(x-ex-ox,-std::numeric_limits<double>::infinity())/resolution)));
        const int x1=std::min(width,int(std::floor((x+ex-ox)/resolution))+1);
        const int y0=std::max(0,int(std::floor(std::nextafter(y-ey-oy,-std::numeric_limits<double>::infinity())/resolution)));
        const int y1=std::min(height,int(std::floor((y+ey-oy)/resolution))+1);
        const int stride=width+1;
        const int count=prefix[y1*stride+x1]-prefix[y0*stride+x1]-prefix[y1*stride+x0]+prefix[y0*stride+x0];
        if (!count) continue;
        const double projection=resolution/2*(std::abs(c)+std::abs(s));
        bool unknown=false;
        for (int row=y0; row<y1; ++row) {
            // 每行前缀剪枝只跳过无危险格区域，不改变格面积判定。
            const int row_count=prefix[(row+1)*stride+x1]-prefix[row*stride+x1]-prefix[(row+1)*stride+x0]+prefix[row*stride+x0];
            if (!row_count) continue;
            for (int col=x0; col<x1; ++col) {
                const int value=cells[row*width+col];
                if (value==0 || (allow_unknown && value==-1)) continue;
                const double dx=ox+(col+.5)*resolution-x, dy=oy+(row+.5)*resolution-y;
                if (std::abs(c*dx+s*dy)<=hl+projection+1e-12 && std::abs(-s*dx+c*dy)<=hw+projection+1e-12) {
                    if (value==1) return 1;
                    unknown=true;
                }
            }
        }
        if (unknown) return 2;
    }
    return 0;
}
