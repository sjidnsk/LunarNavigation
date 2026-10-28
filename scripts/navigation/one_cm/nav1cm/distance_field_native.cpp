// 精确可分离平方欧氏距离变换；ABI v1，行优先 uint8 可通行位图及 float64 输出。
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
#ifdef _WIN32
#define API extern "C" __declspec(dllexport)
#else
#define API extern "C"
#endif

// 仅有限抛物线参与下包络，避免无障碍行的 infinity-infinity。
static void edt(const double* f, int n, double* d,
                std::vector<int>& v, std::vector<double>& z) {
    int k = -1;
    for (int q = 0; q < n; ++q) {
        if (!std::isfinite(f[q])) continue;
        double s = -std::numeric_limits<double>::infinity();
        while (k >= 0) {
            const double p = v[k];
            s = ((f[q] + double(q)*q) - (f[v[k]] + p*p)) / (2.*(q-p));
            if (s > z[k]) break;
            --k;
        }
        ++k; v[k] = q; z[k] = k == 0 ? -std::numeric_limits<double>::infinity() : s;
        z[k+1] = std::numeric_limits<double>::infinity();
    }
    if (k < 0) { std::fill(d, d+n, std::numeric_limits<double>::infinity()); return; }
    int active = 0;
    for (int q = 0; q < n; ++q) {
        while (z[active+1] < q) ++active;
        const double delta = q-v[active];
        d[q] = delta*delta+f[v[active]];
    }
}
API int nav1cm_field_abi_version() { return 1; }
// 返回 0 成功、1 无效参数、2 分配或计算异常。调用者保证缓冲区各含 h*w 元素。
API int nav1cm_distance_field_v1(const uint8_t* passable, int h, int w, double r,
                                double* lower, double* gx, double* gy) {
    if (!passable || !lower || !gx || !gy || h < 2 || w < 2 || !std::isfinite(r) || r <= 0) return 1;
    try {
        const size_t count = size_t(h)*size_t(w);
        const int n = std::max(h,w);
        std::vector<double> intermediate(count), f(n), d(n), z(n+1);
        std::vector<int> v(n);
        for (int y=0; y<h; ++y) {
            for (int x=0; x<w; ++x) f[x] = passable[size_t(y)*w+x] ? std::numeric_limits<double>::infinity() : 0.;
            edt(f.data(), w, intermediate.data()+size_t(y)*w, v, z);
        }
        const double half_diagonal = std::sqrt(2.)*r/2.;
        for (int x=0; x<w; ++x) {
            for (int y=0; y<h; ++y) f[y] = intermediate[size_t(y)*w+x];
            edt(f.data(), h, d.data(), v, z);
            const double cx = (x+.5)*r;
            const double edge_x = std::min(cx, w*r-cx);
            for (int y=0; y<h; ++y) {
                const double cy=(y+.5)*r;
                const double edge_y=std::min(cy,h*r-cy);
                lower[size_t(y)*w+x] = std::min(std::min(std::max(0.,std::sqrt(d[y])*r-half_diagonal),edge_y),edge_x);
            }
        }
        // np.gradient(edge_order=1)：边缘单边差分，其余中心差分。
        for (int y=0; y<h; ++y) for (int x=0; x<w; ++x) {
            const int xl=std::max(0,x-1), xr=std::min(w-1,x+1);
            const int yl=std::max(0,y-1), yr=std::min(h-1,y+1);
            gx[size_t(y)*w+x]=(lower[size_t(y)*w+xr]-lower[size_t(y)*w+xl])/((xr-xl)*r);
            gy[size_t(y)*w+x]=(lower[size_t(yr)*w+x]-lower[size_t(yl)*w+x])/((yr-yl)*r);
        }
        return 0;
    } catch (...) { return 2; }
}
