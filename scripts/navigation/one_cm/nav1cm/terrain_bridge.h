#pragma once

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#define NAV1CM_TERRAIN_EXPORT __declspec(dllexport)
#else
#define NAV1CM_TERRAIN_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

// regions 每行是 [x0,y0,x1,y1,state]，闭开源格矩形；state 为 -1/0/1。
// 坐标以 origin 为格角、yaw 为源格轴朝向。数据归 handle 所有，仅在下次
// apply 或 destroy 前有效。调用方应复制后再释放锁，不得修改这块内存。
typedef struct Nav1cmTerrainResultV2 {
  double resolution_m;
  double origin_x_m;
  double origin_y_m;
  double yaw;
  const int64_t* regions;
  size_t region_count;
  size_t lattice_count;
  // source_id 在单 handle 生命周期内单调分配且不复用；0 表示没有新观测。
  // 淘汰源后必须同步清理调用方的派生缓存；0 表示本次没有淘汰。
  uint64_t source_id;
  uint64_t evicted_source_id;
  size_t cache_capacity;
} Nav1cmTerrainResultV2;

// 最近一次成功 apply 的有中心高程源格，含直接观测和受影响邻域。
// x/y 相对 V2 结果源格原点；height_m 是累计中心高程（含 z 偏移），
// state 为 -1/0/1；slope_rad 和 relief_m 为同一次 intrinsic 求值结果。
// 缓冲区属于 handle，下次 apply 或 destroy 后失效；调用方不得修改。
typedef struct Nav1cmTerrainMeasurementV3 {
  int64_t x, y;
  double height_m;
  int64_t state;
  double slope_rad, relief_m;
} Nav1cmTerrainMeasurementV3;

NAV1CM_TERRAIN_EXPORT int nav1cm_terrain_measurements_v3(
    void* handle, const Nav1cmTerrainMeasurementV3** measurements, size_t* count);

NAV1CM_TERRAIN_EXPORT int nav1cm_terrain_abi_version(void);
NAV1CM_TERRAIN_EXPORT const char* nav1cm_terrain_last_error_v2(void);
NAV1CM_TERRAIN_EXPORT void* nav1cm_terrain_create_v2(
    double maximum_slope_rad, double maximum_local_obstacle_relief_m,
    double minimum_underbody_clearance_m);
// 平台窗口入口；1轮式/2足式。旧三参数入口仅保留legacy3x3兼容。
NAV1CM_TERRAIN_EXPORT void* nav1cm_terrain_create_platform_window_v2(
    int platform_kind, double long_side_m, double maximum_slope_rad,
    double maximum_relief_m, double minimum_clearance_m);
NAV1CM_TERRAIN_EXPORT void nav1cm_terrain_destroy_v2(void* handle);

// 输入是按升序 [y,x] 连续排列的原始高程，NaN/Inf 为没有新观测。
// 几何由源网格的左下角、真实分辨率及 map 中的平面 yaw 给定；高程加
// z_offset 后累计。源网格先分类，再由 Python 保守转换成厘米区域。
// 成功为 0，错误为 1；异常不会越过 C ABI。单 handle 由调用方串行调用。
// 内部最多累计 8 个源，有限高程观测刷新 LRU；全 NaN/Inf 不分配、不淘汰。
NAV1CM_TERRAIN_EXPORT int nav1cm_terrain_apply_v2(
    void* handle, const float* elevation_m, size_t height, size_t width,
    double resolution_m, double origin_x_m, double origin_y_m, double yaw,
    double z_offset_m, Nav1cmTerrainResultV2* result);

// op: 0 intersection, 1 difference. Clockwise input is normalized.
// Query pointers remain valid until result destruction; offsets contain count + 1 entries.
NAV1CM_TERRAIN_EXPORT int nav1cm_polygon_boolean_v1(
    int op, const double* subject_xy, size_t subject_count,
    const double* clipper_xy, size_t clipper_count, double area_epsilon, void** result);
NAV1CM_TERRAIN_EXPORT int nav1cm_polygon_query_v1(
    const void* result, const double** xy, const size_t** offsets, size_t* polygon_count);
NAV1CM_TERRAIN_EXPORT void nav1cm_polygon_destroy_v1(void* result);

#ifdef __cplusplus
}
#endif
