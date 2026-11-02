"""平台尺度由能力配置派生，不能把足式尺寸当成轮式默认值。"""
import math
import pytest
from scripts.navigation.one_cm.nav1cm.terrain_native import WheelTerrainCapability
from test_p3_terrain_native import compiled

def test_wheel_window_uses_long_side_from_capability():
    cap=WheelTerrainCapability.from_mapping({'platform':'wheel','capability':dict(body_extent_m=[1.182,.818,1.3],maximum_slope_rad=math.radians(20),maximum_local_obstacle_relief_m=.2,minimum_underbody_clearance_m=.21)})
    assert cap.body_long_side_m==1.182
    assert cap.platform_kind=='wheel'

def test_legged_window_uses_own_dimensions_and_step_capability():
    cap=WheelTerrainCapability.from_mapping({'platform':'legged','capability':dict(body_extent_m=[.68,.33,.35],maximum_slope_rad=math.radians(20),maximum_step_height_m=.5,minimum_body_clearance_m=.3)})
    assert cap.body_long_side_m==.68
    assert cap.platform_kind=='legged'
    assert cap.maximum_local_obstacle_relief_m==.5
    assert cap.minimum_underbody_clearance_m==.3

@pytest.mark.parametrize('extent',[[0.,0.,.3],[float('nan'),.4,.3]])
def test_structured_capability_cannot_silently_drop_window_size(extent):
    with pytest.raises(ValueError):
        WheelTerrainCapability.from_mapping({'platform':'wheel','capability':dict(body_extent_m=extent,maximum_slope_rad=.3,maximum_local_obstacle_relief_m=.2,minimum_underbody_clearance_m=.21)})

def test_explicit_legacy_three_scalar_capability_stays_legacy():
    cap=WheelTerrainCapability(.3,.2,.21)
    assert cap.body_long_side_m is None


@pytest.mark.parametrize('resolution',[.05,.1,.2])
@pytest.mark.parametrize('platform,height', [('wheel',.199),('wheel',.201),('legged',.499),('legged',.501)])
def test_native_window_uses_platform_step_limits(compiled,platform,height,resolution):
    import numpy as np
    from test_p3_terrain_native import result_cells
    if platform=='wheel':
        cap=WheelTerrainCapability.from_mapping({'platform':'wheel','capability':dict(body_extent_m=[1.182,.818,1.3],maximum_slope_rad=math.radians(20),maximum_local_obstacle_relief_m=.2,minimum_underbody_clearance_m=.21)})
    else:
        cap=WheelTerrainCapability.from_mapping({'platform':'legged','capability':dict(body_extent_m=[.68,.33,.35],maximum_slope_rad=math.radians(20),maximum_step_height_m=.5,minimum_body_clearance_m=.3)})
    from scripts.navigation.one_cm.nav1cm.terrain_native import NativeTerrain
    h=np.zeros((61,61),np.float32);h[:,30:]=height
    with NativeTerrain(compiled[0],capability=cap) as terrain:
        update=terrain.apply(h,resolution_m=resolution,origin_xy_m=(0.,0.),yaw=0.)
    assert result_cells(update,h.shape)[30,30]==(0 if height<cap.maximum_local_obstacle_relief_m else 1)


def test_native_changed_halo_reaches_platform_window(compiled):
    import numpy as np
    from scripts.navigation.one_cm.nav1cm.terrain_native import NativeTerrain
    from test_p3_terrain_native import result_cells
    cap=WheelTerrainCapability(.35,.2,.21,1.182,'wheel')
    h=np.zeros((15,15),np.float32);h[7,10]=np.nan
    with NativeTerrain(compiled[0],capability=cap) as terrain:
        terrain.apply(h,resolution_m=.2,origin_xy_m=(0.,0.),yaw=0.)
        update=terrain.apply(np.zeros((1,1),np.float32),resolution_m=.2,origin_xy_m=(2.,1.4),yaw=0.)
    # 新窗口影响域超过原先一格halo，必须输出中心的新分类。
    cells=result_cells(update,(15,15))
    assert cells[7,7]==0
