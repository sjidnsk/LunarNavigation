"""冻结原场景的显式已知图演示区；不提供任意ROI或C探索真值入口。"""
import math

SOURCE_SHA='e0573e650232211b6ef0c0281fe644028855a95f823eda7f9db9d51a9e5eb70e'
C_EXCLUDED_BOUNDS=(-140.,100.,-420.,-180.)
_REGIONS={
    'A': ((235.,335.,-315.,-215.),'/World/Environment/DetailTerrain_Rockfield/DetailMesh_Rockfield','original A known-map demo; forbidden for C/Actor'),
    'KNOWN_DEMO_FLAT': ((-490.,-370.,-250.,-130.),'/World/Environment/Terrain_0_1__250m/V02_TerrainMesh_0_1','known-map flat demo from original collision geometry; not C exploration; forbidden for C/Actor'),
}
REGION_NAMES=tuple(_REGIONS)


def region_spec(region='A'):
    if not isinstance(region,str) or region not in _REGIONS: raise ValueError('only A and fixed KNOWN_DEMO_FLAT known-map regions are authorized')
    bounds,terrain,usage=_REGIONS[region]
    return dict(region=region,bounds_xy_m=list(bounds),terrain_path=terrain,usage=usage,resolution_m=.2)


def validate_region_map(region,bounds,resolution,shape=None):
    spec=region_spec(region)
    if not isinstance(bounds,(list,tuple)) or len(bounds)!=4 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in bounds): raise ValueError('invalid known-region bounds')
    x0,x1,y0,y1=map(float,bounds)
    if resolution!=.2 or x1<=x0 or y1<=y0: raise ValueError('known-region resolution/bounds mismatch')
    expected=spec['bounds_xy_m']
    if region=='KNOWN_DEMO_FLAT' and list(bounds)!=expected: raise ValueError('KNOWN_DEMO_FLAT bounds are frozen; relabelling another map is forbidden')
    if region=='A' and not (expected[0]<=x0<x1<=expected[1] and expected[2]<=y0<y1<=expected[3]): raise ValueError('map outside original A bounds')
    if x0<C_EXCLUDED_BOUNDS[1] and x1>C_EXCLUDED_BOUNDS[0] and y0<C_EXCLUDED_BOUNDS[3] and y1>C_EXCLUDED_BOUNDS[2]: raise ValueError('known-map demo must not overlap C exploration region')
    counts=((y1-y0)/resolution,(x1-x0)/resolution)
    if any(abs(value-round(value))>1e-6 for value in counts): raise ValueError('known-region map must contain whole cells')
    if shape is not None and tuple(shape)!=tuple(round(v) for v in counts): raise ValueError('known-region map dimensions mismatch')
    return spec
