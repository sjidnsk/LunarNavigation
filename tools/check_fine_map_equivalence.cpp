// Deterministic state/cost dump for comparing two core shared-library builds.
// Run with baseline and candidate LD_LIBRARY_PATH, then cmp the output files.
#include <iostream>
#include <iomanip>
#include <random>
#include <limits>
#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"
using namespace lunar::incremental_navigation;
int main(){std::cout<<std::hexfloat;
 for(int platform=0;platform<2;++platform)for(double r:{.05,.1,.2})for(int scene=0;scene<6;++scene){
 GridGeometry g{.frame_id="map",.width=48,.height=40,.resolution_m=r,.origin_m={-1.37,-2.61,0}};
 std::vector<float> z(g.CellCount());std::mt19937 rng(20260923);
 for(int y=0;y<40;++y)for(int x=0;x<48;++x){float h=0;
 if(scene==1)h=x>23?.6:0;
 if(scene==2)h=(rng()%100<8)?std::numeric_limits<float>::quiet_NaN():0;
 if(scene==3)h=float(int(rng()%7)-3)*.02;
 if(scene==4)h=float(x*r*.3+(x>23?.4:0));
 if(scene==5)h=(x>20&&x<28&&y>15&&y<25)?1:0;
 z[y*48+x]=h;}
 PersistentElevationMap map; (void)map.Apply({.geometry=g,.elevation_m=z,.map_from_source={.parent_frame="map",.child_frame="map"}});
 LeggedCapability c;c.maximum_slope_rad=.523598775598;c.maximum_step_height_m=.5;c.maximum_gap_width_m=.3;
 TraversabilityProfile p;p.planar_envelope_xy_m={{-.34,-.165},{.34,-.165},{.34,.165},{-.34,.165}};p.preferred_clearance_m=.3;p.slope_weight=1;p.relief_weight=1;p.clearance_weight=2;
 WheeledCapability w;w.maximum_slope_rad=.523598775598;w.maximum_local_obstacle_relief_m=.2;w.minimum_underbody_clearance_m=.3;
 PlatformCapability cap=platform==0?PlatformCapability(c):PlatformCapability(w);
 auto snap=map.Snapshot();FineCellEvaluator eval(*snap,cap,p);
 for(int y=0;y<40;++y)for(int x=0;x<48;++x){auto a=eval.Evaluate({x,y});std::cout<<int(a.state)<<' '<<a.traversal_cost<<'\n';}
 FineTraversabilityBuilder builder;auto fine=builder.Derive(snap,cap,p);
 for(int rev=0;rev<2;++rev){
 for(int y=10;y<24;++y)for(int x=12;x<30;++x)z[y*48+x]=rev==0?.7F:0.F;
 (void)map.Apply({.geometry=g,.elevation_m=z,.map_from_source={.parent_frame="map",.child_frame="map"}});
 fine=builder.Derive(map.Snapshot(),cap,p,fine);
 auto geom=fine->geometry();
 for(auto y=geom.min_inclusive().y;y<geom.max_exclusive().y;++y)
 for(auto x=geom.min_inclusive().x;x<geom.max_exclusive().x;++x)
 std::cout<<int(fine->State({x,y}))<<' '<<fine->TraversalCost({x,y})<<'\n';
 }

 }
}
