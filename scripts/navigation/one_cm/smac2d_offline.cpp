// 离线调用官方 Humble Smac Node2D；不发布ROS消息、不配置导航栈。
#include <chrono>
#include <fstream>
#include <iostream>
#include "nav2_smac_planner/a_star.hpp"
#include "rclcpp/rclcpp.hpp"
int main(int argc,char**argv){
 rclcpp::init(argc,argv);
 auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("offline_smac2d",rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false));
 unsigned w,h; double resolution,budget; unsigned sx,sy,gx,gy;
 while(std::cin>>w>>h>>resolution>>budget>>sx>>sy>>gx>>gy){
  nav2_costmap_2d::Costmap2D map(w,h,resolution,0,0,0);
  for(unsigned i=0;i<w*h;i++){unsigned value;std::cin>>value;map.getCharMap()[i]=value;}
  nav2_smac_planner::GridCollisionChecker checker(&map,1,node);
  checker.setFootprint({},true,0.0);
  nav2_smac_planner::SearchInfo info{};info.cost_penalty=1.0;
  nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::Node2D> astar(nav2_smac_planner::MotionModel::TWOD,info);
  int max_iterations=100000000;astar.initialize(true,max_iterations,1000,budget,0.0,1);
  astar.setCollisionChecker(&checker);astar.setStart(sx,sy,0);astar.setGoal(gx,gy,0);
  nav2_smac_planner::Node2D::CoordinateVector path;int iterations=0;
  auto begin=std::chrono::steady_clock::now();bool ok=false;std::string error;
  try{ok=astar.createPath(path,iterations,0.0);}catch(const std::exception&e){error=e.what();}
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  std::cout<<"RESULT "<<ok<<" "<<ms<<" "<<iterations<<" "<<path.size()<<"\n";
  for(auto it=path.rbegin();it!=path.rend();++it)std::cout<<it->x<<" "<<it->y<<"\n";
  std::cout.flush();std::cerr<<error<<"\n";
 }
 rclcpp::shutdown();
}
