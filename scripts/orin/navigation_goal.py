"""标准PoseStamped与显式旧Action目标；输入仅按有限浮点数处理。"""
import argparse
import json
import math
import os

def finite(text):
    value=float(text)
    if not math.isfinite(value): raise argparse.ArgumentTypeError('目标必须是有限数值')
    return value

def command(argv=None):
    parser=argparse.ArgumentParser()
    parser.add_argument('x',type=finite);parser.add_argument('y',type=finite)
    parser.add_argument('yaw',type=finite,nargs='?')
    parser.add_argument('--mode',choices=('nav1cm','incremental_v2','legged'),default=os.environ.get('LUNAR_NAVIGATION_MODE','nav1cm'))
    parser.add_argument('--frame',default=os.environ.get('LUNAR_MAP_FRAME','map'))
    parser.add_argument('--topic',default=os.environ.get('LUNAR_GOAL_TOPIC','/Car/T4/navigation/goal'))
    parser.add_argument('--action',default='/Car/T4/navigation/navigate_to_pose')
    args=parser.parse_args(argv)
    if args.mode!='nav1cm':
        data=dict(target_x_m=args.x,target_y_m=args.y,has_target_yaw=args.yaw is not None,target_yaw_rad=args.yaw or 0.0)
        return ['ros2','action','send_goal','--feedback',args.action,'lunar_planning_msgs/action/NavigateToPose',json.dumps(data)]
    yaw=args.yaw or 0.0
    data={'header':{'frame_id':args.frame},'pose':{'position':{'x':args.x,'y':args.y},'orientation':{'z':math.sin(yaw/2),'w':math.cos(yaw/2)}}}
    return ['ros2','topic','pub','--once',args.topic,'geometry_msgs/msg/PoseStamped',json.dumps(data)]

if __name__=='__main__':
    args=command();os.execvp(args[0],args)
