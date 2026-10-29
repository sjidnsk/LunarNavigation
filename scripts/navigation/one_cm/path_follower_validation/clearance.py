"""独立实际车体矩形净空；只读 raw STATE，不导入生产几何或更改plant。"""
import math
BODY=(1.182,.818)


def pose_distance(row,boxes):
    x,y,yaw=(float(row[k]) for k in ('x','y','yaw'))
    c,s=math.cos(yaw),math.sin(yaw);hl,hw=BODY[0]/2,BODY[1]/2
    answer=math.inf
    for x0,y0,x1,y1 in boxes:
        ox,oy=(x0+x1)/2-x,(y0+y1)/2-y;hx,hy=(x1-x0)/2,(y1-y0)/2
        separated=False
        for ax,ay in ((1.,0.),(0.,1.),(c,s),(-s,c)):
            body=hl*abs(ax*c+ay*s)+hw*abs(-ax*s+ay*c)
            if abs(ox*ax+oy*ay)>body+hx*abs(ax)+hy*abs(ay):separated=True;break
        if not separated:return 0.
        # 分离凸矩形的最近点一定包含顶点到对方边；两组投影覆盖全部八顶点。
        for a in (-hl,hl):
            for b in (-hw,hw):
                px,py=x+c*a-s*b,y+s*a+c*b
                answer=min(answer,math.hypot(max(x0-px,0.,px-x1),max(y0-py,0.,py-y1)))
        for px in (x0,x1):
            for py in (y0,y1):
                dx,dy=px-x,py-y
                answer=min(answer,math.hypot(max(abs(c*dx+s*dy)-hl,0.),max(abs(-s*dx+c*dy)-hw,0.)))
    return answer


def measure(rows,obstacles_at,required):
    required=float(required)
    if not math.isfinite(required) or required<0:raise ValueError('invalid expected clearance')
    minimum_pose=minimum_swept=math.inf;worst_t=None;obstacle_samples=0
    previous=None
    for row in rows:
        if not all(math.isfinite(row[k]) for k in ('x','y','yaw','t')):raise ValueError('nonfinite raw state')
        boxes=tuple(obstacles_at(row['t']));distance=pose_distance(row,boxes)
        obstacle_samples+=bool(boxes);minimum_pose=min(minimum_pose,distance)
        lower=distance
        if previous is not None:
            if row['t']<previous['t']:raise ValueError('raw clock reversed')
            boxes=tuple(set(boxes+tuple(obstacles_at(previous['t']))))
            angle=math.atan2(math.sin(row['yaw']-previous['yaw']),math.cos(row['yaw']-previous['yaw']))
            travel=math.hypot(row['x']-previous['x'],row['y']-previous['y'])+math.hypot(*BODY)/2*abs(angle)
            # 与独立plant swept_clear相同的相邻状态插值语义；最近端点最多半段位移。
            lower=max(0.,min(pose_distance(previous,boxes),pose_distance(row,boxes))-travel/2)
        if lower<minimum_swept:minimum_swept=lower;worst_t=row['t']
        previous=row
    return {'pass':bool(rows) and minimum_swept+1e-9>=required,'required_m':required,
            'body_m':list(BODY),'state_count':len(rows),'obstacle_state_count':obstacle_samples,
            'minimum_pose_distance_m':minimum_pose if math.isfinite(minimum_pose) else None,
            'minimum_swept_lower_bound_m':minimum_swept if math.isfinite(minimum_swept) else None,
            'worst_t':worst_t,'method':'independent exact OBB/AABB vertex-edge distance minus half adjacent-pose body motion bound'}
