"""共享矩形障碍几何：点距、连续线段和原型矩形车体距离。"""
import numpy as np


def rectangle_distance_squared(dx,dy,rx,ry,c,s,hl,hw):
    """障碍中心相对车体中心；支持同一姿态对多个障碍的向量计算。"""
    dx,dy,rx,ry=np.broadcast_arrays(dx,dy,rx,ry)
    ex,ey=abs(c)*hl+abs(s)*hw,abs(s)*hl+abs(c)*hw
    intersects=((np.abs(dx)<=ex+rx+1e-12)&(np.abs(dy)<=ey+ry+1e-12)&
        (np.abs(c*dx+s*dy)<=hl+abs(c)*rx+abs(s)*ry+1e-12)&
        (np.abs(-s*dx+c*dy)<=hw+abs(s)*rx+abs(c)*ry+1e-12))
    distance=np.full(dx.shape,np.inf)
    # 分离凸多边形的最近点包含一方顶点到另一方边的正交投影。
    for a in (-hl,hl):
        for b in (-hw,hw):
            x,y=c*a-s*b,s*a+c*b
            qx,qy=np.maximum(np.abs(x-dx)-rx,0.),np.maximum(np.abs(y-dy)-ry,0.)
            distance=np.minimum(distance,qx*qx+qy*qy)
    for ax in (-1.,1.):
        for ay in (-1.,1.):
            x,y=dx+ax*rx,dy+ay*ry
            qx,qy=np.maximum(np.abs(c*x+s*y)-hl,0.),np.maximum(np.abs(-s*x+c*y)-hw,0.)
            distance=np.minimum(distance,qx*qx+qy*qy)
    return np.where(intersects,0.,distance)


def point_box_distance_squared(point, boxes):
    """点到原始闭矩形的精确距离平方，不含车辆膨胀。"""
    point=np.asarray(point,dtype=float)
    boxes=np.asarray(boxes,dtype=float).reshape(-1,4)
    delta=np.maximum(np.maximum(boxes[:,:2]-point,point-boxes[:,2:]),0.)
    return np.einsum('...j,...j->...',delta,delta)


def point_box_clearance(point, boxes, bounds):
    """精确距离及对应梯度，包含局部窗口边界。"""
    x,y=point
    distances=[x-bounds[0],y-bounds[1],bounds[2]-x,bounds[3]-y]
    gradients=[(1.,0.),(0.,1.),(-1.,0.),(0.,-1.)]
    i=int(np.argmin(distances));d=float(distances[i]);gradient=gradients[i]
    if len(boxes):
        squared=point_box_distance_squared(point,boxes);j=int(np.argmin(squared))
        obstacle=float(np.sqrt(squared[j]))
        if obstacle<d:
            d=obstacle
            nearest=np.clip(np.asarray(point),boxes[j,:2],boxes[j,2:])
            gradient=tuple((np.asarray(point)-nearest)/d) if d>0 else (0.,0.)
    return max(0.,d),*gradient


def segment_hits_box(a, b, box):
    """闭区间slab相交；与向量参考同判据，接触算相交。"""
    lo, hi = 0., 1.
    for axis in (0, 1):
        origin, end = float(a[axis]), float(b[axis])
        low, high = float(box[axis]), float(box[axis+2])
        delta = end-origin
        if delta == 0:
            if origin < low or origin > high:
                return False
        else:
            near, far = (low-origin)/delta, (high-origin)/delta
            lo = max(lo, min(near, far))
            hi = min(hi, max(near, far))
            if lo > hi:
                return False
    return True


def segment_clear(a, b, boxes, corner_radius=0):
    """连续认证整条线段；接触也视为碰撞，不做离散采样。

    boxes是硬膨胀AABB；corner_radius>0时先内缩恢复原障碍矩形，
    用线段到原矩形的最短距离认证圆形半径，0保留原AABB语义。
    """
    if not len(boxes):
        return True
    # 包围盒闭区间相交是必要条件；仅排除确定无关障碍。
    boxes = boxes[(boxes[:, 0] <= max(a[0], b[0])) &
                  (boxes[:, 2] >= min(a[0], b[0])) &
                  (boxes[:, 1] <= max(a[1], b[1])) &
                  (boxes[:, 3] >= min(a[1], b[1]))]
    if not len(boxes):
        return True
    if corner_radius:
        raw = boxes + np.array([corner_radius, corner_radius, -corner_radius, -corner_radius])
        # 固定短前缀获得常见碰撞早退；余量沿原向量slab判据批量处理。
        # 8仅限制Python循环工作量，不改变碰撞阈值或遗漏任何障碍。
        if any(segment_hits_box(a, b, box) for box in raw[:8]):
            return False
        if len(raw) > 8 and not segment_clear(a, b, raw[8:]):
            return False
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        # 凸矩形与线段的最短距离出现在端点到矩形或角点到线段。
        distance2 = np.minimum(point_box_distance_squared(a,raw),point_box_distance_squared(b,raw))
        corners = raw[:, ((0,1), (0,3), (2,1), (2,3))]
        direction = b-a
        norm2 = float(np.dot(direction, direction))
        if norm2:
            t = np.clip(np.sum((corners-a)*direction, axis=2)/norm2, 0., 1.)
            delta = corners-a-t[:,:,None]*direction
            distance2 = np.minimum(distance2, np.sum(delta*delta, axis=2).min(axis=1))
        return not bool(np.any(distance2 <= corner_radius*corner_radius))
    lo = np.zeros(len(boxes))
    hi = np.ones(len(boxes))
    possible = np.ones(len(boxes), dtype=bool)
    for axis in (0, 1):
        delta = b[axis] - a[axis]
        if delta == 0:
            possible &= (a[axis] >= boxes[:, axis]) & (a[axis] <= boxes[:, axis + 2])
        else:
            near = (boxes[:, axis] - a[axis]) / delta
            far = (boxes[:, axis + 2] - a[axis]) / delta
            lo = np.maximum(lo, np.minimum(near, far))
            hi = np.minimum(hi, np.maximum(near, far))
    return not bool(np.any(possible & (lo <= hi)))


def points_box_clearance(points, boxes, bounds):
    """批量精确距离/梯度；N×障碍数临时数组，仅用于局部小窗口。"""
    points=np.asarray(points,dtype=float).reshape(-1,2)
    boundary=np.column_stack((points[:,0]-bounds[0],points[:,1]-bounds[1],
                              bounds[2]-points[:,0],bounds[3]-points[:,1]))
    which=np.argmin(boundary,axis=1)
    d=boundary[np.arange(len(points)),which]
    gradient=np.array([[1.,0.],[0.,1.],[-1.,0.],[0.,-1.]])[which]
    if len(boxes):
        squared=point_box_distance_squared(points[:,None,:],boxes)
        nearest=np.argmin(squared,axis=1)
        obstacle=np.sqrt(squared[np.arange(len(points)),nearest])
        selected=obstacle<d
        delta=points-np.clip(points,boxes[nearest,:2],boxes[nearest,2:])
        normal=np.divide(delta,obstacle[:,None],out=np.zeros_like(delta),where=obstacle[:,None]>0)
        d=np.where(selected,obstacle,d)
        gradient=np.where(selected[:,None],normal,gradient)
    return np.column_stack((np.maximum(d,0.),gradient))
