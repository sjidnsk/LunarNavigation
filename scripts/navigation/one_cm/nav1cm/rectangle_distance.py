"""实际旋转矩形到轴对齐矩形的欧氏距离；配置净空是圆盘膨胀。"""
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
