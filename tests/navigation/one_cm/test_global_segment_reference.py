"""冻结30e5ee14连续认证作为独立差分oracle。"""
import numpy as np
from scripts.navigation.one_cm.nav1cm.global_planner import _segment_clear

def _segment_clear_reference(a, b, boxes, corner_radius=0):
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
    if corner_radius:
        raw = boxes + np.array([corner_radius, corner_radius, -corner_radius, -corner_radius])
        if not _segment_clear_reference(a, b, raw):
            return False
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        # 凸矩形与线段的最短距离出现在端点到矩形或角点到线段。
        delta = np.maximum(np.maximum(raw[:, :2]-a, a-raw[:, 2:]), 0.)
        distance2 = np.sum(delta*delta, axis=1)
        delta = np.maximum(np.maximum(raw[:, :2]-b, b-raw[:, 2:]), 0.)
        distance2 = np.minimum(distance2, np.sum(delta*delta, axis=1))
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


def test_rounded_segment_certification_matches_frozen_reference():
    rng = np.random.default_rng(209)
    for offset in (0., -1e6, 1e6, -1e9, 1e9):
        for _ in range(90):
            raw = rng.uniform(-3, 3, (6, 2))
            raw = np.c_[raw, raw+rng.uniform(.01, 2, (6, 2))]+offset
            radius = .75
            boxes = raw+[-radius, -radius, radius, radius]
            a, b = rng.uniform(-5, 5, (2, 2))+offset
            for aa, bb in ((a,b), (b,a), (a,a)):
                for obstacles in (boxes, boxes[::-1], boxes[:0], np.vstack((boxes, boxes))):
                    assert _segment_clear(aa,bb,obstacles,corner_radius=radius) == _segment_clear_reference(aa,bb,obstacles,corner_radius=radius)


def test_contacts_parallel_signed_zero_and_nextafter_match_reference():
    radius = .75
    boxes = np.array([[-radius,-radius,1+radius,1+radius]])
    for x in (-radius, np.nextafter(-radius, -np.inf), np.nextafter(-radius, np.inf), -.6, -.5):
        for a,b in [((x,0.),(x,1.)), ((x,-0.),(x,-0.)), ((x,x),(x,x)),
                    ((-1.3,.1),(.1,-1.3)), ((-.8,.2),(.2,-.8))]:
            assert _segment_clear(a,b,boxes,corner_radius=radius) == _segment_clear_reference(a,b,boxes,corner_radius=radius)
            assert _segment_clear(b,a,boxes,corner_radius=radius) == _segment_clear_reference(b,a,boxes,corner_radius=radius)
    # 第一框无关不能遮蔽后框碰撞；新地图每次重新认证。
    distant = boxes+100
    assert _segment_clear((0.,0.),(1.,1.),distant,corner_radius=radius)
    assert not _segment_clear((0.,0.),(1.,1.),np.vstack((distant,boxes)),corner_radius=radius)


def test_collision_free_broad_phase_uses_bounded_scalar_prefix(monkeypatch):
    from scripts.navigation.one_cm.nav1cm import global_planner as api
    raw = np.array([(float(i), float(i)+10., float(i)+.2, float(i)+10.2)
                    for i in range(1000)])
    boxes = raw+[-.75,-.75,.75,.75]
    calls = []
    original = api._segment_hits_box
    def recording(a,b,box):
        calls.append(1)
        return original(a,b,box)
    monkeypatch.setattr(api, '_segment_hits_box', recording)
    a,b = (0.,0.),(1000.,1000.)
    assert api._segment_clear(a,b,boxes,corner_radius=.75) == _segment_clear_reference(a,b,boxes,corner_radius=.75)
    assert len(calls) <= 8
