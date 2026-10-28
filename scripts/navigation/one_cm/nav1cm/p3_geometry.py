"""不相交源矩形的连续条带覆盖，只有几何逻辑，不含地形算法。"""


def continuous_strip_intervals(sections):
    """在无顶点事件的开条带中，保持同一连通分量的左右边界对应。

    sections 中每项是同一多边形在条带下/上端的 (left,right) 截面。
    调用方已按所有顶点 y 分段，因此边界是仿射函数；源矩形内部互不
    相交，开条带内的分量顺序和连接关系不会变化。仿射边界的平均值
    只用于确定分量身份，不是用中点抽样来证明面积覆盖。

    对每个固定分量分别取 left 的上界、right 的下界，再合并结果；
    不允许下端的某个分量和上端的另一个分量共同证明一个自由厘米格。
    """
    ordered = sorted((.5 * (lo[0] + hi[0]), .5 * (lo[1] + hi[1]),
                      lo[0], lo[1], hi[0], hi[1]) for lo, hi in sections)
    if not ordered:
        return []
    result = []
    component = list(ordered[0])
    for part in ordered[1:]:
        if part[0] <= component[1]:
            component[1] = max(component[1], part[1])
            component[2] = min(component[2], part[2])
            component[3] = max(component[3], part[3])
            component[4] = min(component[4], part[4])
            component[5] = max(component[5], part[5])
        else:
            left, right = max(component[2], component[4]), min(component[3], component[5])
            if left < right:
                result.append((left, right))
            component = list(part)
    left, right = max(component[2], component[4]), min(component[3], component[5])
    if left < right:
        result.append((left, right))
    return result
