"""带路线弧长状态的轮式中点离散 unicycle。"""

import numpy as np


def step(state_vector, control, dt: float) -> np.ndarray:
    """状态 [x,y,yaw,v,w,s]，控制 [a,alpha,nu]，时间单位秒。

    v_mid=v+0.5*a*dt，w_mid=w+0.5*alpha*dt，
    yaw_mid=yaw+0.5*w_mid*dt；位置以 (v_mid,yaw_mid) 推进，
    yaw 以 w_mid 推进，v/w/s 分别以 a/alpha/nu 推进。
    """
    state = np.asarray(state_vector, dtype=np.float64)
    u = np.asarray(control, dtype=np.float64)
    if state.shape != (6,) or u.shape != (3,) or not np.isfinite(dt) or dt <= 0:
        raise ValueError("step needs six states, three controls, and positive dt")
    if not np.isfinite(state).all() or not np.isfinite(u).all():
        raise ValueError("step inputs must be finite")
    x, y, yaw, v, w, s = state
    a, alpha, nu = u
    v_mid = v + 0.5 * a * dt
    w_mid = w + 0.5 * alpha * dt
    yaw_mid = yaw + 0.5 * w_mid * dt
    return np.array([
        x + v_mid * np.cos(yaw_mid) * dt,
        y + v_mid * np.sin(yaw_mid) * dt,
        yaw + w_mid * dt,
        v + a * dt,
        w + alpha * dt,
        s + nu * dt,
    ], dtype=np.float64)
