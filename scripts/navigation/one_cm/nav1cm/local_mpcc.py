"""真实 acados SQP-RTI MPCC；候选必须由调用方另外执行整车认证。"""
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter, process_time
import hashlib
import numpy as np
from .contracts import CellState, Trajectory
from .maps import build_distance_field
from .seeds import initial_seed, rollout
from .pose_goal import angle_error, terminal_candidate


# 固定 SDK acados-get-stats-source.txt 实际支持；时间值保留 SDK 原始秒单位。
ACADOS_STATS_FIELDS = ('time_tot','time_lin','time_sim','time_qp',
                       'time_qp_solver_call','sqp_iter','qp_iter')


def _solver_stats(solver):
    values, unavailable = {}, {}
    for field in ACADOS_STATS_FIELDS:
        try:
            value = np.asarray(solver.get_stats(field))
            if not np.isfinite(value).all():
                raise ValueError('nonfinite solver diagnostic')
            values[field] = value.tolist()
        except Exception as exc:
            unavailable[field] = {'type':type(exc).__name__, 'detail':str(exc)}
    return values, unavailable


class SolverUnavailable(RuntimeError):
    pass


@dataclass(frozen=True)
class LocalPlanResult:
    candidate: object
    reason: str
    timings_ms: dict
    attempts: tuple = ()


def _wrap(angle):
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def select_mode(state, reference):
    if len(reference.points)<2:
        if reference.terminal_yaw is not None and abs(state.v) <= 1e-10:
            return 'spin' if abs(angle_error(reference.terminal_yaw, state.yaw)) > .05 or abs(state.w) > 1e-10 else 'brake'
        return 'brake'
    delta=reference.points[1]-reference.points[0]
    error=_wrap(np.arctan2(delta[1],delta[0])-state.yaw)
    desired='reverse' if abs(error)>np.pi*.75 else 'forward'
    if abs(state.v)>1e-8:
        current='forward' if state.v>0 else 'reverse'
        return current if current==desired else 'brake'
    if abs(state.w)>1e-8:
        return 'spin'
    aligned=_wrap(error-(np.pi if desired=='reverse' else 0.))
    return 'spin' if abs(aligned)>.3 else desired


class MpccPlanner:
    N=20
    dt=.2

    def __init__(self, limits, build_dir, *, n=20, dt=.2):
        """build_dir 参数为可共享根目录；实例 build_dir 为配置指纹隔离目录。"""
        if type(n) is not int or n < 1 or not np.isfinite(dt) or dt <= 0:
            raise ValueError('horizon requires positive integer n and finite positive dt')
        self.N, self.dt = n, float(dt)
        self.model_fingerprint = hashlib.sha256(f'pose-terminal-v2:{self.N}:{self.dt.hex()}:{limits.digest()}'.encode()).hexdigest()[:12]
        try:
            import casadi as ca
            from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
        except ImportError as exc:
            raise SolverUnavailable('real acados_template/CasADi dependencies unavailable; no fallback') from exc
        self.limits=limits
        self.build_root=Path(build_dir).resolve()
        self.build_dir=self.build_root/self.model_fingerprint
        self.build_dir.mkdir(parents=True,exist_ok=True)
        self._warm=None
        self._solvers={}
        # 三圆覆盖整个矩形：每个圆覆盖长度 L/3、宽 W 的矩形条带。
        self.offsets=np.array([-limits.body_length_m/3,0.,limits.body_length_m/3])
        self.radius=float(np.hypot(limits.body_length_m/6,limits.body_width_m/2))
        started=perf_counter()
        for mode in ('forward','reverse','spin'):
            ocp=AcadosOcp()
            model=AcadosModel()
            model.name='nav1cm_'+mode+'_'+self.model_fingerprint
            x=ca.SX.sym('x',6); u=ca.SX.sym('u',3)
            # route xy/tangent/s/yaw, previous controls, three affine circle clearances
            p=ca.SX.sym('p',22)
            vm=x[3]+.5*u[0]*self.dt; wm=x[4]+.5*u[1]*self.dt
            ym=x[2]+.5*wm*self.dt
            model.x=x; model.u=u; model.p=p
            model.disc_dyn_expr=ca.vertcat(x[0]+vm*ca.cos(ym)*self.dt,x[1]+vm*ca.sin(ym)*self.dt,
                x[2]+wm*self.dt,x[3]+u[0]*self.dt,x[4]+u[1]*self.dt,x[5]+u[2]*self.dt)
            dx=x[0]-(p[0]+p[2]*(x[5]-p[4])); dy=x[1]-(p[1]+p[3]*(x[5]-p[4]))
            contour=-p[3]*dx+p[2]*dy; lag=p[2]*dx+p[3]*dy
            tracking=80*contour**2+100*lag**2+3*(x[2]-p[5])**2
            # s 是优化状态，-s/-nu 显式奖励进度；a/alpha 及迭代间偏移控制平滑。
            model.cost_expr_ext_cost=tracking+2*u[0]**2+2*u[1]**2+u[2]**2-2*u[2]-x[5]
            model.cost_expr_ext_cost+=.2*ca.sumsqr(u-p[6:9])
            model.cost_expr_ext_cost_e=tracking-2*x[5]
            hs=[]
            for j in range(3):
                q=p[9+4*j:13+4*j]
                hs.append(q[0]*x[0]+q[1]*x[1]+q[2]*x[2]+q[3])
            sign=-1 if mode=='reverse' else 1
            if mode!='spin':
                hs += [sign*limits.curvature_inv_m*x[3]-x[4],sign*limits.curvature_inv_m*x[3]+x[4]]
            # 刹停距离不超过有效参考余量，终点自然减速。
            # 在 |v|<=vmax 内，vmax*|v|/(2a) 保守上界 v²/(2a)，且 RTI 不再低估凸制动项。
            hs.append(p[21]-x[5]-sign*x[3]*limits.linear_speed_mps/(2*limits.linear_accel_mps2))
            model.con_h_expr=ca.vertcat(*hs); model.con_h_expr_e=ca.vertcat(*hs)
            ocp.model=model; ocp.solver_options.N_horizon=self.N
            ocp.solver_options.tf=self.N*self.dt
            ocp.solver_options.integrator_type='DISCRETE'
            ocp.solver_options.nlp_solver_type='SQP_RTI'
            ocp.solver_options.qp_solver='PARTIAL_CONDENSING_HPIPM'
            ocp.solver_options.hessian_approx='EXACT'
            ocp.solver_options.regularize_method='PROJECT'
            ocp.solver_options.qp_solver_iter_max=100
            ocp.solver_options.qp_solver_tol_stat=1e-8
            ocp.solver_options.qp_solver_tol_eq=1e-8
            ocp.solver_options.qp_solver_tol_ineq=1e-8
            ocp.solver_options.qp_solver_tol_comp=1e-8
            ocp.cost.cost_type='EXTERNAL'; ocp.cost.cost_type_e='EXTERNAL'
            vmax=limits.linear_speed_mps-1e-5; wmax=limits.angular_speed_rps-1e-5
            vlo,vhi=(-vmax,0.) if mode=='reverse' else (0.,vmax)
            if mode=='spin': vlo=vhi=0.
            ocp.constraints.idxbx=np.array([3,4,5]); ocp.constraints.idxbx_e=np.array([3,4,5])
            ocp.constraints.lbx=np.array([vlo,-wmax,0.]); ocp.constraints.ubx=np.array([vhi,wmax,1e6])
            ocp.constraints.lbx_e=ocp.constraints.lbx.copy(); ocp.constraints.ubx_e=ocp.constraints.ubx.copy()
            ocp.constraints.idxbu=np.arange(3)
            amin=limits.linear_accel_mps2-1e-5; almax=limits.angular_accel_rps2-1e-5
            ocp.constraints.lbu=np.array([0. if mode=='spin' else -amin,-almax,0.])
            ocp.constraints.ubu=np.array([0. if mode=='spin' else amin,almax,0. if mode=='spin' else vmax])
            ocp.constraints.lh=np.zeros(len(hs)); ocp.constraints.uh=np.ones(len(hs))*1e6
            ocp.constraints.lh_e=ocp.constraints.lh.copy(); ocp.constraints.uh_e=ocp.constraints.uh.copy()
            ocp.constraints.x0=np.zeros(6); ocp.parameter_values=np.zeros(22)
            ocp.code_export_directory=str(self.build_dir/mode)
            self._solvers[mode]=AcadosOcpSolver(ocp,json_file=str(self.build_dir/(mode+'.json')),verbose=False)
        self.build_ms=(perf_counter()-started)*1000

    def _parameters(self, states, controls, reference, field, target_yaw, end):
        params=[]
        for k,z in enumerate(states):
            s=float(max(reference.s[0],min(end,z[5])))
            i=max(0,min(len(reference.s)-2,int(np.searchsorted(reference.s,s,side='right'))-1))
            tangent=(reference.points[i+1]-reference.points[i])/(reference.s[i+1]-reference.s[i])
            xy=reference.points[i]+tangent*(s-reference.s[i])
            yaw=z[2]+_wrap(target_yaw-z[2])
            # 只有当前参考实际到达最终路线终点且该终点进入求解时域时，
            # 终端代价才采用目标车体 yaw；倒车也不能给最终朝向再加 pi。
            if k == len(states)-1 and reference.terminal_yaw is not None and end == float(reference.s[-1]):
                yaw=z[2]+angle_error(reference.terminal_yaw,z[2])
            p=np.zeros(22); p[:6]=[*xy,*tangent,s,yaw]
            p[6:9]=controls[min(k,len(controls)-1)]
            for j,offset in enumerate(self.offsets):
                cx=z[0]+offset*np.cos(z[2]); cy=z[1]+offset*np.sin(z[2])
                d,gx,gy=field.sample(cx,cy)
                gyaw=offset*(-gx*np.sin(z[2])+gy*np.cos(z[2]))
                p[9+4*j:13+4*j]=[gx,gy,gyaw,d-self.radius-.003-gx*z[0]-gy*z[1]-gyaw*z[2]]
            p[21]=end
            params.append(p)
        return params

    def plan(self,state,grid,reference,*,distance_field=None,field_grid_digest=None):
        """外部净空场必须显式附带其来源 grid.digest()；仅生成未认证候选。"""
        started=perf_counter(); started_cpu=process_time(); timing={'field':0.,'reference':0.,'seed':0.,'solve':0.,'candidate':0.}
        for stage in ('reset','input_set','solve_call','read_validate','diagnostics','solve'):
            timing.setdefault(stage,0.)
            timing[stage+'_cpu']=0.
        attempts=[]
        def finish(candidate,reason):
            timing['plan_total']=(perf_counter()-started)*1000
            timing['plan_total_cpu']=(process_time()-started_cpu)*1000
            return LocalPlanResult(candidate,reason,dict(timing),tuple(attempts))
        if reference.map_epoch!=grid.epoch: return finish(None,'EPOCH_MISMATCH')
        if len(reference.points)<2:
            if reference.terminal_yaw is not None:
                return finish(terminal_candidate(state,grid,reference,self.limits,horizon_s=self.N*self.dt),
                              'TERMINAL_CANDIDATE_UNCERTIFIED')
            return finish(None,'REFERENCE_END')
        if grid.cell_at_world(*reference.points[-1])==CellState.BLOCKED: return finish(None,'KNOWN_BLOCKED_GOAL')
        mode=select_mode(state,reference)
        if mode=='brake': return finish(None,'MODE_CHANGE_REQUIRES_BRAKE')
        t=perf_counter()
        if distance_field is not None:
            if field_grid_digest!=grid.digest(): raise ValueError('distance field requires exact source grid digest')
            field=distance_field
            if field.epoch!=grid.epoch or field.origin_xy!=grid.origin_xy or field.resolution_m!=grid.resolution_m:
                raise ValueError('distance field identity mismatch')
        else: field=build_distance_field(grid)
        timing['field']=(perf_counter()-t)*1000
        t=perf_counter()
        delta=reference.points[1]-reference.points[0]
        heading=float(np.arctan2(delta[1],delta[0]))
        if mode=='reverse' or (mode=='spin' and abs(_wrap(heading-state.yaw))>np.pi*.75): heading+=np.pi
        # 转向已对准但尚有角速度时，目标保持当前航向以受限角制动。
        yaw=state.yaw+_wrap(heading-state.yaw)
        if mode=='spin' and abs(_wrap(yaw-state.yaw))<=.3: yaw=state.yaw
        end=min(float(reference.s[-1]),reference.progress_s+self.N*self.dt*self.limits.linear_speed_mps)
        initial=np.array([state.x,state.y,state.yaw,state.v,state.w,reference.progress_s])
        initial_circle_feasible=all(field.sample(state.x+offset*np.cos(state.yaw),
            state.y+offset*np.sin(state.yaw))[0]>=self.radius+.003 for offset in self.offsets)
        key=(grid.digest(),reference.route_version,mode)
        if reference.terminal_yaw is not None:
            key+=(reference.terminal_yaw,)
        timing['reference']=(perf_counter()-t)*1000
        solver=self._solvers[mode]
        for attempt,side in enumerate((0,1,-1)):
            t=perf_counter()
            warmed=attempt==0 and self._warm is not None and self._warm[0]==key
            if warmed:
                controls=self._warm[2].copy(); states=rollout(initial,controls,self.dt)
            else: states,controls=initial_seed(initial,mode,self.limits,yaw,side,n=self.N,dt=self.dt)
            params=self._parameters(states,controls,reference,field,yaw,end)
            timing['seed']+=(perf_counter()-t)*1000
            row={'seed':(('warm' if warmed else 'nominal') if attempt==0 else ('left' if side>0 else 'right'))}
            def measure(stage, operation):
                wall,cpu=perf_counter(),process_time()
                value=operation()
                row[stage+'_ms']=(perf_counter()-wall)*1000
                row[stage+'_cpu_ms']=(process_time()-cpu)*1000
                timing[stage]+=row[stage+'_ms']
                timing[stage+'_cpu']+=row[stage+'_cpu_ms']
                return value
            measure('reset',solver.reset)
            def set_inputs():
                solver.set(0,'lbx',initial); solver.set(0,'ubx',initial)
                for k in range(self.N+1):
                    solver.set(k,'x',states[k]); solver.set(k,'p',params[k])
                    if k<self.N: solver.set(k,'u',controls[k])
            measure('input_set',set_inputs)
            status=int(measure('solve_call',solver.solve))
            # 旧 solve 精确包含 reset + input_set + solve_call，不含诊断及候选读取。
            row['status']=status
            for suffix, aggregate in (('_ms','solve'),('_cpu_ms','solve_cpu')):
                row['solve'+suffix]=sum(row[k+suffix] for k in ('reset','input_set','solve_call'))
                timing[aggregate]+=row['solve'+suffix]
            row['acados_stats'],row['acados_stats_unavailable']=measure('diagnostics',lambda:_solver_stats(solver))
            row['read_validate_ms']=row['read_validate_cpu_ms']=0.
            attempts.append(row)
            if status!=0:
                row['reason']='SOLVER_STATUS'
                # x0 被实测初值固定；该处硬圆约束不可行时左右初始化不可能修复。
                # 保留真实首次求解失败，只跳过已知无益的重试，不返回停车为 MPCC。
                if not initial_circle_feasible:
                    row['reason']='FIXED_INITIAL_CIRCLE_INFEASIBLE'
                    break
                continue
            def read_validate():
                us=np.array([solver.get(k,'u') for k in range(self.N)])
                # 非线性预测状态不充当动力学证据；从实测初值按相同离散模型滚动。
                # 仅清理等式约束的机器舍入，绝不修改实测初值或放宽限制。
                if mode=='spin': us[:,0]=0.; us[:,2]=0.
                us[np.abs(us)<1e-12]=0.
                xs=rollout(initial,us,self.dt)
                reason=self._candidate_reason(xs,us,mode,field,end)
                return us,xs,reason
            us,xs,reason=measure('read_validate',read_validate)
            timing['candidate']+=row['read_validate_ms']
            row['reason']=reason
            if reason!='CANDIDATE': continue
            candidate=Trajectory(grid.epoch,reference.route_version,np.arange(self.N+1)*self.dt,xs,us,mode,'mpcc:acados-sqp-rti')
            self._warm=(key,xs,us)
            return finish(candidate,'CANDIDATE_UNCERTIFIED')
        self._warm=None
        return finish(None,'NO_FEASIBLE_MPCC_CANDIDATE')

    def _candidate_reason(self,xs,us,mode,field,end):
        l=self.limits
        if not np.isfinite(xs).all() or not np.isfinite(us).all(): return 'NONFINITE'
        if (np.max(np.abs(xs[:,3]))>l.linear_speed_mps+1e-9 or np.max(np.abs(xs[:,4]))>l.angular_speed_rps+1e-9
            or np.max(np.abs(us[:,0]))>l.linear_accel_mps2+1e-9 or np.max(np.abs(us[:,1]))>l.angular_accel_rps2+1e-9
            or np.min(us[:,2]) < -1e-9): return 'LIMIT'
        if mode!='spin':
            sign=-1 if mode=='reverse' else 1
            if np.min(sign*xs[:,3]) < -1e-9 or np.any(np.abs(xs[:,4])>l.curvature_inv_m*np.abs(xs[:,3])+1e-9): return 'CURVATURE'
            if xs[-1,5]-xs[0,5]<.005: return 'NO_PROGRESS'
        if np.any(xs[:,5]+xs[:,3]**2/(2*l.linear_accel_mps2)>end+1e-6): return 'REFERENCE_BRAKING_BOUND'
        for z in xs:
            for offset in self.offsets:
                d,_,_=field.sample(z[0]+offset*np.cos(z[2]),z[1]+offset*np.sin(z[2]))
                if d<self.radius: return 'CIRCLE_CLEARANCE'
        return 'CANDIDATE'
