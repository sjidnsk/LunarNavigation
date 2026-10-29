"""同一平台文件的运动模型选择；旧执行器保持既有能力合同。"""
MODELS=('differential_drive','curvature_limited')


def resolve_motion_model(capability,requested,*,standard_path=False,explicit=False):
    if not standard_path:
        if explicit and requested!='curvature_limited':
            raise ValueError('differential_drive requires standard Path MPPI')
        return 'curvature_limited'
    value=requested if explicit else capability.get('motion_model','curvature_limited')
    if not isinstance(value,str) or value not in MODELS:
        raise ValueError('unknown motion_model: '+str(value))
    return value
