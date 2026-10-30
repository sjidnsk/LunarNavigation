"""隔离推理启动钩子：保留acados编码器注册并启用禁止训练守卫。"""
import os
import future_fstrings
future_fstrings.register()
from inference_only_guard import install
install(os.environ['INFERENCE_ONLY_GUARD_DIR'])
