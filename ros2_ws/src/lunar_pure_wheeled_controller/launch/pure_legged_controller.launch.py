"""Launch the stage-one legged vx/omega deployment policy."""
from lunar_pure_wheeled_controller.deployment_launch import generate_controller_launch


def generate_launch_description():
    return generate_controller_launch('legged', __file__)
