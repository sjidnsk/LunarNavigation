"""Launch the wheel deployment policy, with optional explicit overrides."""
from lunar_pure_wheeled_controller.deployment_launch import generate_controller_launch


def generate_launch_description():
    return generate_controller_launch('wheel', __file__)
