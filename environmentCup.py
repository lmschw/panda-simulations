from panda_gym.envs.core import RobotTaskEnv
from panda_gym.pybullet import PyBullet

from taskCup import Shake
from pybulletCup import PybulletCup

from panda_gym.envs.robots.panda import Panda

class CupEnvironment(RobotTaskEnv):
    """My robot-task environment."""

    def __init__(self, render_mode):
        sim = PybulletCup(render_mode=render_mode)
        robot = Panda(sim)
        task = Shake(sim)
        super().__init__(robot, task)