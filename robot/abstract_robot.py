from abc import ABC, abstractmethod

from rria_api_denso import RobotCartesianCommand, RobotJointCommand


class AbstractRobot(ABC):

    def __init__(self, workspace_name, control_name, options):
        ...

