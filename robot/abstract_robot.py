from abc import ABC, abstractmethod

from rria_api_denso import RobotCartesianCommand, RobotJointCommand


class AbstractRobot(ABC):

    def __init__(self, workspace_name, control_name, options):
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        ...

    @abstractmethod
    def motor_enabled(self) -> bool:
        ...

    @abstractmethod
    def connect(self) -> bool:
        ...

    @abstractmethod
    def disconnect(self) -> bool:
        ...

    @abstractmethod
    def motor_on(self) -> bool:
        ...

    @abstractmethod
    def motor_off(self) -> bool:
        ...

    @abstractmethod
    def move_joints(self, comand: RobotJointCommand) -> bool:
        ...

    @abstractmethod
    def move_cartesian(self, comand: RobotCartesianCommand) -> bool:
        ...

    @abstractmethod
    def get_joints_pose(self) -> RobotJointCommand | None:
        ...

    @abstractmethod
    def get_cartesian_pose(self) -> RobotCartesianCommand | None:
        ...

    @abstractmethod
    def set_arm_speed(self, speed, acce, decel) -> bool:
        ...

    @abstractmethod
    def get_current_arm_speed(self) -> tuple | None:
        ...

    @abstractmethod
    def emergency_stop(self) -> bool:
        ...

    @abstractmethod
    def clear_faults(self) -> bool:
        ...

