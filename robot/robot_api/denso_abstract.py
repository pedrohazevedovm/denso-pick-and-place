from abc import ABC, abstractmethod

from rria_api_denso import RobotCartesianCommand, RobotJointCommand

# Abstract robot class
class AbstractDenso(ABC):
    # Check connection
    @abstractmethod
    def is_connected(self) -> bool: ...

    # Check whether the motor is turned on
    @abstractmethod
    def motor_enabled(self) -> bool: ...

    # Connect the robot
    @abstractmethod
    def connect(self) -> bool: ...

    # Disconnect the robot
    @abstractmethod
    def disconnect(self) -> bool: ...

    # Turn on the motor
    @abstractmethod
    def motor_on(self) -> bool: ...

    # Turn off the motor
    @abstractmethod
    def motor_off(self) -> bool: ...

    # Move robot joints according to command
    @abstractmethod
    def move_joints(self, command: RobotJointCommand) -> bool: ...

    # Move robot in cartesian manner according to command
    @abstractmethod
    def move_cartesian(self, command: RobotCartesianCommand) -> bool: ...

    # Return current joints configuration
    @abstractmethod
    def get_joints_pose(self) -> RobotJointCommand | None: ...

    # Return current cartesian configuration
    @abstractmethod
    def get_cartesian_pose(self) -> RobotCartesianCommand | None: ...

    # Setup speed, acceleration and deceleration for the robotic arm
    @abstractmethod
    def set_arm_speed(self, speed, accel, decel) -> bool: ...

    # Return current speed, acceleration and deceleration of the robotic arm
    @abstractmethod
    def get_current_arm_speed(self) -> tuple | None: ...

    # Return current raised error
    @abstractmethod
    def get_current_error(self): ...

    # Return current cartesian figure
    @abstractmethod
    def get_current_fig(self) -> int | None: ...

    # Check whether the arm is whithin workspace
    @abstractmethod
    def is_out_of_range(self, robot_command) -> bool | None: ...

    # Convert joint to cartesian pose
    @abstractmethod
    def joint_to_cartesian(self, command: RobotJointCommand) -> RobotCartesianCommand | None: ...

    # Convert cartesian to joint pose
    @abstractmethod
    def cartesian_to_joint(self, command: RobotCartesianCommand) -> RobotJointCommand | None: ...

    # Move end effector forward or backward according to step size
    @abstractmethod
    def move_tool_z(self, z_step: float) -> bool: ...

    # Return distance between current pose and target pose
    @abstractmethod
    def calculate_dist_to(self, position): ...

    # Specify number of steps for the gripper's motor
    @abstractmethod
    def move_gripper_to(self, value): ...

    # Totally open the gripper
    @abstractmethod
    def open_gripper_full(self): ...

    # Totally close the gripper
    @abstractmethod
    def close_gripper_full(self): ...

    # Return whether the gripper is functioning correctly
    @abstractmethod
    def gripper_status(self): ...

    # Return current number of steps of the gripper's motor
    @abstractmethod
    def current_gripper_angle(self): ...

    # Connect the gripper
    @abstractmethod
    def connect_gripper(self): ...

    # Disconnect the gripper
    @abstractmethod
    def disconnect_gripper(self): ...

    # Check connection of the gripper
    @abstractmethod
    def gripper_connected(self): ...

    # Return minimum step of the gripper's motor
    @abstractmethod
    def gripper_min_angle(self): ...

    # Return maximum step of the gripper's motor
    @abstractmethod
    def gripper_max_angle(self): ...