from time import sleep

# Import commands for the gripper and the robot
from rria_api_denso import (GripperResponses, RobotCartesianCommand,
                            RobotJointCommand)
# Import abstract robot class
from robot_api.abstract_robot_api import AbstractRobot


# Implement test Denso robot from abstract robot
class TestRobotAPI(AbstractRobot):
    def __init__(self):
        self.__motor_enabled = False
        self.__connected = False
        self.__gripper_is_connect = False

    # Check connection
    def is_connected(self) -> bool:
        sleep(1)
        return self.__connected

    # Check whether the motor is turned on
    def motor_enabled(self) -> bool:
        sleep(1)
        return self.__motor_enabled

    # Connect the robot
    def connect(self) -> bool:
        sleep(1)
        if not self.__connected:
            self.__connected =  True
            print('\nRobot successfully connected')
        else:
            print('\nRobot already connected')
        return True

    # Disconnect the robot
    def disconnect(self) -> bool:
        sleep(2)
        if self.__connected:
            self.__connected = False
            print('\nRobot successfully disconnected')
        else:
            print('\nRobot already disconnected')
        return True

    # Turn on the motor
    def motor_on(self) -> bool:
        sleep(2)
        if not self.__motor_enabled:
            self.__motor_enabled = True
            print('\nRobot motor was turned on')
        else:
            print('\nRobot motor is already on')
        return True

    # Turn off the motor
    def motor_off(self) -> bool:
        sleep(2)
        if self.__motor_enabled:
            self.__motor_enabled = False
            print('\nRobot motor was turned off')
        else:
            print('\nRobot motor is already off')
        return True

    # Move robot joints according to command
    def move_joints(self, command) -> bool:
        sleep(2)
        if self.__connected and self.__motor_enabled
        print(
            "moving joints to:",
            [command.joint_1, command.joint_2, command.joint_3, command.joint_4, command.joint_5, command.joint_6],
        )
        return True

    # Move robot in cartesian manner according to command
    def move_cartesian(self, command) -> bool:
        sleep(2)
        print(
            "moving cartesian to:", [command.x, command.y, command.z, command.rx, command.ry, command.rz, command.fig]
        )
        return True

    # Return current joints configuration
    def get_joints_pose(self):
        sleep(1)
        return RobotJointCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    # Return current cartesian configuration
    def get_cartesian_pose(self):
        sleep(1)
        return RobotCartesianCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)

    # Setup speed, acceleration and deceleration for the robotic arm
    def set_arm_speed(self, speed, accel, decel) -> bool:
        sleep(1)
        print("set arm speed to:", [speed, accel, decel])
        return True

    # Return current speed, acceleration and deceleration of the robotic arm
    def get_current_arm_speed(self) -> tuple | None:
        sleep(1)
        return (10, 10, 10)

    # Return current raised error
    def get_current_error(self):
        return (1, "", "")

    # Return current cartesian figure
    def get_current_fig(self) -> int | None:
        return 1

    # Check whether the arm is whithin workspace
    def is_out_of_range(self, robot_command) -> bool | None:
        sleep(1)
        return False

    # Convert joint to cartesian pose
    def joint_to_cartesian(self, command):
        return RobotCartesianCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)

    # Convert cartesian to joint pose
    def cartesian_to_joint(self, command):
        return RobotJointCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    # Move end effector forward or backward according to step size
    def move_tool_z(self, z_step: float) -> bool:
        sleep(2)
        print(f"move z: {z_step}")
        return True

    # Return distance between current pose and target pose
    def calculate_dist_to(self, position):
        return 1

    # Specify number of steps for the gripper's motor
    def move_gripper_to(self, value):
        sleep(1)
        print(f'\nGripper is moving by {value} steps')
        return True

    # Totally open the gripper
    def open_gripper_full(self):
        sleep(1)
        print('\nGripper was totally opened')
        return True

    # Totally close the gripper
    def close_gripper_full(self):
        sleep(1)
        print('\nGripper was totally closed')
        return True

    # Return whether the gripper is functioning correctly
    def gripper_status(self) -> GripperResponses:
        return GripperResponses.GRIPPER_OK

    # Return current number of steps of the gripper's motor
    def current_gripper_angle(self) -> int | None:
        return 500

    # Connect the gripper
    def connect_gripper(self) -> bool:
        if ...
        sleep(1)
        return True

    def disconnect_gripper(self) -> bool:
        sleep(1)
        return True

    def gripper_connected(self) -> bool:
        sleep(1)
        return True

    def gripper_min_angle(self):
        return 100

    def gripper_max_angle(self):
        return 500
