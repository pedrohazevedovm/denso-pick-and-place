from time import sleep

# Import commands for the gripper and the robot
from rria_api_denso import (GripperResponses, RobotCartesianCommand,
                            RobotJointCommand)
# Import abstract robot class
from robot.robot_api.denso_abstract import AbstractDenso
from robot.bank_movements import get_pose

# Implement test Denso robot from abstract robot
class DensoTest(AbstractDenso):
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
        if self.__connected:
            if not self.__motor_enabled:
                self.__motor_enabled = True
                print('\nRobot motor was successfully turned on')
            else:
                print('\nRobot motor is already on')
            return True
        else:
            print('\nRobot not connected')
            return False

    # Turn off the motor
    def motor_off(self) -> bool:
        sleep(2)
        if self.__connected:
            if self.__motor_enabled:
                self.__motor_enabled = False
                print('\nRobot motor was successfully turned off')
            else:
                print('\nRobot motor is already off')
            return True
        else:
            print('\nRobot not connected')
            return False

    # Move robot joints according to command
    def move_joints(self, command) -> bool:
        sleep(2)
        if self.__connected and self.__motor_enabled:
            print(
                "moving joints to:",
                [command.joint_1, command.joint_2, command.joint_3, command.joint_4, command.joint_5, command.joint_6],
            )
            return True
        elif not self.__motor_enabled:
            print('\nRobot motor is off')
            return False
        elif not self.__connected:
            print('\nRobot not connected')
            return False

    # Move robot in cartesian manner according to command
    def move_cartesian(self, command) -> bool:
        sleep(2)
        if self.__connected and self.__motor_enabled:
            print(
                "moving cartesian to:", [command.x, command.y, command.z, command.rx, command.ry, command.rz, command.fig]
            )
            return True
        elif not self.__motor_enabled:
            print('\nRobot motor is off')
            return False
        elif not self.__connected:
            print('\nRobot not connected')
            return False

    # Move robot to a preset joints pose
    def move_preset_joints(self, pose : str) -> bool:
        if self.__connected and self.__motor_enabled:
            _, msg = get_pose(pose, 0)
            sleep(1)
            print('\n' + msg + 'by joints move')
            return True
        elif not self.motor_enabled():
            print('\nRobot motor is off')
            return False
        elif not self.is_connected():
            print('\nRobot not connected')
            return False
        
    # Move robot to a preset cartesian pose
    def move_preset_cartesian(self, pose : str) -> bool:
        if self.__connected and self.__motor_enabled:
            _, msg = get_pose(pose, 1)
            sleep(1)
            print('\n' + msg + 'by cartesian move')
        elif not self.motor_enabled():
            print('\nRobot motor is off')
            return False
        elif not self.is_connected():
            print('\nRobot not connected')
            return False

    # Return current joints configuration
    def get_joints_pose(self):
        sleep(1)
        if self.__connected:
            return RobotJointCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            print('\nRobot not connected')
            return None

    # Return current cartesian configuration
    def get_cartesian_pose(self):
        sleep(1)
        if self.__connected:
            return RobotCartesianCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)
        else:
            print('\nRobot not connected')
            return None

    # Setup speed, acceleration and deceleration for the robotic arm
    def set_arm_speed(self, speed, accel, decel) -> bool:
        sleep(1)
        if self.__connected:
            print("set arm speed to:", [speed, accel, decel])
            return True
        else:
            print('\nRobot not connected')
            return False

    # Return current speed, acceleration and deceleration of the robotic arm
    def get_current_arm_speed(self) -> tuple | None:
        sleep(1)
        if self.__connected:
            return (10, 10, 10)
        else:
            print('\nRobot not connected')
            return None

    # Return current raised error
    def get_current_error(self):
        if self.__connected:
            return (1, "", "")
        else:
            print('\nRobot not connected')
            return None

    # Return current cartesian figure
    def get_current_fig(self) -> int | None:
        if self.__connected:
            return 1
        else:
            print('\nRobot not connected')
            return None

    # Check whether the arm is whithin workspace
    def is_out_of_range(self, robot_command) -> bool | None:
        sleep(1)
        if not self.__connected:
            print('\nRobot not connected')
        return False

    # Convert joint to cartesian pose
    def joint_to_cartesian(self, command):
        if self.__connected:
            return RobotCartesianCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)
        else:
            print('\nRobot not connected')
            return None

    # Convert cartesian to joint pose
    def cartesian_to_joint(self, command):
        if self.__connected:
            return RobotJointCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            print('\nRobot not connected')
            return None

    # Move end effector forward or backward according to step size
    def move_tool_z(self, z_step: float) -> bool:
        sleep(2)
        if self.__connected and self.__motor_enabled:
            print(f"move z: {z_step}")
            return True
        elif not self.__connected:
            print('\nRobot not connected')
            return False
        elif not self.__motor_enabled:
            print('\nRobot motor is off')
            return False

    # Return distance between current pose and target pose
    def calculate_dist_to(self, position):
        if self.__connected:
            return 1
        else:
            print('\nRobot not connected')
            return None

    # Specify number of steps for the gripper's motor
    def move_gripper_to(self, value):
        sleep(1)
        if self.__gripper_is_connect:
            print(f'\nGripper is moving by {value} steps')
            return True
        else:
            print('\nGripper not connected')
            return False

    # Totally open the gripper
    def open_gripper_full(self):
        sleep(1)
        if self.__gripper_is_connect:
            print('\nGripper was totally opened')
            return True
        else:
            print('\nGripper not connected')
            return False

    # Totally close the gripper
    def close_gripper_full(self):
        sleep(1)
        if self.__gripper_is_connect:
            print('\nGripper was totally closed')
            return True
        else:
            print('\nGripper not connected')
            return False

    # Return whether the gripper is functioning correctly
    def gripper_status(self) -> GripperResponses:
        if self.__gripper_is_connect:
            return GripperResponses.GRIPPER_OK
        else:
            print('\nGripper not connected')
            return None

    # Return current number of steps of the gripper's motor
    def current_gripper_angle(self) -> int | None:
        if self.__gripper_is_connect:
            return 500
        else:
            print('\nGripper not connected')
            return None

    # Connect the gripper
    def connect_gripper(self) -> bool:
        sleep(1)
        if not self.__gripper_is_connect:
            self.__gripper_is_connect = True
        else:
            print('\nGripper is already connected')
        return True

    # Disconnect the gripper
    def disconnect_gripper(self) -> bool:
        sleep(1)
        if self.__gripper_is_connect:
            self.__gripper_is_connect = False
        else:
            print('\nGripper is already disconnected')
        return True

    # Check connection of the gripper
    def gripper_connected(self) -> bool:
        sleep(1)
        if self.__gripper_is_connect:
            return True
        else:
            return False

    # Return minimum step of the gripper's motor
    def gripper_min_angle(self):
        if self.__gripper_is_connect:
            return 100
        else:
            print('\nGripper not connected')
            return None

    # Return maximum step of the gripper's motor
    def gripper_max_angle(self):
        if self.__gripper_is_connect:
            return 500
        else:
            print('\nGripper not connected')
            return False