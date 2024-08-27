# Import control over the gripper
from rria_api_denso import (GRIPPER_ETHERNET_IP, GRIPPER_SOCKET_PORT,
                            DensoRobotAPI, GripperCommSocket, GripperResponses, RobotJointCommand, RobotCartesianCommand)
# Import abstract robot class
from robot.robot_api.denso_abstract import AbstractDenso
from robot.bank_movements import get_pose


# Implement real Denso robot from abstract robot
class DensoControl(AbstractDenso):
    def __init__(self, workspace_name, control_name, options):
        self.__denso_api = DensoRobotAPI(workspace_name, control_name, options) # Instantiate the robot
        self.__gripper = GripperCommSocket(GRIPPER_ETHERNET_IP, GRIPPER_SOCKET_PORT) # Instantiate the gripper

    # Check connection
    def is_connected(self) -> bool:
        return self.__denso_api.is_connected()

    # Check whether the motor is turned on
    def motor_enabled(self) -> bool:
        return self.__denso_api.motor_enabled()

    # Setup CAO workspace and controller and connect the robot
    def connect(self) -> bool:
        try:
            return self.__denso_api.connect()
        except Exception as e:
            print(e)
            return False

    # Delete CAO workspace and controller and disconnect the gripper and the robot
    def disconnect(self) -> bool:
        try:
            if self.__gripper.is_connect():
                self.__gripper.disconnect()

            return self.__denso_api.disconnect()
        except Exception as e:
            print(e)
            return False

    # Take control of the robotic arm and turn on the motor
    def motor_on(self) -> bool:
        try:
            return self.__denso_api.motor_on()
        except Exception as e:
            print(e)
            return False

    # Turn off the motor and give off control of the robotic arm
    def motor_off(self) -> bool:
        try:
            return self.__denso_api.motor_off()
        except Exception as e:
            print(e)
            return False

    # Move robot joints according to command
    def move_joints(self, command : RobotJointCommand) -> bool:
        try:
            return self.__denso_api.move_joints(command)
        except Exception as e:
            print(e)
            return False

    # Move robot in cartesian manner according to command
    def move_cartesian(self, command : RobotCartesianCommand) -> bool:
        try:
            return self.__denso_api.move_cartesian(command)
        except Exception as e:
            print(e)
            return False
    
    # Move robot to a preset joints pose
    def move_preset_joints(self, pose : str) -> bool:
        if self.is_connected() and self.motor_enabled():
            joints, _ = get_pose(pose, 0)
            if joints != None:
                command = RobotJointCommand.from_list(joints)
                self.move_joints(command)
                return True
        elif not self.motor_enabled():
            print('\nRobot motor is off')
            return False
        elif not self.is_connected():
            print('\nRobot not connected')
            return False
        
    # Move robot to a preset cartesian pose
    def move_preset_cartesian(self, pose : str) -> bool:
        if self.is_connected() and self.motor_enabled():
            coordinates, _ = get_pose(pose, 1)
            if coordinates != None:
                command = RobotCartesianCommand.from_list(coordinates)
                self.move_cartesian(command)
                return True
        elif not self.motor_enabled():
            print('\nRobot motor is off')
            return False
        elif not self.is_connected():
            print('\nRobot not connected')
            return False

    # Move end effector forward or backward according to step size
    def move_tool_z(self, z_step: float) -> bool:
        try:
            return self.__denso_api.move_tool_z(z_step)
        except Exception as e:
            print(e)
            return False

    # Return current joints configuration
    def get_joints_pose(self):
        try:
            return self.__denso_api.get_joints_pose()
        except Exception as e:
            print(e)

    # Return current cartesian configuration
    def get_cartesian_pose(self):
        try:
            return self.__denso_api.get_cartesian_pose()
        except Exception as e:
            print(e)

    # Setup speed, acceleration and deceleration for the robotic arm
    def set_arm_speed(self, speed, accel, decel) -> bool:
        try:
            return self.__denso_api.set_arm_speed(speed, accel, decel)
        except Exception as e:
            print(e)
            return False

    # Return current speed, acceleration and deceleration of the robotic arm
    def get_current_arm_speed(self) -> tuple | None:
        try:
            return self.__denso_api.get_current_arm_speed()
        except Exception as e:
            print(e)

    # Return current raised error
    def get_current_error(self):
        try:
            return self.__denso_api.get_current_error()
        except Exception as e:
            print(e)
            return False

    # Return current cartesian figure
    def get_current_fig(self) -> int | None:
        try:
            return self.__denso_api.get_current_fig()
        except Exception as e:
            print(e)

    # Check whether the arm is whithin workspace
    def is_out_of_range(self, robot_command) -> bool | None:
        try:
            return self.__denso_api.is_out_of_range(robot_command)
        except Exception as e:
            print(e)

    # Convert joint to cartesian pose
    def joint_to_cartesian(self, command):
        try:
            return self.__denso_api.joint_to_cartesian(command)
        except Exception as e:
            print(e)

    # Convert cartesian to joint pose
    def cartesian_to_joint(self, command):
        try:
            return self.__denso_api.cartesian_to_joint(command)
        except Exception as e:
            print(e)

    # Return distance between current pose and target pose
    def calculate_dist_to(self, position):
        return self.__denso_api.dist_to(position)

    # Specify number of steps for the gripper's motor
    def move_gripper_to(self, value):
        try:
            if self.__gripper.gripper_status() != GripperResponses.GRIPPER_OK:
                self.__gripper.reset_motor()

            return self.__gripper.move_gripper_to(value)
        except Exception as e:
            print(e)

    # Totally open the gripper
    def open_gripper_full(self):
        try:
            if self.__gripper.gripper_status() != GripperResponses.GRIPPER_OK:
                self.__gripper.reset_motor()

            open = self.__gripper.move_gripper_to(self.__gripper.MAX_POSITION_ANGLE)
            return open
        except Exception as e:
            print(e)

    # Totally close the gripper
    def close_gripper_full(self):
        try:
            if self.__gripper.gripper_status() != GripperResponses.GRIPPER_OK:
                self.__gripper.reset_motor()

            return self.__gripper.move_gripper_to(self.__gripper.MIN_POSITION_ANGLE)
        except Exception as e:
            print(e)

    # Return whether the gripper is functioning correctly
    def gripper_status(self) -> GripperResponses:
        return self.__gripper.gripper_status()

    # Return current number of steps of the gripper's motor
    def current_gripper_angle(self) -> int | None:
        return self.__gripper.gripper_position()

    # Connect the gripper
    def connect_gripper(self) -> bool:
        if self.__gripper.is_connect():
            return True
        return self.__gripper.connect()

    # Disconnect the gripper
    def disconnect_gripper(self) -> bool:
        if not (self.__gripper.is_connect()):
            return True
        return self.__gripper.disconnect()

    # Check connection of the gripper
    def gripper_connected(self) -> bool:
        return self.__gripper.is_connect()

    # Return minimum step of the gripper's motor
    def gripper_min_angle(self) -> int:
        return self.__gripper.MIN_POSITION_ANGLE

    # Return maximum step of the gripper's motor
    def gripper_max_angle(self) -> int:
        return self.__gripper.MAX_POSITION_ANGLE