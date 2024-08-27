from rria_api_denso import GripperResponses
from robot.robot_api.denso_robot import DensoControl
from robot.robot_api.denso_test import DensoTest

# Device manipulator robot class
class ManipulatorRobot:
    def __init__(self, tag, robot_type=2):
        if robot_type == 1:
            # Real robot
            self.__api = DensoControl(f"celular_{tag}", f"robot_{tag}", "Server=192.168.160.226")
        else:
            # Test robot
            self.__api = DensoTest(f"celular_{tag}", f"robot_{tag}", "Server=192.168.160.226")

        self.__speed = 20
        self.__acc = 20
        self.__decel = 20

        self.__is_connected = False
        self.__motor_enabled = False
        self.__gripper_connected = False
        self.__ready_to_go = False

    # Setup CAO workspace and controller and connect the robot
    def __start_connection(self):
        self.__api.connect()

    # Disconnect the robot
    def __stop_connection(self):
        if not self.__motor_enabled:
            self.__api.disconnect()

    # Connect the robot, turn on the motor, setup arm speed and connect the gripper
    def take_control(self):
        if not self.__ready_to_go:
            self.__is_connected = self.__start_connection() # Connect robot
            self.__motor_enabled = self.__api.motor_on() # Turn on motor
            speed, acc, decel = self.__speed, self.__acc, self.__decel # Setup arm speed
            self.__api.set_arm_speed(speed, acc, decel)
            self.__gripper_connected = self.connect_gripper() # Connect gripper
            self.__ready_to_go = True # Robot is ready to operate
        else:
            print('\nRobot is ready to operate')

    # Disconnect the gripper, turn off the motor and disconnect the robot
    def give_control(self):
        if self.__ready_to_go:
            self.__motor_enabled = not self.__api.motor_off() # Turn off motor
            self.__gripper_connected = not self.disconnect_gripper() # Disconnect gripper
            self.__is_connected = not self.__stop_connection() # Disconnect robot
            self.__ready_to_go = False # Robot is ready to be dismissed
        else:
            print('\nRobot is ready to be dismissed')
    
    # Return current joints configuration
    def get_cur_joints_position(self):
        return self.__api.get_joints_pose()

    # Return current cartesian configuration and figure
    def get_cur_cart_position(self):
        return self.__api.get_cartesian_pose(), self.__api.get_current_fig()

    # Return distance between current pose and target pose
    def dist_to(self, position):
        return self.__api.calculate_dist_to(position)
    
    # Move the robot to a preset joints pose
    def joint_move(self, pose : str):
        self.__api.move_preset_joints(pose)

    # Move the robot to a preset cartesian pose
    def cartesian_move(self, pose : str):
        self.__api.move_preset_cartesian(pose)

    # Totally open the gripper
    def open_gripper_full(self):
        return self.__api.open_gripper_full()

    # Totally close the gripper
    def close_gripper_full(self):
        return self.__api.close_gripper_full()

    # Open or close gripper according to specifications
    def close_to_grasp(self, device_width):
        angle = int(7.870 * (device_width - 10) - 273.552)
        return self.__api.move_gripper_to(angle)

    # Return whether the gripper is functioning correctly
    def gripper_status(self) -> GripperResponses:
        return self.__api.gripper_status()

    # Return current number of steps of the gripper's motor
    def current_gripper_angle(self) -> int | None:
        return self.__api.current_gripper_angle()

    # Connect the gripper
    def connect_gripper(self) -> bool:
        return self.__api.connect_gripper()

    # Disconnect the gripper
    def disconnect_gripper(self) -> bool:
        return self.__api.disconnect_gripper()

    # Return minimum step of the gripper's motor
    def gripper_min_angle(self):
        return self.__api.gripper_min_angle()

    # Return maximum step of the gripper's motor
    def gripper_max_angle(self):
        return self.__api.gripper_max_angle()