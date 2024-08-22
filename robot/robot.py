from rria_api_denso import (CAOVariableType, GripperResponses, MoveType,
                            RobotCartesianCommand)
from scipy.spatial.transform import Rotation as R

from packages.proto.pb.events import (Command1D, Command2D, DeviceOrientation,
                                      DeviceOrientationType, ProblemDetails,
                                      ProblemDetailsProblemLevel,
                                      ProblemDetailsProblemType)
from robot.robot_api.denso_robot_interface import DensoRobotInterface
from robot.robot_api.test_robot_api import TestRobotAPI
from utils.robot_device_context import context
from utils.setup_config import setup
from utils.setup_enum import RobotModel


class Robot:
    def __init__(self, tag, local_context=None):
        if RobotModel(setup.get_int("General", "robot_model")) == RobotModel.DENSO:
            self.__api = DensoRobotInterface(f"celular_{tag}", f"robot_{tag}", "Server=192.168.160.226")
        else:
            self.__api = TestRobotAPI(f"celular_{tag}", f"robot_{tag}", "Server=192.168.160.226")

        self.__angular_step = 10
        self.__cartesian_step = 10
        self.__speed = 20
        self.__acc = 20
        self.__decel = 20

        self.__local_context = context

        if not (local_context is None):
            self.__local_context = local_context

        self.__joystick_speed = 20
        self.__joystick_acc = 20
        self.__joystick_decel = 20

        self.MIN_DISTANCE_TOLERANCE_MM = 20

    def start_connection(self):
        comm = self.__api.connect()
        self.__local_context.set_robot_connected(comm)

    def take_control(self, joystick=False):
        if self.__local_context.robot_is_connected():
            auth = self.__api.motor_on()

            speed, acc, decel = self.__speed, self.__acc, self.__decel
            if joystick:
                speed, acc, decel = self.__joystick_speed, self.__joystick_acc, self.__joystick_decel

            auth = auth and self.__api.set_arm_speed(speed, acc, decel)
            self.__local_context.set_robot_authority(auth)
        else:
            self.__local_context.set_robot_authority(False)

    def give_control(self):
        if self.__local_context.robot_is_connected() and self.__local_context.has_robot_authority():
            auth = self.__api.motor_off()
            self.__local_context.set_robot_authority(not auth)

    def stop_connection(self):
        if self.__local_context.has_robot_authority():
            auth = self.__api.motor_off()
            self.__local_context.set_robot_authority(not auth)

        comm = self.__api.disconnect()
        self.__local_context.set_robot_connected(not comm)

    def get_cur_position(self):
        return self.__api.get_joints_pose()

    def move_plan(self, command: Command2D):
        cur_position = self.__api.get_cartesian_pose()

        if context.device_orientation() == DeviceOrientationType.LANDSCAPE_LEFT:
            command.direction_x, command.direction_y = command.direction_y, -command.direction_x
        elif context.device_orientation() == DeviceOrientationType.LANDSCAPE_RIGHT:
            command.direction_x, command.direction_y = -command.direction_y, command.direction_x

        # Create a rotation object from Euler angles specifying axes of rotation
        rot = R.from_euler("xyz", [cur_position.rx, cur_position.ry, cur_position.rz], degrees=True)
        step_offset = rot.apply(
            [self.__cartesian_step * command.direction_y, self.__cartesian_step * command.direction_x, 0]
        )

        new_pos = RobotCartesianCommand(
            cur_position.x - step_offset[0],
            cur_position.y - step_offset[1],
            cur_position.z - step_offset[2],
            cur_position.rx,
            cur_position.ry,
            cur_position.rz,
            cur_position.fig,
        )

        if self.__api.is_out_of_range(new_pos):
            print("move cartesian limits exceeded")
            return ProblemDetails(
                ProblemDetailsProblemType.ROBOT_LIMITS_EXCEEDED,
                ProblemDetailsProblemLevel.WARNING,
                "move cartesian command exceed limits",
                "send other command",
                "",
            )
        else:
            self.__api.move_cartesian(new_pos)

    def move_z(self, command: Command1D):
        if not self.__api.move_tool_z(-self.__cartesian_step * command.direction):
            print("move cartesian limits exceeded")
            return ProblemDetails(
                ProblemDetailsProblemType.ROBOT_LIMITS_EXCEEDED,
                ProblemDetailsProblemLevel.WARNING,
                "move cartesian command exceed limits",
                "send other command",
                "",
            )

    def move_rotate_base(self, command: Command1D):
        cur_position = self.__api.get_joints_pose()
        cur_position.joint_1 = cur_position.joint_1 - self.__angular_step * command.direction

        if self.__api.is_out_of_range(cur_position):
            print("move joints limits exceeded")
            return ProblemDetails(
                ProblemDetailsProblemType.ROBOT_LIMITS_EXCEEDED,
                ProblemDetailsProblemLevel.WARNING,
                "move joints command exceed limits",
                "send other command",
                "",
            )
        else:
            self.__api.move_joints(cur_position)

    def move_rotate_elbow(self, command: Command1D):
        cur_position = self.__api.get_joints_pose()
        cur_position.joint_3 = cur_position.joint_3 - self.__angular_step * command.direction

        if self.__api.is_out_of_range(cur_position):
            print("move joints limits exceeded")
            return ProblemDetails(
                ProblemDetailsProblemType.ROBOT_LIMITS_EXCEEDED,
                ProblemDetailsProblemLevel.WARNING,
                "move joints command exceed limits",
                "send other command",
                "",
            )
        else:
            self.__api.move_joints(cur_position)

    def move_rotate_camera_angle(self, command: DeviceOrientation):
        cur_position = self.__api.get_joints_pose()

        delta = int(command.orientation) - int(self.__local_context.device_orientation())
        inc_j6 = -delta * 90
        cur_position.joint_6 = cur_position.joint_6 + inc_j6

        if self.__api.is_out_of_range(cur_position):
            print("move joints limits exceeded")
            return ProblemDetails(
                ProblemDetailsProblemType.ROBOT_LIMITS_EXCEEDED,
                ProblemDetailsProblemLevel.WARNING,
                "move joints command exceed limits",
                "send other command",
                "",
            )
        else:
            self.__api.move_joints(cur_position)
            self.__local_context.set_device_orientation(command.orientation)

    def move(self, command):
        if self.__api.is_out_of_range(command):
            print("move joints limits exceeded")
        else:
            if command.type == MoveType.JOINT:
                self.__api.move_joints(command)
            else:
                self.__api.move_cartesian(command)

    def move_by_internal_variable(self, cao_type, number):
        return self.__api.move_by_variable(cao_type, number)

    def release(self):
        self.stop_connection()

    def clear_faults(self):
        self.__api.clear_faults()

    def dist_to(self, position):
        return self.__api.calculate_dist_to(position)

    def open_gripper_full(self):
        return self.__api.open_gripper_full()

    def close_gripper_full(self):
        return self.__api.close_gripper_full()

    def close_to_grasp(self, device_width):
        angle = int(7.870 * (device_width - 10) - 273.552)
        return self.__api.move_gripper_to(angle)

    def gripper_status(self) -> GripperResponses:
        return self.__api.gripper_status()

    def current_gripper_angle(self) -> int | None:
        return self.__api.current_gripper_angle()

    def connect_gripper(self) -> bool:
        return self.__api.connect_gripper()

    def disconnect_gripper(self) -> bool:
        return self.__api.disconnect_gripper()

    def gripper_min_angle(self):
        return self.__api.gripper_min_angle()

    def gripper_max_angle(self):
        return self.__api.gripper_max_angle()
