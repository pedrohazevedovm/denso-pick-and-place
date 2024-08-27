from robot.manipulator_robot import ManipulatorRobot
from device.device import Device
from time import sleep

# Test script for pick and place
if __name__ == '__main__':
    tag = 'bla'

    robot = ManipulatorRobot(tag)#, 1)
    device = Device('192.168.155.2:40605', 'moto_g32')
    device.connect()

    # Start pick and place
    robot.take_control()  # Get ready to operate
    robot.joint_move('home')  # Start position
    robot.open_gripper_full()  # Open gripper
    robot.joint_move('pre_grasp')  # Move close to the device
    robot.cartesian_move('grasp')  # Get ready to grasp the device
    robot.close_to_grasp(device.width)  # Close the gripper to grasp the device
    robot.cartesian_move('pre_grasp')  # Retreat above device holder
    robot.joint_move('flash')  # Get ready to take picture

    device.take_picture()
    sleep(1)
    device.save_photo()
    device.clear_gallery()
    device.return_home()
    device.disconnect()

    robot.joint_move('pre_grasp')  # Get ready to lay down the device
    robot.cartesian_move('grasp')  # Put device on the holder
    robot.open_gripper_full()  # Open gripper to release device
    robot.cartesian_move('pre_grasp')  # Retreat above device holder
    robot.joint_move('home')  # Retreat to initial position
    robot.give_control()  # Task finished