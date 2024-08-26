from robot_api.denso_abstract import AbstractDenso
from manipulator_robot import ManipulatorRobot

def get_positions(robot: AbstractDenso):
    status = False
    while not status:
        input_rec = input("Digite 1 para gravar posição e o nome da posição ou 2 para sair: ").split()
        if input_rec[0] == "1":
            pos_cartesian = robot.get_cartesian_pose()
            pos_joint = robot.get_joints_pose()
            with open("positions.txt", 'a') as pos:
                pos.write(f'pos_cartesian {input_rec[1]}: {pos_cartesian}\n'
                          f'pos_joint {input_rec[1]}: {pos_joint}\n')
        else:
            status = True


if __name__ == '__main__':
    tag = 'blah blah'
    robot = ManipulatorRobot(tag, 1)
    robot.take_control()

    get_positions(AbstractDenso)

    robot.give_control()