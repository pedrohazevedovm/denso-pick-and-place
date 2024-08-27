from robot.robot_api.denso_abstract import AbstractDenso
from robot.manipulator_robot import DensoControl

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
    robot = DensoControl(f"celular_{tag}", f"robot_{tag}", "Server=192.168.160.226")
    robot.connect()

    get_positions(robot)

    robot.disconnect()