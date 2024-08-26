# Bank of movements for Denso robot
BANK_MOVEMENT = {
    'zero': ([], [], "Robot moved to zero position"),
    'home': ([], [], "Robot moved to 'home' position"),

    'pre_grasp': ([], [], "Robot moved to 'pre grasp' position"),

    'grasp': ([], [], "Robot moved to grasp position"),

    'flash': ([], [], "Robot is ready to take a picture")
}


"""
Return list of joint positions (if indx == 0) or list of cartesian coordinates (if indx == 1) from chosen bm = BANK_MOVEMENT (1 or 2)
and a message for the test robot
"""
def get_pose(key : str, indx : int):
    if key in BANK_MOVEMENT.keys():
        return BANK_MOVEMENT[key][indx], BANK_MOVEMENT[key][2]
    else:
        print('Error trying to select pose')