import os
from cv2 import aruco

# Robot Params #
# Station values come from the environment (blank = upstream default) so this
# file needs no per-machine edits and the checkout stays byte-identical to git.
# SIR sets them from sir/real/station.py on `import sir.real` (see
# sir/real/droid_compat.py); a standalone DROID install exports DROID_* itself.
nuc_ip = os.environ.get("DROID_NUC_IP", "")
robot_ip = os.environ.get("DROID_ROBOT_IP", "")
laptop_ip = os.environ.get("DROID_LAPTOP_IP", "")
sudo_password = ""
robot_type = os.environ.get("DROID_ROBOT_TYPE", "")  # 'panda' or 'fr3'
robot_serial_number = os.environ.get("DROID_ROBOT_SERIAL_NUMBER", "")

# Camera ID's #
hand_camera_id = ""
varied_camera_1_id = ""
varied_camera_2_id = ""

# Charuco Board Params #
CHARUCOBOARD_ROWCOUNT = 9
CHARUCOBOARD_COLCOUNT = 14
CHARUCOBOARD_CHECKER_SIZE = 0.020
CHARUCOBOARD_MARKER_SIZE = 0.016
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_100)

# Ubuntu Pro Token (RT PATCH) #
ubuntu_pro_token = ""

# Code Version [DONT CHANGE] #
droid_version = "1.3"

