import os
from cv2 import aruco

from droid.misc.station_env import load_station_env

# Robot Params #
# Per-machine values (IPs, robot serial, the NUC's sudo password) live OUTSIDE
# the checkout: ~/.config/droid/station.env or DROID_* env vars, see
# droid/misc/station_env.py. Never edit them here -- this file stays identical
# to git on every machine. Blank = upstream default.
_station = load_station_env()
nuc_ip = _station["DROID_NUC_IP"]
robot_ip = _station["DROID_ROBOT_IP"]
laptop_ip = _station["DROID_LAPTOP_IP"]
sudo_password = _station["DROID_SUDO_PASSWORD"]
robot_type = _station["DROID_ROBOT_TYPE"]  # 'panda' or 'fr3'
robot_serial_number = _station["DROID_ROBOT_SERIAL_NUMBER"]

# Camera ID's #
hand_camera_id = _station["DROID_HAND_CAMERA_ID"]
varied_camera_1_id = _station["DROID_VARIED_CAMERA_1_ID"]
varied_camera_2_id = _station["DROID_VARIED_CAMERA_2_ID"]

# Charuco Board Params #
CHARUCOBOARD_ROWCOUNT = 9
CHARUCOBOARD_COLCOUNT = 14
CHARUCOBOARD_CHECKER_SIZE = 0.020
CHARUCOBOARD_MARKER_SIZE = 0.016
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_100)

# Ubuntu Pro Token (RT PATCH) #
ubuntu_pro_token = _station["DROID_UBUNTU_PRO_TOKEN"]

# Code Version [DONT CHANGE] #
droid_version = "1.3"

