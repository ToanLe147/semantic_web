"""Path hack to make tests work."""
import sys

modpath = '/home/led/catkin_ws/src/robot_vision/src/models/research'
sys.path.insert(0, modpath)

modpath2 = '/home/led/catkin_ws/src/robot_vision/src'
sys.path.insert(1, modpath2)
