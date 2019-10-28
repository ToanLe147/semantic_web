#!/usr/bin/python
import sys
import rospkg

rospack = rospkg.RosPack()
modpath = rospack.get_path('semantic_web') + '/src'
sys.path.insert(0, modpath)
sys.path.insert(1, "/home/nico/.local/lib/python2.7/site-packages")
sys.path.insert(2, "/home/nico/.local/lib/python2.7/site-packages")
# print (sys.path)
from FlaskApp import app as application
application.secret_key = "ashdfgvalsdjkfyv"
