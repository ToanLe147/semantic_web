#!/usr/bin/python
import sys
sys.path.insert(0, "/home/nico/catkin_ws/src/semantic_web/server/server")
sys.path.insert(1, "/home/nico/.local/lib/python2.7/site-packages")
sys.path.insert(2, "/home/nico/.local/lib/python2.7/site-packages")
# print (sys.path)
from FlaskApp import app as application
application.secret_key = "ashdfgvalsdjkfyv"
