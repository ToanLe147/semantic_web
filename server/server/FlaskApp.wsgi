#!/usr/bin/python
import sys
import logging
logging.basicConfig(stream=sys.stderr)
sys.path.insert(0, "/home/nico/catkin_ws/src/semantic_web/server/server")
# print (sys.path)
from FlaskApp import app as application
application.secret_key = "toanle"
