import sys
import rospkg

rospack = rospkg.RosPack()
modpath = rospack.get_path('semantic_web') + '/src'
sys.path.insert(0, modpath)
