#!/usr/bin/env python
# license removed for brevity

# import time
#
# import roslibpy
#
# client = roslibpy.Ros(host='localhost', port=9090)
# client.run()
#
# talker = roslibpy.Topic(client, '/chatter', 'std_msgs/String')
# test = 0
#
# while client.is_connected:
#     test += 1
#     talker.publish(roslibpy.Message({'data': str(test)}))
#     print('Sending message...')
#     time.sleep(1)
#
# talker.unadvertise()
#
# client.terminate()

import rospy
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher('detect_image', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "=> %s" % rospy.get_time()
        hello_str = raw_input("On/Off: ")
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        hello_str = 0


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
