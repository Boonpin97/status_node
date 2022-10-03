#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter2', String, queue_size=10)
    rospy.init_node('talker2', anonymous=False)
    rate = rospy.Rate(24) # 20hz
    while not rospy.is_shutdown():
        hello_str = "hello world 2 %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except KeyboardInterrupt:
        pass

