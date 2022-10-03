#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter3', String, queue_size=10)
    rospy.init_node('talker3', anonymous=False)
    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        hello_str = "hello world 3 %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except KeyboardInterrupt:
        pass

