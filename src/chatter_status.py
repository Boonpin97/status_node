#!/usr/bin/env python
import rospy
from azimorph_msg.msg import Status, Topic

def talker():
    T = Topic()
    T.name = "chatter1"
    T.freq = 10
    msg = Status()
    msg.Topics.append(T)
    # msg.Topics[0].name = "chatter1"
    # msg.Topics[0].freq = 10
    # msg.Topics[0].status = 1
    # msg.Topics[1].name = "chatter2"
    # msg.Topics[1].freq = 20
    # msg.Topics[1].status = 1
    # msg.Nodes[0].name = "talker1"
    # msg.Nodes[0].status = 1
    # msg.Nodes[1].name = "talker2"
    # msg.Nodes[1].status = 1

    pub = rospy.Publisher('chatter1', Status, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hzs
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except KeyboardInterrupt:
        pass