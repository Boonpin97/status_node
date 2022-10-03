import rospy
from azimorph_msg.msg import Node

def talker():
    msg = Status()
    msg.Topics[0].name = "chatter1"
    pub = rospy.Publisher('chatter1', Status, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except KeyboardInterrupt:
        pass