#!/usr/bin/env python

from distutils.log import info
from platform import node
import rospy, rostopic, rosnode, rospkg
import yaml
import sys
import threading
from std_msgs.msg import String
from azimorph_msg.msg import Status, Topic, Node


message_type = {
	"String": String
}

class simple_class:

	def __init__(self):
		r = rospkg.RosPack()
		yamlFileDir = r.get_path('status_node') + '/src/params.yaml'
		yaml_file = open(yamlFileDir,"r")
		self.yaml_dict = yaml.safe_load(yaml_file)
		self.timeout = self.yaml_dict["Params"]["timeout"]
		self.window_size = self.yaml_dict["Params"]["window_size"]
		self.topics_dict = self.yaml_dict["Topics"]
		self.nodes_dict = self.yaml_dict["Nodes"]
		self.r = rostopic.ROSTopicHz(self.window_size)
		self.pub = rospy.Publisher('status', Status, queue_size=10)
		for topic, info in self.topics_dict.items():
			rospy.Subscriber(topic, message_type[info["type"]], self.callBack, callback_args = topic)
			info["timer"] = threading.Timer(self.timeout, self.timeoutFunc, [topic])
			info["status"] = 0 
			info["freq"] = 0
			info["timer"].start()

		while not rospy.is_shutdown():
			self.main_loop()
			rospy.Rate(10).sleep()	
		

	def main_loop(self):
		self.checkAllTopicStatus()
		self.checkAllNodeStatus()
		self.publishTopicsAndNodes()


	def publishTopicsAndNodes(self):
		msg = Status()
		for topic, info in self.topics_dict.items():
			t = Topic()
			t.name = topic
			t.status = info["status"]
			t.freq = info["freq"]
			msg.Topics.append(t)
		for node, info in self.nodes_dict.items():
			n = Node()
			n.name = node
			n.status = info["status"]
			msg.Nodes.append(n)
		self.pub.publish(msg)

	def callBack(self, data, topic):
		self.r.callback_hz(data, topic)
		freq = self.r.get_hz(topic)
		if freq != None:
			self.topics_dict[topic]["freq"] = round(freq[0])
			self.topics_dict[topic]["timer"].cancel()
			self.topics_dict[topic]["timer"] = threading.Timer(self.timeout, self.timeoutFunc, [topic])
			self.topics_dict[topic]["timer"].start()
		else:
			pass
			rospy.logwarn("\nCannot get frequency from topic:" + topic)
		if self.topics_dict[topic]["status"] == 0:
			rospy.loginfo("Topic:" + topic + " is alive!")
			self.topics_dict[topic]["status"] = 1
		deviation = (self.topics_dict[topic]["freq"] - self.topics_dict[topic]["target_freq"]) / self.topics_dict[topic]["target_freq"]
		max_deviation = self.topics_dict[topic]["max_deviation"]
		if deviation < -max_deviation:
			if self.topics_dict[topic]["status"] != 2:
				rospy.logwarn("Topic: " + topic + " 's frequency is slower than usual..")
			self.topics_dict[topic]["status"] = 2
		elif deviation > max_deviation:
			if self.topics_dict[topic]["status"] != 3:
				rospy.logwarn("Topic: " + topic + " 's frequency is higher than usual..")
			self.topics_dict[topic]["status"] = 3
		else:
			if self.topics_dict[topic]["status"] != 1:
				rospy.logwarn("Topic: " + topic + " 's frequency is back to normal!")
			self.topics_dict[topic]["status"] = 1

	def timeoutFunc(self, topic):
		rospy.logerr("No message received from " + str(topic) + " for the past " + str(self.timeout) + " second(s)")
		self.topics_dict[str(topic)]["status"] = 0

	def checkAllTopicStatus(self):
		message = "\nCurrent information of all topics of interest:"
		for topic, info in self.topics_dict.items():
			if info["status"] == 0:
				status = "Dead"
			elif info["status"] == 1:
				status = "Alive"
			elif info["status"] == 2:
				status = "Slow Freq"
			elif info["status"] == 3:
				status = "Fast Freq"
			else:
				status = "Unknown"
			freq = info["freq"]
			message += str("\nTopic:{0:30} Status:{1:10}  Frequency:{2:3} ".format(topic, status, freq))
		rospy.loginfo(message)

	def checkAllNodeStatus(self):
		message = "\nCurrent information of all nodes of interest:"
		current_node_ls = rosnode.get_node_names()
		for node, info in self.nodes_dict.items():
			if node in current_node_ls:
				info["status"] = 1
				status = "Alive"
			else:
				info["status"] = 0
				status = "Dead"
			message += str("\nNode:{0:31} Status:{1:1} ".format(node, status))
		rospy.loginfo(message)



def main(args):
	rospy.init_node('simple_class', anonymous=False)
	obc = simple_class()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.logwarn("Shutting down 2")

if __name__ == '__main__':
    main(sys.argv)
