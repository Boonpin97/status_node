#!/usr/bin/env python

from distutils.log import info
import rospy, rostopic, rosnode
import yaml
import sys
import threading
from std_msgs.msg import String

message_type = {
	"String": String
}

class simple_class:

	def __init__(self):
		yaml_file = open("catkin_ws/src/status_node/src/params.yaml","r")
		self.yaml_dict = yaml.safe_load(yaml_file)
		self.timeout = self.yaml_dict["Params"]["timeout"]
		self.window_size = self.yaml_dict["Params"]["window_size"]
		self.topics_dict = self.yaml_dict["Topics"]
		self.nodes_dict = self.yaml_dict["Nodes"]
		self.r = rostopic.ROSTopicHz(self.window_size)
		print(self.nodes_dict)
		for topic, info in self.topics_dict.items():
			rospy.Subscriber(topic, message_type[info["type"]], self.callBack, callback_args = topic)
			info["timer"] = threading.Timer(self.timeout, self.timeoutFunc, [topic])
			info["status"] = 0 
			info["freq"] = 0
			info["timer"].start()

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
		self.checkAllTopicStatus()
		self.checkAllNodeStatus()

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
				status = "Slow Frequency"
			elif info["status"] == 3:
				status = "Fast Frequency"
			else:
				status = "Unknown"
			freq = info["freq"]
			message += str("\nStatus:{1:20} Last Frequency:{2:3}   Topic:{0:30} ".format(topic, status, freq))
		rospy.loginfo(message)

	def checkAllNodeStatus(self):
		current_node_ls = rosnode.get_node_names()
		for node, info in self.nodes_dict.items():
			if node in current_node_ls:
				info["status"] = 1
			else:
				info["status"] = 0
		print("Nodes: ",self.nodes_dict)

def main(args):
	rospy.init_node('simple_class', anonymous=False)
	obc = simple_class()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.logwarn("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
