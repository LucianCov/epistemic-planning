#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

class agent():
    def __init__(self, pub):
        self.pose = Pose()
        self.pub = pub

    def update_position(self, x, y, z):
        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.position.z = z

    def update_orientation(self, x, y, z, w):
        self.pose.orientation.x = x
        self.pose.orientation.y = y
        self.pose.orientation.z = z
        self.pose.orientation.w = w

    def publish_pose(self):
        self.pub.publish(self.pose)

# Create agents
rospy.init_node("coordinator")
agent1 = agent(rospy.Publisher("agent_1/pose", Pose, queue_size=2))
agent2 = agent(rospy.Publisher("agent_2/pose", Pose, queue_size=2))

agent1.update_position(0, 0, 0)
agent1.update_orientation(0,0,0,0)
agent2.update_position(1, 2, 0)
agent2.update_orientation(0,0,0,0)

while not rospy.is_shutdown():
    rospy.rostime.wallsleep(1.0)
    agent1.publish_pose()
    agent2.publish_pose()

