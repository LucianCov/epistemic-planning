#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA

class agent():
    def __init__(self, pose_pub, color_pub):
        self.pose = Pose()
        self.pose_pub = pose_pub
        self.color_pub = color_pub

    def update_position(self, x, y, z):
        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.position.z = z

    def update_orientation(self, x, y, z, w):
        self.pose.orientation.x = x
        self.pose.orientation.y = y
        self.pose.orientation.z = z
        self.pose.orientation.w = w
    
    def update_pose(self, pose):
        self.pose = pose

    def publish_pose(self):
        self.pose_pub.publish(self.pose)

    def publish_color(self, color):
        self.color_pub.publish(color)

    @staticmethod
    def get_interpolation(pose1, pose2, num_steps):
        '''
        Returns a list of poses that interpolates the two inputs
        '''
        result = []
        for i in range(num_steps):
            between = Pose()
            scale = float(i) / num_steps
            between.position.x = pose1.position.x + (scale * (pose2.position.x - pose1.position.x))
            between.position.y = pose1.position.y + (scale * (pose2.position.y - pose1.position.y))
            between.position.z = pose1.position.z + (scale * (pose2.position.z - pose1.position.z))

            between.orientation.x = pose1.orientation.x + (scale * pose2.orientation.x)
            between.orientation.y = pose1.orientation.y + (scale * pose2.orientation.y)
            between.orientation.z = pose1.orientation.z + (scale * pose2.orientation.z)
            between.orientation.w = pose1.orientation.w + (scale * pose2.orientation.w)

            result.append(between)
        return result
    
    def move(self, pose, total_time, num_steps):
        points = agent.get_interpolation(self.pose, pose, num_steps)
        timestep = total_time / float(num_steps)

        for point in points:
            self.update_pose(point)
            self.publish_pose()
            rospy.sleep(timestep)
        self.update_pose(pose)
        self.publish_pose()

# Create agents
rospy.init_node("coordinator")
agent1 = agent(
        rospy.Publisher("agent_1/pose", Pose, queue_size=2),
        rospy.Publisher("agent_1/color", ColorRGBA, queue_size=2)
        )
agent2 = agent(
        rospy.Publisher("agent_2/pose", Pose, queue_size=2),
        rospy.Publisher("agent_2/color", ColorRGBA, queue_size=2)
        )

room1 = Pose(Point(-3.5,1,0), Quaternion(0,0,0,0))
room1_enter = Pose(Point(-3.5,-1.5,0), Quaternion(0,0,0,0))
room2 = Pose(Point(-1,1,0), Quaternion(0,0,0,0))
room2_enter = Pose(Point(-1,-1.5,0), Quaternion(0,0,0,0))
room3 = Pose(Point(1.5,1,0), Quaternion(0,0,0,0))
room3_enter = Pose(Point(1.5,-1.5,0), Quaternion(0,0,0,0))
room4 = Pose(Point(4,1,0), Quaternion(0,0,0,0))
room4_enter = Pose(Point(4,-1.5,0), Quaternion(0,0,0,0))


#agent1.update_position(0, 0, 0)
#agent1.update_orientation(0,0,0,0)
#agent2.update_position(1, 2, 0)
#agent2.update_orientation(0,0,0,0)

while not rospy.is_shutdown():
    rospy.rostime.wallsleep(2.0)

    agent1.publish_color(ColorRGBA(1,0,0,1))
    agent1.update_pose(room1)
    agent1.publish_pose()
    agent2.publish_color(ColorRGBA(0,1,0,1))
    agent2.update_pose(Pose(Point(room2.position.x+0.25, room2.position.y, room2.position.z), room2.orientation))
    agent2.publish_pose()

    agent1.move(room1_enter, 2, 60*2)
    agent1.move(room2_enter, 2, 60*2)
    agent1.move(room2, 2, 60*2)
    agent2.publish_color(ColorRGBA(1,0,0,1))

