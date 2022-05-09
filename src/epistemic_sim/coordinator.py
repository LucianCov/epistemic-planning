#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA, Bool

import threading, time

class agent():
    def __init__(self, pose_pub, color_pub):
        self.pose = Pose()
        self.pose_pub = pose_pub
        self.color_pub = color_pub

    def update_pose(self, pose):

        self.pose.position.x = pose.position.x
        self.pose.position.y = pose.position.y
        self.pose.position.z = pose.position.z

        self.pose.orientation.x = pose.orientation.x
        self.pose.orientation.y = pose.orientation.y
        self.pose.orientation.z = pose.orientation.z
        self.pose.orientation.w = pose.orientation.w
    
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
agent3 = agent(
        rospy.Publisher("agent_3/pose", Pose, queue_size=2),
        rospy.Publisher("agent_3/color", ColorRGBA, queue_size=2)
        )
agent4 = agent(
        rospy.Publisher("agent_4/pose", Pose, queue_size=2),
        rospy.Publisher("agent_4/color", ColorRGBA, queue_size=2)
        )

cake_pub = rospy.Publisher("agent_1/cake", Bool, queue_size=2)
decorations_pub = rospy.Publisher("agent_3/decoration", Bool, queue_size=2) 

off = 0.5

house = Pose(Point(4,-1.5,0),Quaternion(0,0,0,0))
house1 = Pose(Point(4+off,-1.5+off,0),Quaternion(0,0,0,0))
house2 = Pose(Point(4+off,-1.5-off,0),Quaternion(0,0,0,0))
house3 = Pose(Point(4-off,-1.5-off,0),Quaternion(0,0,0,0))
house4 = Pose(Point(4-off,-1.5+off,0),Quaternion(0,0,0,0))
house_entrance = Pose(Point(4,0.5,0), Quaternion(0,0,0,0))
room = Pose(Point(4,3,0), Quaternion(0,0,0,0))
room1 = Pose(Point(4+off,3+off,0), Quaternion(0,0,0,0))
room2 = Pose(Point(4+off,3-off,0), Quaternion(0,0,0,0))
room3 = Pose(Point(4-off,3+off,0), Quaternion(0,0,0,0))
room4 = Pose(Point(4-off,3-off,0), Quaternion(0,0,0,0))
room_entrance = house_entrance
store = Pose(Point(-4,3,0), Quaternion(0,0,0,0))
store_entrance = Pose(Point(-4,0.5,0), Quaternion(0,0,0,0))
bakery = Pose(Point(-4,-1.5,0), Quaternion(0,0,0,0))
bakery_entrance = store_entrance

red = ColorRGBA(1.0,0,0,1.0)
green = ColorRGBA(0,1.0,0,1.0)

def init_poses():
    cake_pub.publish(Bool(False))
    decorations_pub.publish(Bool(False))
    agent1.publish_color(green)
    agent2.publish_color(red)
    agent3.publish_color(red)
    agent4.publish_color(red)
    time.sleep(1)

    agent1.update_pose(room1)
    agent2.update_pose(room2)
    agent3.update_pose(room3)
    agent4.update_pose(room4)

    agent1.publish_pose()
    agent2.publish_pose()
    agent3.publish_pose()
    agent4.publish_pose()

def move_path(agent, points, time):
    time_per = time / float(len(points))
    for point in points:
        agent.move(point, time_per, 200)

def block(init_time, relative_time):
    while True:
        if time.time() - init_time > relative_time:
            return
        else:
            time.sleep(0.01)

while False: #not rospy.is_shutdown():
    rospy.sleep(20)
    init_poses()
    init_time = rospy.get_time()    
    # A tells B @ 0.001 for 1
    
    block(init_time, 5)
    threading.Thread(target=agent1.move, args=(room_entrance, 1, 100)).start()
    threading.Thread(target=move_path, args=(agent2, [room_entrance, house2], 1)).start()

while not rospy.is_shutdown():
    rospy.sleep(15)
    init_poses()
    rospy.sleep(5)
    init_time = time.time()    
    # A tells B @ 0.001 for 1
    block(init_time, 0.001)
    agent2.publish_color(green)
    # move C house @ 0.001 for 2
    block(init_time, 0.001)
    threading.Thread(target=move_path, args=(agent3, [room_entrance, house3], 2.0)).start()
    # move A bakery @ 1.011 for 12
    block(init_time, 1.011)
    threading.Thread(target=move_path, args=(agent1, [room_entrance, bakery_entrance, bakery], 12.0)).start()    
    # move b house 1.012 for 2
    block(init_time, 1.012)
    threading.Thread(target=move_path, args=(agent2, [room_entrance, house2], 2)).start()
    # B tell C # 3.022 for 1
    block(init_time, 3.022)
    agent3.publish_color(green)
    # move C store 4.032 for 10
    block(init_time, 4.032)
    threading.Thread(target=move_path, args=(agent3, [room_entrance, store_entrance, store], 10)).start()
    # A pick up cake @ 13.022 for 3
    block(init_time, 13.022)
    cake_pub.publish(Bool(True))
    # A move house @ 16.032 for 12
    block(init_time, 16.032)
    threading.Thread(target=move_path, args=(agent1, [bakery_entrance, house_entrance, house1], 12)).start()
    # C pickup decorations @ 16.033 for 3
    decorations_pub.publish(Bool(True))
    # C move to house @ 19.043 for 10
    block(init_time, 19.043)
    threading.Thread(target=move_path, args=(agent3, [store_entrance, house_entrance, house3], 10)).start()
    # deliver cake
    # deliver decorations
    # D move to house
    block(init_time, 30)
    threading.Thread(target=move_path, args=(agent4, [room_entrance, house4], 2)).start()

    block(init_time, 40)




