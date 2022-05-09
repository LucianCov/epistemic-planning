#! /usr/bin/env python

import rospy
from rospy.core import rospyinfo
from rospy.names import get_name
from std_msgs.msg import ColorRGBA, Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point


class marker_rebroadcaster():

    def __init__(self):
        
        rospy.init_node("marker")

        self.pub = rospy.Publisher(rospy.get_name()+"/marker", Marker, queue_size = 2)
        self.pose_sub = rospy.Subscriber(rospy.get_name()+"/pose", Pose, self.pose_callback)
        self.color_sub = rospy.Subscriber(rospy.get_name()+"/color", ColorRGBA, self.color_callback)

        self.cake_toggle = False
        self.decoration_toggle = False
        self.has_cake = False
        self.has_decoration = False
        if rospy.get_name() == "/agent_1":
            self.cake_sub = rospy.Subscriber(rospy.get_name()+"/cake", Bool, self.cake_callback)
            self.cake_pub = rospy.Publisher("cake", Marker, queue_size=2)
            self.has_cake = True
        if rospy.get_name() == "/agent_3":
            self.decoration_sub = rospy.Subscriber(rospy.get_name()+"/decoration", Bool, self.decoration_callback)
            self.decoration_pub = rospy.Publisher("decoration", Marker, queue_size=2)
            self.has_decoration = True

        self.marker = Marker()

        self.marker.header.frame_id = "/map"
        self.marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        self.marker.type = 2
        self.marker.id = 0

        # Set the scale of the marker
        self.marker.scale.x = 0.3
        self.marker.scale.y = 0.3
        self.marker.scale.z = 0.3

        # Set the color
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 0

        self.cake = Marker()
        self.cake.header.frame_id = "/map"
        self.cake.color = ColorRGBA(0,0,1,1)
        self.cake.pose = Pose()
        self.cake.scale.x = 0.2
        self.cake.scale.y = 0.2
        self.cake.scale.z = 0.2
        self.cake.type = 1
        
        self.decoration = Marker()
        self.decoration.header.frame_id = "/map"
        self.decoration.color = ColorRGBA(0,0,1,1)
        self.decoration.pose = Pose()
        self.decoration.scale.x = 0.2
        self.decoration.scale.y = 0.2
        self.decoration.scale.z = 0.2
        self.decoration.type = 1
        
        rospy.spin()
    
    def pose_callback(self, pose):


        # Set the pose of the marker
        self.marker.pose.position.x = pose.position.x
        self.marker.pose.position.y = pose.position.y
        self.marker.pose.position.z = pose.position.z
        self.marker.pose.orientation.x = pose.orientation.x
        self.marker.pose.orientation.y = pose.orientation.y
        self.marker.pose.orientation.z = pose.orientation.z
        self.marker.pose.orientation.w = pose.orientation.w

        off = 0.1
        if self.has_cake:
            if self.cake_toggle:
                self.cake.pose.position = Point(pose.position.x+off, pose.position.y+off, 0)
            else:
                self.cake.pose.position = Point(-4+off, -1.5+off, 0)
            self.cake_pub.publish(self.cake)

        if self.has_decoration:
            if self.decoration_toggle:
                self.decoration.pose.position = Point(pose.position.x+off, pose.position.y+off, 0)
            else:
                self.decoration.pose.position = Point(-4+off, 3+off, 0)
            self.decoration_pub.publish(self.decoration)

        
        self.pub.publish(self.marker)


    def color_callback(self, color):
        self.marker.color.r = color.r
        self.marker.color.g = color.g
        self.marker.color.b = color.b
        self.marker.color.a = color.a

        self.pub.publish(self.marker)

    def cake_callback(self, switch):
        self.cake_toggle = switch.data

    def decoration_callback(self, switch):
        self.decoration_toggle = switch.data


marker_rebroadcaster()
