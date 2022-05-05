#! /usr/bin/env python

import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose


class marker_rebroadcaster():

    def __init__(self):
        
        rospy.init_node("marker")

        self.pub = rospy.Publisher(rospy.get_name()+"/marker", Marker, queue_size = 2)
        self.pose_sub = rospy.Subscriber(rospy.get_name()+"/pose", Pose, self.pose_callback)
        self.color_sub = rospy.Subscriber(rospy.get_name()+"/color", ColorRGBA, self.color_callback)

        self.marker = Marker()

        self.marker.header.frame_id = "/map"
        self.marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        self.marker.type = 2
        self.marker.id = 0

        # Set the scale of the marker
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2

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
        
        self.pub.publish(self.marker)

    def color_callback(self, color):
        self.marker.color.r = color.r
        self.marker.color.g = color.g
        self.marker.color.b = color.b
        self.marker.color.a = color.a

        self.pub.publish(self.marker)


marker_rebroadcaster()
