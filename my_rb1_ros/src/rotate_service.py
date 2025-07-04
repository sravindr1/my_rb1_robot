#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from my_rb1_robot.srv import Rotate, RotateResponse
import tf
import math




class Service_to_Rotate:
    def __init__(self):
        
        
        rospy.init_node("rotate_service_server_node")
        self.service = rospy.Service('/rotate_robot', Rotate, self.rotate_robot)
    
    
        rospy.Subscriber('/odom', Odometry, self.check_odom_values)
        

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist,queue_size= 1 )
        self.cmd_msg = Twist()
        self.cmd_msg.linear.x = 0

        

        rospy.loginfo("rotate_robot service ready")
        rospy.spin()

    def check_odom_values(self, msg):
        
        robot_orientation = msg.pose.pose.orientation
        q = [robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w]
        (_,_,self.current_robot_yaw) = tf.transformations.euler_from_quaternion(q)
    
    def rotate_robot(self, request):
        rate = rospy.Rate(10)
        rospy.loginfo("Calculating difference angle")
        #rotate if difference in requested angle and current angle is > threshold = 1 degrees 
        threshold = math.radians(1)
        
        target_yaw = self.current_robot_yaw + request.degrees *math.pi/180
        difference_angle = self.normalize_angle( target_yaw - self.current_robot_yaw)
        rospy.loginfo("Difference angle %2f ", difference_angle)
        max_speed = 0.3
        

        while(abs(difference_angle)>threshold and not rospy.is_shutdown()): 
            self.cmd_msg.angular.z = max(-max_speed, min(max_speed, difference_angle * 0.15)) #max limit on rotation speed
            rospy.loginfo("Publishing difference angle")
            self.cmd_pub.publish(self.cmd_msg)
            difference_angle = self.normalize_angle(target_yaw - self.current_robot_yaw)
            
            rate.sleep()
        self.cmd_msg.angular.z = 0
        rospy.loginfo("Final yaw: %.2f", self.current_robot_yaw)
        
        self.cmd_pub.publish(self.cmd_msg)
        return RotateResponse(result="Success")

    def normalize_angle(self, x):
    # normalize rotation angle so that values range between -pi and +pi so no unnecessary rotation all the way 
    # formala (angle + pi)/(2*pi) - pi
        return (x + math.pi)%(2*math.pi) - math.pi


if __name__== "__main__":
    print("here")
    server = Service_to_Rotate()
    