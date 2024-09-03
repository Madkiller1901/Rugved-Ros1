#!/usr/bin/env python3

from math import inf
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
rospy.init_node("centering_node")
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

class pid_controller:
    def __init__(self,kp,ki,kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
    def computelinear(self,kp,setpoint, process_variable):
        error = setpoint - process_variable
        self.integral += error
        output = kp * error + self.ki * self.integral + self.kd * (error - self.prev_error)
        self.prev_error = error
        return output
    def computeangular(self,setpoint, process_variable):
        error = setpoint - process_variable
        self.integral += error
        output = self.kp * error + self.ki * self.integral + self.kd * (error - self.prev_error)
        self.prev_error = error
        return output

def callback(msg):
       twist_msg = Twist()
       x=msg.ranges[0]
       y=msg.ranges[2]
       l=msg.ranges[1]
       if(x==inf):
           x=5
       if(y==inf):
           y=5
       pid=pid_controller(1,0.7,0.3)
       if(l<5.5):
          b=pid.computelinear(0.1,1.3,3.0)
          twist_msg.linear.x=3.0+b
       else:
          twist_msg.linear.x=3.5
       a=pid.computeangular((x+y)/2,x)
       twist_msg.angular.z=-a
       pub.publish(twist_msg)

             

rospy.Subscriber("/rugbot2/scan",LaserScan, callback)
rospy.spin()




