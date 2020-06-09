#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopJoy(object):
    
    def __init__(self):
        """Initialize node."""
        # get parameter values
        self.scale_lin = rospy.get_param('~scale_linear', 0.15)
        self.scale_ang = rospy.get_param('~scale_angular', 1.82)

        # publisher
        self.cmd_vel_pub = rospy.Publisher('/youbot_teleop', Twist, queue_size=1)

        #subscriber
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_cb, queue_size=1)
        #self.last_command = False

    def joy_cb(self, joy_msg):
        """Publish command velocity according to joy message."""
        cmd_vel_msg = Twist()
        if joy_msg.buttons[0]:
            cmd_vel_msg.linear.x = self.scale_lin * joy_msg.axes[1]
            cmd_vel_msg.linear.y = self.scale_lin * joy_msg.axes[0]
            cmd_vel_msg.angular.z = self.scale_ang * joy_msg.axes[3]
        self.cmd_vel_pub.publish(cmd_vel_msg)

def main():
    rospy.init_node('teleop_joy')
    teleop_joy = TeleopJoy()
    rospy.spin()

if __name__ == '__main__':
    main()
