#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Joy


class JoyTwist(object):
    def __init__(self):
        self._joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback, queue_size=1)
        self._twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._smooth_twist_pub = rospy.Publisher('/raw_cmd_vel', Twist, queue_size=1)
        
        self.smooth_flag = False
        self.level = 1
    def limitter(self, lvl):
        if lvl <= 0:
            return 1
        if lvl >= 6:
            return 5
        return lvl

    def joy_callback(self, joy_msg):
        if joy_msg.buttons[7] == 1:
            self.level += 1
        if joy_msg.buttons[6] == 1:
            self.level -= 1
        self.level = self.limitter(self.level)

        twist = Twist()
        if joy_msg.buttons[0] == 1:
            # uncomment the following two lines to use speed up function
            #twist.linear.x = joy_msg.axes[1] * 0.4 * self.level
            #twist.angular.z = joy_msg.axes[0] * 3.14 / 32 * (self.level + 15)
            # comment out the following two lines to use speed up function
            twist.linear.x = joy_msg.axes[1] * 0.4
            twist.angular.z = joy_msg.axes[0] * 3.14 / 32 * 15
            self._twist_pub.publish(twist)
            self.smooth_flag = False
        elif joy_msg.buttons[2] == 1:
            # uncomment the following two lines to use speed up function
            #twist.linear.x = joy_msg.axes[1] * 0.4 * self.level
            #twist.angular.z = joy_msg.axes[0] * 3.14 / 32 * (self.level + 15)
            # comment out the following two lines to use speed up function
            twist.linear.x = joy_msg.axes[1] * 0.4
            twist.angular.z = joy_msg.axes[0] * 3.14 / 32 * 15
            self._smooth_twist_pub.publish(twist)
            self.smooth_flag = True;
        else:
            twist.linear.x = 0
            twist.angular.z = 0
            if self.smooth_flag:
                self._smooth_twist_pub.publish(twist)
            elif not self.smooth_flag:
                self._twist_pub.publish(twist)

        if joy_msg.axes[1] == joy_msg.axes[0] == 0:
            self.level -= 1

if __name__ == '__main__':
    rospy.init_node('logicool_cmd_vel')

    if rospy.get_param("/logicool_cmd_vel/motor_on_off"):
        rospy.loginfo("motor_on and motor_off service call has been enabled.")
        rospy.loginfo("waiting for service...")
        rospy.wait_for_service('/motor_on')
        rospy.wait_for_service('/motor_off')
        rospy.loginfo("motor_on and motor_off service found.")
        rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
        rospy.ServiceProxy('/motor_on', Trigger).call()
    else:
        rospy.loginfo("motor_on and motor_off service call has been disabled.")
    
    logicool_cmd_vel = JoyTwist()
    rospy.spin()
