#!/usr/bin/env python2
# encoding=utf-8
import time
import rospy
from sensor_msgs.msg import JointState
from pymycobot.mycobot import MyCobot

import math

GRIP_PWM0 = 2050
GRIP_PWM1 = 1300
GRIP_DOPEN = 9
GRIP_DCLOSE = -40

SPD = 80 # 1-100

mc = None
#------
def set_grip(mc, d):
    pwm = GRIP_PWM0 * (1- d)  + d * GRIP_PWM1
    pwm = int(pwm)
    print("set grip pwm:", pwm)
    #mc.set_encoder(7, pwm)

#----
def calc_grip(r):
    #gv = int(abs(-0.7 - data_list[6])*117)
    gd = 180 * r / math.pi
    print("grip dgr = ", gd)
    gv = ( gd - GRIP_DOPEN ) / ( GRIP_DCLOSE - GRIP_DOPEN)
    print("grip val = ", gv)
    return gv


#-------
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "%s", data)
    data_list = []
    for index, value in enumerate(data.position):
        # if index != 2:
        #     value *= -1
        data_list.append(value)

    print("send_rdians():", data_list)
    mc.send_radians(data_list, 80)

    #----
    gr = data_list[6]
    print("grip radian gr=", gr)
    gv = calc_grip(gr)
    mc.set_gripper_value(int(gv*100), SPD)


#-------
def listener():
    global mc
    rospy.init_node("mycobot_reciver", anonymous=True)

    port = rospy.get_param("~port", "/dev/ttyAMA0")
    baud = rospy.get_param("~baud", 1000000)
    print(port, baud)
    mc = MyCobot(port, baud)

    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # spin() 只是阻止 python 退出，直到该节点停止
    rospy.spin()


if __name__ == "__main__":
    listener()
