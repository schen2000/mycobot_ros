#!/usr/bin/env python2
# encoding=utf-8
import time
import rospy
from sensor_msgs.msg import JointState
from pymycobot.mycobot import MyCobot
from threading import Thread, Lock

import math

GRIP_PWM0 = 2050
GRIP_PWM1 = 1300
GRIP_DOPEN = 9
GRIP_DCLOSE = -40

SPD = 80 # 1-100

# Note: this py disabled grip

class MyCobotSync(object):
    def __init__(self, mc):
        self.lock_ = Lock()
        self.mc_ = mc
    
    #------ deprecated
    def set_grip(self, d):
        pwm = GRIP_PWM0 * (1- d)  + d * GRIP_PWM1
        pwm = int(pwm)
        print("set grip pwm:", pwm)
        #self.mc_.set_encoder(7, pwm)

    #----
    def calc_grip(self, r):
        #gv = int(abs(-0.7 - data_list[6])*117)
        gd = 180 * r / math.pi
        print("grip dgr = ", gd)
        gv = ( gd - GRIP_DOPEN ) / ( GRIP_DCLOSE - GRIP_DOPEN)
        print("grip val = ", gv)
        return gv


    #-------
    def callback(self, data):
        self.lock_.acquire()
        rospy.loginfo(rospy.get_caller_id() + "%s", data)
        data_list = []
        for index, value in enumerate(data.position):
            # if index != 2:
            #     value *= -1
            data_list.append(value)

        #----
        rads = data_list[:6]
        #print("send_rdians():", rads)
        #self.mc_.send_radians(rads, SPD)
    
        angs = []
        for r in rads:
            angs.append(r * 180/math.pi)

        print("send_angles():", angs)
        self.mc_.send_angles(angs, SPD)

        #----
        gr = data_list[6]
        print("grip radian gr=", gr)
        gv = self.calc_grip(gr)
        #self.mc_.set_gripper_value(int(gv*100), SPD)
        self.lock_.release()


#-------
def listener():
    rospy.init_node("mycobot_reciver", anonymous=True)

    port = rospy.get_param("~port", "/dev/ttyAMA0")
    baud = rospy.get_param("~baud", 1000000)
    print(port, baud)
    mc = MyCobot(port, baud)
    mcsync = MyCobotSync(mc)

    rospy.Subscriber("joint_states", JointState, mcsync.callback)

    # spin() simply keeps python from exiting until this node is stopped
    # spin() 只是阻止 python 退出，直到该节点停止
    rospy.spin()


if __name__ == "__main__":
    listener()
