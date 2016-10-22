#!/usr/bin/env python
#! -*- coding: utf-8 -*-

import rospy
from subsomption.msg import Channel 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

import random

bumper_l=0
bumper_r=0
lasers=LaserScan()
speed=2
threshold=50
timer=10


def callback_right_bumper(data):
    global bumper_r
    bumper_r=data.data
    # rospy.loginfo(rospy.get_caller_id()+"Right bumper %d",data.data)

def callback_left_bumper(data):
    global bumper_l
    bumper_l=data.data
    # rospy.loginfo(rospy.get_caller_id()+"Left bumper %d",data.data)


def callback_lasers(data):
    global lasers
    lasers=data
    # rospy.loginfo(rospy.get_caller_id()+" Lasers %s"," ".join(map(str,lasers)))


# replace 'my_behavior' by the name of your behavior
def follow_walls():
    rospy.init_node('follow_walls', anonymous=True)

    # remove the subscriptions you don't need.
    rospy.Subscriber("/simu_fastsim/laser_scan", LaserScan, callback_lasers)
    # rospy.Subscriber("/simu_fastsim/right_bumper", Bool, callback_right_bumper)
    # rospy.Subscriber("/simu_fastsim/left_bumper", Bool, callback_left_bumper)
    
    # replace /subsomption/channel0 by /subsomption/channeli where i is the number of the channel you want to publish in
    pub = rospy.Publisher('/subsomption/channel1', Channel , queue_size=10)
    r = rospy.Rate(10) # 10hz
    v=Channel()
    v.activated=True

    count=0
    limiteMax = 80
    limiteMin = 50
    avoidSpeed = -1
    followSpeed = 1

    while not rospy.is_shutdown():
        print lasers.ranges
        if any([limiteMax > i >= limiteMin for i in lasers.ranges[:len(lasers.ranges)/2]]):  # Detection d'un mur a suivre a gauche
            v.activated = True
            v.speed_left = followSpeed
            v.speed_right = avoidSpeed
            count = 0
        elif any([i < limiteMin for i in lasers.ranges[:len(lasers.ranges)/2]]):  # Detection d'un mur trop proche a gauche
            v.activated = True
            v.speed_left = avoidSpeed
            v.speed_right = followSpeed
            count = 0
        elif any([limiteMax > i >= limiteMin for i in lasers.ranges[len(lasers.ranges)/2:]]):  # Detection d'un mur a suivre a droite
            v.activated = True
            v.speed_left = avoidSpeed
            v.speed_right = followSpeed
            count = 0
        elif any([i < limiteMin for i in lasers.ranges[len(lasers.ranges)/2:]]):  # Detection d'un mur trop proche a droite
            v.activated = True
            v.speed_left = followSpeed
            v.speed_right = avoidSpeed
            count = 0
        else:
            if count < 5:
                count += 1
            else:
                v.activated = False
                count = 0

        # compute the value of v that will be sent to the subsomption channel. 
        pub.publish(v)
        r.sleep()   


if __name__ == '__main__':
    random.seed()
    try:
        follow_walls()
    except rospy.ROSInterruptException: pass
