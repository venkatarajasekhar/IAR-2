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
    #rospy.loginfo(rospy.get_caller_id()+" Lasers %s"," ".join(map(str,lasers)))


# replace 'my_behavior' by the name of your behavior
def forward():
    rospy.init_node('forward', anonymous=True)

    # remove the subscriptions you don't need.
    rospy.Subscriber("/simu_fastsim/laser_scan", LaserScan, callback_lasers)
    # rospy.Subscriber("/simu_fastsim/right_bumper", Bool, callback_right_bumper)
    # rospy.Subscriber("/simu_fastsim/left_bumper", Bool, callback_left_bumper)
    
    # replace /subsomption/channel0 by /subsomption/channeli where i is the number of the channel you want to publish in
    pub = rospy.Publisher('/subsomption/channel2', Channel , queue_size=10)
    r = rospy.Rate(10) # 10hz
    v=Channel()
    v.activated=True

    distanceTooClose = 80
    avoidSpeed = -1
    followSpeed = 1
    
    while not rospy.is_shutdown():
        if any([i < distanceTooClose for i in lasers.ranges[:len(lasers.ranges)/2]]):  # Détection d'un mur à éviter à gauche
            v.activated = True
            v.speed_left = avoidSpeed
            v.speed_right = followSpeed
        elif any([i < distanceTooClose for i in lasers.ranges[len(lasers.ranges)/2:]]):  # Détection d'un mur à éviter à droite
            v.activated = True
            v.speed_left = followSpeed
            v.speed_right = avoidSpeed
        else:
            v.activated = False

        # compute the value of v that will be sent to the subsomption channel. 
        pub.publish(v)
        r.sleep()   


if __name__ == '__main__':
    random.seed()
    try:
        forward()
    except rospy.ROSInterruptException: pass
