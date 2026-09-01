#!/usr/bin/env python3

import rospy
import numpy as np
from duckietown_msgs.msg import WheelEncoderStamped

def pattern_generator(i):
    if i < 2:
        return (0,0.0785) # left
    elif i >= 2 and i < 22:
        return (.1,.1) # up before U
    elif i >= 22 and i < 24:
        return (0.0785,0) # right
    elif i >= 24 and i < 25:
        return (0.1,0.1) # space before U
    elif i >= 25 and i < 27:
        return (0.0785,0) # right
    elif i >= 27 and i < 47:
        return (.1,.1) # down of U
    elif i >= 47 and i < 49:
        return (0,0.0785) # left
    elif i >= 49 and i < 60:
        return (.1,.1) # bottom of U
    elif i >= 60 and i < 62:
        return (0,0.0785) # left
    elif i >= 62 and i < 82:
        return (.1,.1) # up of U/M
    elif i >= 82 and i < 85:
        return (0.0785,0) # right 135
    elif i >= 85 and i < 95:
        return (.1,.1) # down diagonal
    elif i >= 95 and i < 97:
        return (0,0.0785) # left
    elif i >= 97 and i < 107:
        return (.1,.1) # up diagonal
    elif i >= 107 and i < 110:
        return (0.0785,0) # right 135
    elif i >= 110 and i < 130:
        return (.1,.1) # down for M/L    
    elif i >= 130 and i < 132:
        return (0,0.0785) # left
    elif i >= 132 and i < 145:
        return (.1,.1) # bottom of L
        
    return (0,0)

def make_msg(ticks, resolution):
    """Build a WheelEncoderStamped just like a Duckiebot's wheel encoder node does:
    'data' is the rolling (cumulative) tick count, not the ticks since last message."""
    msg = WheelEncoderStamped()
    msg.header.stamp = rospy.Time.now()
    msg.data = ticks
    msg.resolution = resolution
    msg.type = WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL
    return msg

if __name__ == "__main__":
    rospy.init_node('wheel_tick_pub', anonymous=True)
    left_pub = rospy.Publisher("left_wheel_encoder_driver_node/tick", WheelEncoderStamped, queue_size=10)
    right_pub = rospy.Publisher("right_wheel_encoder_driver_node/tick", WheelEncoderStamped, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    R = 0.0318
    N_TOTAL = 135 # encoder resolution, ticks per full wheel revolution
    alpha = 2 * np.pi / N_TOTAL

    # rolling tick counters, as reported by the wheel encoders
    ticks_left = 0
    ticks_right = 0
    # distance already accounted for by the published tick counts is
    # tracked separately so that fractional ticks are not thrown away
    dist_left_total = 0.0
    dist_right_total = 0.0

    for i in range(50):
        left_pub.publish(make_msg(ticks_left, N_TOTAL))
        right_pub.publish(make_msg(ticks_right, N_TOTAL))
        if rospy.is_shutdown():
            break
        rate.sleep()

    for i in range(160):
        dist_left,dist_right = pattern_generator(i)
        dist_left_total += dist_left
        dist_right_total += dist_right
        ticks_left = int(dist_left_total / (R * alpha))
        ticks_right = int(dist_right_total / (R * alpha))
        rospy.logwarn("left: %d right: %d" % (ticks_left,ticks_right))
        left_pub.publish(make_msg(ticks_left, N_TOTAL))
        right_pub.publish(make_msg(ticks_right, N_TOTAL))
        if rospy.is_shutdown():
            break
        rate.sleep()
