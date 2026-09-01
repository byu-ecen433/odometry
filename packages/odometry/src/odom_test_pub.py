#!/usr/bin/env python3

import rospy
import numpy as np
from duckietown_msgs.msg import WheelEncoderStamped

# One straight step drives both wheels forward by STEP_FORWARD. One turn step
# pivots about a stationary wheel: the moving wheel covers STEP_PIVOT, which for
# the Duckiebot baseline (0.1 m) works out to 45 degrees of rotation.
STEP_FORWARD = 0.1
STEP_PIVOT = 0.0785

# The BYU wordmark, traced as one continuous stroke. Each entry is (command,
# count): "F" drives forward, "L"/"R" pivot 45 degrees per step. The pen is
# always down, so the letters are connected by strokes along the baseline and a
# few strokes get drawn twice; those doubled strokes are hidden inside the
# letters (the B's middle bar, the Y's left arm).
BYU_PATTERN = [
    ("L", 2),                                   # face up before starting the B

    # --- B: spine, then a chamfered bowl above and below the middle bar ---
    ("F", 14),                                  # spine, bottom-left to top-left
    ("R", 2), ("F", 7),                         # top bar
    ("R", 1), ("F", 3), ("R", 1), ("F", 3),     # upper bowl, rounded corners
    ("R", 1), ("F", 3), ("R", 1), ("F", 7),     # ... back to the middle-left
    ("R", 4), ("F", 7),                         # middle bar, back out to the right
    ("R", 1), ("F", 3), ("R", 1), ("F", 3),     # lower bowl, rounded corners
    ("R", 1), ("F", 3), ("R", 1), ("F", 7),     # ... and the bottom bar

    # --- baseline run over to the foot of the Y ---
    ("R", 4), ("F", 16),

    # --- Y: stem, out to the left arm and back, then the right arm ---
    ("L", 2), ("F", 9),                         # stem up to the junction
    ("L", 1), ("F", 7),                         # left arm
    ("R", 4), ("F", 7),                         # back down to the junction
    ("L", 2), ("F", 7),                         # right arm

    # --- U: flows straight out of the top of the Y's right arm ---
    ("R", 3), ("F", 12),                        # left side, down
    ("L", 1), ("F", 3), ("L", 1), ("F", 4),     # rounded bottom
    ("L", 1), ("F", 3), ("L", 1), ("F", 12),    # ... and up the right side
]


def _expand(pattern):
    """Flatten the (command, count) pattern into one (dist_left, dist_right) pair
    per publishing step."""
    steps = []
    for command, count in pattern:
        for _ in range(count):
            if command == "F":
                steps.append((STEP_FORWARD, STEP_FORWARD))
            elif command == "L":
                steps.append((0.0, STEP_PIVOT))
            else:
                steps.append((STEP_PIVOT, 0.0))
    return steps


BYU_STEPS = _expand(BYU_PATTERN)


def pattern_generator(i):
    if i < len(BYU_STEPS):
        return BYU_STEPS[i]

    return (0, 0)


def make_msg(ticks, resolution):
    """Build a WheelEncoderStamped the way a Duckiebot's wheel encoder node does:
    'data' is the rolling (cumulative) tick count, not the ticks since the last
    message."""
    msg = WheelEncoderStamped()
    msg.header.stamp = rospy.Time.now()
    msg.data = ticks
    msg.resolution = resolution
    msg.type = WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL
    return msg


if __name__ == "__main__":
    rospy.init_node('wheel_tick_pub', anonymous=True)
    left_pub = rospy.Publisher("left_wheel_encoder_node/tick", WheelEncoderStamped, queue_size=10)
    right_pub = rospy.Publisher("right_wheel_encoder_node/tick", WheelEncoderStamped, queue_size=10)
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

    for i in range(len(BYU_STEPS)):
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
