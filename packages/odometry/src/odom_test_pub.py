#!/usr/bin/env python3

import rospy
import numpy as np
from duckietown_msgs.msg import WheelEncoderStamped

BASELINE = 0.1          # distance between the duckiebot's wheels (m)
STEP_FORWARD = 0.1      # straight-line distance covered by one published step (m)
STEP_TURN = np.pi / 4   # largest pivot covered by one published step (rad)

# Outline of the BYU stretch-Y, traced as one closed loop starting from the
# bottom-left corner of the foot: up the left of the stem, out over the left arm,
# down through the notch, back out over the right arm and home along the foot.
# Coordinates are in metres; the mark comes out 2.20 m wide by 1.60 m tall.
Y_OUTLINE = [
    (0.63, 0.05), (0.68, 0.27), (0.88, 0.24), (0.88, 0.54),   # left of the foot and stem
    (0.30, 1.19), (0.14, 1.08), (0.00, 1.25),                 # under the left arm, out to the tip
    (0.39, 1.48), (0.91, 1.60),                               # top edge of the left arm
    (0.96, 1.37), (0.77, 1.34), (1.23, 0.80),                 # down into the notch between the arms
    (1.65, 1.30), (1.45, 1.34), (1.48, 1.57),                 # back up the inside of the right arm
    (1.86, 1.46), (2.20, 1.26), (2.06, 1.08), (1.89, 1.20),   # right arm tip
    (1.34, 0.53), (1.34, 0.25), (1.55, 0.28), (1.60, 0.06),   # right of the stem and foot
    (1.22, 0.00),                                             # bottom of the foot
]


# The oval the logo rings the Y with. It is concentric with the mark and clears
# it on every side, so the finished drawing is 2.96 m wide by 1.92 m tall.
OVAL_CENTRE = (1.10, 0.80)
OVAL_SEMI_AXES = (1.48, 0.96)
OVAL_WAYPOINTS = 48


def _wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def _oval_waypoints(from_x, from_y):
    """The oval, broken into waypoints. It starts at whichever point is nearest
    (from_x, from_y) so the run out from the foot of the Y is as short as
    possible, and winds the same way round as the outline."""
    centre_x, centre_y = OVAL_CENTRE
    semi_x, semi_y = OVAL_SEMI_AXES
    start = np.arctan2((from_y - centre_y) / semi_y, (from_x - centre_x) / semi_x)
    angles = [start - 2 * np.pi * i / OVAL_WAYPOINTS for i in range(OVAL_WAYPOINTS + 1)]

    return [(centre_x + semi_x * np.cos(a), centre_y + semi_y * np.sin(a)) for a in angles]


def _steps_along(waypoints):
    """Drive through the waypoints in order, returning the (dist_left, dist_right)
    pair to publish at each step.

    Corners are taken by pivoting about whichever wheel is held still, so both
    wheels only ever roll forward and the tick counts only ever climb. A pivot
    also drags the robot sideways, so rather than turning by a precomputed angle
    we re-aim at the target waypoint after every pivot step; that keeps the path
    on the waypoints instead of letting the corner drift accumulate around the
    loop.
    """
    steps = []
    x, y = waypoints[0]
    heading = 0.0

    for target_x, target_y in waypoints[1:]:
        for _ in range(40):
            error = _wrap(np.arctan2(target_y - y, target_x - x) - heading)
            if abs(error) < 1e-4:
                break
            turn = max(-STEP_TURN, min(STEP_TURN, error))
            steps.append((0.0, abs(turn) * BASELINE) if turn > 0 else (abs(turn) * BASELINE, 0.0))
            # the stationary wheel the robot swings around
            side = 1.0 if turn > 0 else -1.0
            wx = x - side * np.sin(heading) * BASELINE / 2
            wy = y + side * np.cos(heading) * BASELINE / 2
            c, s = np.cos(turn), np.sin(turn)
            x, y = wx + c * (x - wx) - s * (y - wy), wy + s * (x - wx) + c * (y - wy)
            heading += turn

        dist = np.hypot(target_x - x, target_y - y)
        n = max(1, int(round(dist / STEP_FORWARD)))
        for _ in range(n):
            steps.append((dist / n, dist / n))
        x, y = target_x, target_y

    return steps


# the outline, closed back onto its first point, then out to the oval and round it
Y_PATH = Y_OUTLINE + [Y_OUTLINE[0]] + _oval_waypoints(*Y_OUTLINE[0])
Y_STEPS = _steps_along(Y_PATH)


def pattern_generator(i):
    if i < len(Y_STEPS):
        return Y_STEPS[i]

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

    for i in range(len(Y_STEPS)):
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
