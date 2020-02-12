#!/usr/bin/env python2

import numpy as np
import roslib
import rospy

from robot_control_modules import *
roslib.load_manifest('kinova_demo')

prefix = 'j2s6s200'
nbJoints = 6

if __name__ == '__main__':
    try:
        prefix, nbJoints = argumentParser(None)

        rospy.init_node('go_home')

        # Preset joint configurations
        jaco_candle = [180, 180, 180, 180, 180, 180]
        jaco_retracted = [270, 150, 25, 360, 212.5, 180]

        result = joint_position_client(jaco_retracted + [0, 0], prefix)

        print("Done!")

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
