#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtle_actionlib.msg import Velocity
import numpy as np


def talker():
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 1hz
    new_location = Twist()
    f = open("/home/aruna/catkin_ws/src/project3/scripts/nodePath.txt", "r+")
    lines = f.readlines()
    linesread = [line.rstrip() for line in lines]
    for loc in linesread:
        if len(loc.split(',')) < 6:
            continue
    # float(lines.split(',')[3] = dx/dt  float(lines.split(',')[4]) = dy/dt  np.deg2rad(float(lines.split(',')[5]) * 100) = dx
        new_location.linear.x = np.sqrt(float(loc.split(',')[3]) ** 2 + float(loc.split(',')[4]) ** 2)/100
        new_location.angular.z = np.deg2rad(float(loc.split(',')[5]) * 100)/100
        for i in range(1000):
            # new_location.angular.z = 0.314
            # t0 = rospy.Time.now().to_sec()
            # while not rospy.is_shutdown():
            #     t1 = rospy.Time.now().to_sec()
            #     print(t1 - t0)
            #     if t1 - t0 >= 9:
            #         break
                # hello_str = loc
                # print(new_location)
            rospy.loginfo(new_location)
            pub.publish(new_location)
            rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
