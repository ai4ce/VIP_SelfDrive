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

# Defines the talker's interface to the rest of ROS
def talker():
    # Declares that your node is publishing to the chatter topic using the message type string. 
    # String is a class from std_msgs. 
    # The queue_size limits the amount of queued messages if any subscriber is not receiving them fast enough
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # init_node() names are node which is necessary for it to communicate wtih the master node
    # By setting anonymous = True our node will have a have random numbers appended to it to make it unique
    rospy.init_node('talker', anonymous=True)
    # A rate object allows us to loop at a desired rate. The integer passed as an arugment is tranlated into hertz
    rate = rospy.Rate(10) # 10hz
    # Does work if rospy is running. You must check shutdown to see if your code should exit
    while not rospy.is_shutdown():
        # hello_str has a string containing "hello world" and a timestamp
        hello_str = "hello world %s" % rospy.get_time()
        # loginfo() prints the message to the screen, writes it to the Node's log file, and writes it to rosout.
        # rosout is a tool for debugging
        rospy.loginfo(hello_str)
        # publish our string to the chatter topic
        pub.publish(hello_str)
        # Calling rate.slee() maintains the desired rate through the loop ()
        rate.sleep()
# Standard Python __main__ check
# Catches the ross interrupt exception which is thrown by rospy.sleep() and rospy.Rate.sleep() when Ctrl-S is pressed or the node is shutdown
# Raised to prevent continued execution of code after sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
