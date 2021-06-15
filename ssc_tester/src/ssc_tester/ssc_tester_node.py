#!/usr/bin/env python3
#

#
# Copyright (C) 2019-2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
#
import time
import rospy
import threading
import math

from std_msgs.msg import Bool
from autoware_msgs.msg import VehicleCmd
from automotive_platform_msgs.msg import SteerMode

# SSCTesterNode
# Reads in a yaml list which specifies a desired speed or steering profile and executes it
class SSCTesterNode(object):
    def __init__(self):
        # Init nodes
        rospy.init_node('ssc_tester')

        # Params
        self.cmd_rate = rospy.get_param('~cmd_rate')
        self.values = rospy.get_param('~values')
        self.value_period = rospy.get_param('~value_period')
        self.loop_values = rospy.get_param('~loop_values')
        self.publish_steer_not_speed = rospy.get_param('~publish_steer_not_speed')

        if (self.publish_steer_not_speed):
            print("NOTE: You are publishing STEERING commands. Do not engage unless this is intended behavior")
        else:
            print("NOTE: You are publishing SPEED commands. Do not engage unless this is intended behavior")

        self.current_value_index = 0

        # Publishers
        self.cmd_pub = rospy.Publisher('vehicle_cmd', VehicleCmd, queue_size=10)
        self.steer_override_pub = rospy.Publisher('steer', SteerMode, queue_size=10)

        # Subscribers
        self.engage_sub = rospy.Subscriber('engage', Bool, self.engage_cb)

        self.engaged = False
        self.engage_lock = threading.Lock()
        
        #Spin Rate
        self.spin_rate = rospy.Rate(self.cmd_rate)

        self.lastTime = rospy.Time.now()


    # Function to handle engage signal
    def engage_cb(self, engage_msg):
        with self.engage_lock: # Lock mutex
            self.engaged = engage_msg.data
            if (self.engaged): 
                print("Engaging commands")
                self.lastTime = rospy.Time.now()
                self.current_value_index = 0
            else:
                print("Disengaging commands")

    def nextCmd(self):
        # Initialize command with disengaged as default
        current_time = rospy.Time.now()
        cmd_msg = VehicleCmd()
        cmd_msg.header.frame_id = 'base_link'
        cmd_msg.header.stamp = current_time
        cmd_msg.gear = 4
        cmd_msg.mode = 0 # Disable automated control

        target_duration = self.value_period[self.current_value_index]
        with self.engage_lock: # Lock mutex
            if (self.engaged):

                if (rospy.rostime.Duration(target_duration) < (current_time - self.lastTime)):
                    self.current_value_index += 1
                    print('Commanding next value at index ' + str(self.current_value_index))
                    self.lastTime = current_time

                    if (self.current_value_index >= len(self.values)):
                        self.current_value_index = 0
                        print('Reached end of values list')

                        if (not self.loop_values):
                            print('Disengaging')
                            self.engaged = False # Disengage if at end of speed list
                            return cmd_msg
                        else:
                            print('Returning to start of value list')

                current_value = self.values[self.current_value_index]

                if (self.publish_steer_not_speed): # Publish values as steering commands
                    cmd_msg.gear = 4
                    cmd_msg.mode = 1
                    cmd_msg.ctrl_cmd.steering_angle = math.radians(current_value)

                else: # Publish values as speed commands
                    cmd_msg.gear = 4
                    cmd_msg.mode = 1
                    cmd_msg.ctrl_cmd.linear_velocity = current_value
                    cmd_msg.ctrl_cmd.linear_acceleration = 0.0

            return cmd_msg

    def spin(self):
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.nextCmd())
            
            self.spin_rate.sleep()

if __name__ == '__main__':
    try:
        tester = SSCTesterNode()
        tester.spin()
    except rospy.ROSInterruptException:
        pass
