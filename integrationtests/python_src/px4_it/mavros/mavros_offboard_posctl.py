#!/usr/bin/env python2
#***************************************************************************
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#***************************************************************************/

#
# @author Andreas Antener <andreas@uaventure.com>
#
# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.
from __future__ import division

PKG = 'px4'

import rospy
import math
import numpy as np
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler
import zmq

# Will need to change this if communicating between different machines.
# IP_ADDRESS = "192.168.20.3"
# IP_ADDRESS = "10.42.0.79"
# IP_ADDRESS = "192.168.1.4"
IP_ADDRESS = "localhost"

# Must match the one in server_gps.py
PORT = 5556

class MavrosOffboardPosctl(MavrosTestCommon):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def setUp(self):
        super(MavrosOffboardPosctl, self).setUp()

        # Socket receive readings from the server.
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB, )
        rospy.loginfo("connecting to server")
        self.socket.connect("tcp://{}:{}".format(IP_ADDRESS, PORT))
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.recv_attempts = 0

        # self.pos = PoseStamped()
        self.pos = GeoPoseStamped()
        self.radius = 1

        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/global', GeoPoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def tearDown(self):
        super(MavrosOffboardPosctl, self).tearDown()

    #
    # Helper methods
    #
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.latitude = x
        self.pos.pose.position.longitude = y
        self.pos.pose.position.altitude = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.global_position.latitude,
                   self.global_position.longitude,
                   self.global_position.altitude))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        # loop_freq = 2  # Hz
        # rate = rospy.Rate(loop_freq)
        # reached = False
        # for i in xrange(timeout * loop_freq):
        #     if self.is_at_position(self.pos.pose.position.latitude,
        #                            self.pos.pose.position.longitude,
        #                            self.pos.pose.position.altitude, self.radius):
        #         rospy.loginfo("position reached | seconds: {0} of {1}".format(
        #             i / loop_freq, timeout))
        #         reached = True
        #         break
        #
        #     try:
        #         rate.sleep()
        #     except rospy.ROSException as e:
        #         self.fail(e)
        #
        # self.assertTrue(reached, (
        #     "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
        #     format(self.global_position.latitude,
        #            self.global_position.longitude,
        #            self.global_position.altitude, timeout)))

    def server_disconnected(self, max_attempts):
        """Returns true if failed recieve attempts is above threshold"""
        if self.recv_attempts != 0 and self.recv_attempts % 10 == 0:
            rospy.loginfo("{} failed attempts to recieve setpoint".format(self.recv_attempts))
        return self.recv_attempts > max_attempts

    def update_setpoint(self, altitude, timeout):
        """Update setpoint based on data from server"""
        try:
            message = self.socket.recv(flags=zmq.NOBLOCK).decode('utf-8')
        except zmq.ZMQError:
            self.recv_attempts += 1
        else:
            self.recv_attempts = 0
            time_sample, latitude, longitude = message.split(',')
            lat = float(latitude)
            lon = float(longitude)
            self.reach_position(lat, lon, altitude, timeout)

    #
    # Test method
    #
    def test_posctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")
        altitude = 20
        timeout = 30 # seconds
        max_attempts = 100
        rate = rospy.Rate(5)

        while not self.server_disconnected(max_attempts):
            self.update_setpoint(altitude, timeout)
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        if self.server_disconnected(max_attempts):
            rospy.loginfo("maximum number of failed attempts to recieve setpoint ({}) reached".format(self.recv_attempts))

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)

    controller = MavrosOffboardPosctl()
    controller.setUp()
    controller.test_posctl()
    controller.tearDown()

    # rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
    #                MavrosOffboardPosctlTest)
