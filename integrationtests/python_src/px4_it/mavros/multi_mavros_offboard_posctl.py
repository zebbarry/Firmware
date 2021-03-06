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

import rospy
import math
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Quaternion
from multi_mavros_common import MultiMavrosCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
import zmq

# Will need to change this if communicating between different machines.
# IP_ADDRESS = "192.168.20.3"
# IP_ADDRESS = "10.42.0.79"
# IP_ADDRESS = "192.168.1.4"
IP_ADDRESS = "localhost"

# Must match the one in server_gps.py
PORT = 5556

DEFAULT_ALTITUDE = 20
MAX_RECV_ATTEMPTS = 100
OFFSETS = [(0, 0), (0.0001, -0.0001), (0.0001, 0.0001), (-0.0001, -0.0001), (-0.0001, 0.0001)]

class MultiMavrosOffboardPosctl(MultiMavrosCommon):
    """ Controls a drone in Gazebo by sending position setpoints via MAVROS. """


    def set_up(self, uav_id=-1):
        rospy.init_node('offboard', anonymous=True)
        super(MultiMavrosOffboardPosctl, self).set_up(uav_id)

        if uav_id != -1:
            self.namespace = 'uav' + str(uav_id) + '/'
        else:
            self.namespace = ''

        self.pos = GeoPoseStamped()
        self.pos_setpoint_pub = rospy.Publisher(
            self.namespace + 'mavros/setpoint_position/global', GeoPoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def tear_down(self):
        """ Teardown, including landing and disarming the drone. """
        super(MultiMavrosOffboardPosctl, self).tear_down()

    def take_off(self):
        """ Set mode to offboard and takeoff """
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)
        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

    def land(self):
        """ Land UAV and disarm """
        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
        self.set_arm(False, 5)

    def send_pos(self):
        """ Runs in a separate thread, publishing current position at a rate of 10 Hz. """
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

    def reach_position(self, lat, lon, alt):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.latitude = lat
        self.pos.pose.position.longitude = lon
        self.pos.pose.position.altitude = alt

        # Remove tf dependency for now to allow running with Python3
        # (this is equivalent to setting heading to North)
        self.pos.pose.orientation = Quaternion(0, 0, 0, 1)



class Controller():

    def __init__(self, num_uavs=1):
        # Socket receive readings from the server.
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB, )
        rospy.loginfo("connecting to server")
        self.socket.connect("tcp://{}:{}".format(IP_ADDRESS, PORT))
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.recv_attempts = 0

        self.num_uavs = num_uavs
        self.uavs = []
        for i in range(num_uavs):
            self.uavs.append(MultiMavrosOffboardPosctl())
            self.uavs[i].set_up(i)

    def tear_down(self):
        for uav in self.uavs:
            uav.tear_down()

    def server_disconnected(self, max_attempts):
        """Returns true if failed recieve attempts is above threshold"""
        if self.recv_attempts != 0 and self.recv_attempts % 10 == 0:
            rospy.loginfo("{} failed attempts to recieve setpoint".format(self.recv_attempts))
        return self.recv_attempts >= max_attempts

    def update_setpoint(self):
        """Update setpoint based on data from server"""
        try:
            message = self.socket.recv(flags=zmq.NOBLOCK).decode('utf-8')
        except zmq.ZMQError:
            self.recv_attempts += 1
            return False
        else:
            self.recv_attempts = 0
            time_sample, latitude, longitude = message.split(',')
            self.setpoint_lat = float(latitude)
            self.setpoint_lon = float(longitude)
            return True

    def reach_position(self, lat, lon, alt):
        """Set setpoints for each drone with predefined offset"""
        rospy.loginfo(
            "setpoint position | x: {0}, y: {1}, z: {2:.1f}".
            format(lat, lon, alt))

        for i in range(self.num_uavs):
            lat_offset = lat + OFFSETS[i][0]
            lon_offset = lon + OFFSETS[i][1]
            self.uavs[i].reach_position(lat_offset, lon_offset, alt)

    def take_off(self):
        """Set mode to offboard and takeoff all uavs"""
        for uav in self.uavs:
            uav.take_off()

    def land(self):
        """Land all uavs and disarm"""
        for uav in self.uavs:
            uav.land()

    def run_posctl(self):
        """Test multi uav posctl simultaneously"""
        rospy.loginfo("run mission")
        self.setpoint_alt = DEFAULT_ALTITUDE
        rate = rospy.Rate(5)

        while not self.server_disconnected(MAX_RECV_ATTEMPTS):
            if self.update_setpoint():
                self.reach_position(self.setpoint_lat, self.setpoint_lon, self.setpoint_alt)
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        if self.server_disconnected(MAX_RECV_ATTEMPTS):
            rospy.loginfo("maximum number of failed attempts to recieve setpoint ({}) reached".format(self.recv_attempts))



if __name__ == '__main__':
    controller = Controller(5)
    controller.take_off()
    controller.run_posctl()
    controller.land()
    controller.tear_down()
