#!/usr/bin/env python

"""
Floor coverage test to be used within AWS RoboMaker
"""

import unittest
import time
from math import hypot, degrees, floor
from PIL import Image

import rospy
import rostest
import tf
import numpy as np

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry, OccupancyGrid
from std_srvs.srv import Trigger, TriggerRequest

from robomaker_simulation_msgs.msg import Tag
from robomaker_simulation_msgs.srv import Cancel, AddTags


class CoverageTest(unittest.TestCase):
    """
    Integration coverage test
    """

    TEST_NAME = "Coverage_Test"

    #Here or via param/user input?
    COVERED_PGM_VALUE = 100   # Wall value in pgm map.
    FREE_PGM_VALUE = 0 # Free known value in pgm map.
    UNKNOWN_PGM_VALUE = -1 #Unkown value in pgm map.

    def setUp(self):
        """
        Overloaded unittest method to set up test variables
        """
        self.test_name = self.TEST_NAME + "_" + str(time.time()).split(".")[0]
        self.is_cancelled = False
        while self.get_time() == 0:
            pass
        self.sim_start_time_secs = self.get_time()
        self.coverage_start_time = self.get_time()  # For later usage, now it does the same as sim time
        self.percentage_covered = 0
        self.tl = tf.TransformListener()
        self.get_params()
        self.get_map()
        self.painting_map = self.map_array

    def runTest(self):
        """
        Start the coverage test
        """
        try:
            rospy.loginfo(self.test_name)
            self.set_tag(name="Test_Start",
                         value=str(self.sim_start_time_secs))
            rospy.Subscriber("/clock", Clock, callback=self.timeout_cb)
            rospy.Subscriber(self.odom_topic, Odometry, callback=self.odom_cb)

            rospy.spin()
        except Exception as ex:
            rospy.logerr("Error %s", ex)
            self.set_tag(name=self.test_name, value="Failed")
            self.cancel_job()


    def odom_cb(self, msg):
        pose_in_odom = self.odometry_to_pose_stamped(msg)

        # Getting an extrapolation error, even when asking if it canTransform
        if self.tl.canTransform(self.map_topic, self.odom_topic, rospy.Time(0)):
            pose_in_map = self.tl.transformPose(self.map_topic, pose_in_odom)

            map_pos_in_array_x = floor((pose_in_map.pose.position.x / self.resolution) + self.map_origin_x)
            map_pos_in_array_y = floor((pose_in_map.pose.position.y / self.resolution) + self.map_origin_y)
            norm_map_pos_in_array_x = int(round(map_pos_in_array_x + self.map_height/2))
            norm_map_pos_in_array_y = int(round(map_pos_in_array_y + self.map_width/2))

            if self.map_array[norm_map_pos_in_array_x][norm_map_pos_in_array_y] != self.COVERED_PGM_VALUE:
                self.painting_map[norm_map_pos_in_array_x][norm_map_pos_in_array_y] = self.COVERED_PGM_VALUE

            rospy.loginfo ("Original free spaces -> %s", self.map_free_spaces)
            rospy.loginfo ("Current free spaces -> %s", np.count_nonzero(self.painting_map == self.FREE_PGM_VALUE))
            current_percentage_covered = float((np.count_nonzero(self.painting_map == self.FREE_PGM_VALUE) / self.map_free_spaces ) * 100)
            rospy.loginfo(current_percentage_covered)

    def odometry_to_pose_stamped(self, msg):
        pose_in_odom = PoseStamped()
        pose_in_odom.header = msg.header
        pose_in_odom.pose = msg.pose.pose
        return pose_in_odom

    def get_map(self):
        """
        Save the raw map and convert it to a numpy array. Saves useful data.
        """
        self.raw_map = rospy.wait_for_message(self.map_topic, OccupancyGrid, timeout=5)
        self.map_array = np.array(self.raw_map.data)

        self.map_height = self.raw_map.info.height
        self.map_width = self.raw_map.info.width
        self.resolution = self.raw_map.info.resolution
        self.map_origin_x = self.raw_map.info.origin.position.x
        self.map_origin_y = self.raw_map.info.origin.position.y
        self.map_free_spaces = np.count_nonzero(self.map_array == self.FREE_PGM_VALUE)

        self.map_array = np.reshape(self.map_array, (self.map_height, self.map_width))

    def timeout_cb(self, msg):
        """
        Cancel the test if the simulation or the coverage process time out
        Timeouts are based on simulation time, published on /clock topic
        """
        if (msg.clock.secs - self.coverage_start_time) > self.coverage_timeout and \
                not self.is_cancelled:
            rospy.loginfo("Coverage test timed out, cancelling job")
            self.set_tag(name=self.test_name + "_Status", value="Failed")
            self.set_tag(name="Coverage_Timed_Out", value=str(True))
            self.cancel_job()
        elif (msg.clock.secs - self.sim_start_time_secs) > self.sim_timeout and \
                not self.is_cancelled:
            rospy.loginfo("Test timed out, cancelling job")
            self.set_tag(name=self.test_name + "_Status", value="Failed")
            self.set_tag(name="Simulation_Timed_Out", value=str(True))
            self.cancel_job()

    def cancel_job(self):
        """
        Cancels current simulation job using AWS RoboMaker service
        """
        rospy.loginfo("Canceling job")
        self.is_cancelled = True
        return
        rospy.wait_for_service("/robomaker/job/cancel")
        requestCancel = rospy.ServiceProxy("/robomaker/job/cancel", Cancel)
        response = requestCancel()
        if response.success:
            self.is_cancelled = True
            rospy.loginfo("Successfully requested cancel job")
            self.set_tag(name=self.test_name + "_Time_Elapsed_End",
                         value=str(self.get_time()))
        else:
            rospy.logerr("Cancel request failed: %s", response.message)

    def set_tag(self, name, value):
        """
        Sets job tags using AWS RoboMaker service
        """
        rospy.loginfo("Setting tag: %s -> %s", name, value)
        return
        rospy.wait_for_service("/robomaker/job/add_tags")
        requestAddTags = rospy.ServiceProxy("/robomaker/job/add_tags", AddTags)
        tags = ([Tag(key=name, value=value)])
        response = requestAddTags(tags)
        if response.success:
            rospy.loginfo("Successfully added tags: %s", tags)
        else:
            rospy.logerr(
                "Add tags request failed for tags (%s): %s",
                tags,
                response.message)

    def get_params(self):
        """
        Loads configuration parameters for the coverage test
        """
        self.coverage_threshold = rospy.get_param("coverage_threshold")
        self.sim_timeout = rospy.get_param("sim_timeout")
        self.coverage_timeout = rospy.get_param("coverage_timeout")
        self.map_topic = rospy.get_param("map_topic")
        self.odom_topic = rospy.get_param("odom_topic")

    def get_time(self):
        """
        Returns current sim time in seconds
        """
        return rospy.Time.now().secs


if __name__ == "__main__":
    rospy.init_node("coverage_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "coverage_test", CoverageTest)
