#!/usr/bin/env python

"""
Floor coverage test to be used within AWS RoboMaker
"""

import unittest
import time
from math import hypot, degrees, floor
from PIL import Image, ImageDraw

import rospy
import rostest
import tf
import numpy as np

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32
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
        self.footprint_in_map_frame = self.transform_footprint_to_map_frame()
        self.painting_map = self.map_array

    def runTest(self):
        """
        Start the coverage test
        """
        try:
            if not self.is_cancelled:
                rospy.loginfo(self.test_name)
                self.set_tag(name="Test_Start",
                                value=str(self.sim_start_time_secs))
                self.clock_sub = rospy.Subscriber("/clock", Clock, callback=self.timeout_cb)
                self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, callback=self.odom_cb)
                rospy.spin()
        except Exception as ex:
            rospy.logerr("Error %s", ex)
            self.set_tag(name=self.test_name, value="Failed")
            self.cancel_job()

    def odom_cb(self, msg):
        if not self.is_cancelled:

            # Convert Points in footprint from /base_footprint frame to /odom frame and add robot position (which
            # comes from /odom topic)
            # I now have footprint points in /odom frame. Convert to /map frame.
            # Get the indexes of the map array of said converted points
            # Use image.draw to draw. Rotation not included as of now

            #map_array_index = self.odom_to_map_array(msg)
            #if self.map_array[map_array_index[0]][map_array_index[1]] != self.COVERED_PGM_VALUE:
            #   self.painting_map[map_array_index[0]][map_array_index[1]] = self.COVERED_PGM_VALUE

            current_percentage_covered = self.calculate_current_percentage_covered()
            if current_percentage_covered >= self.coverage_threshold:
                rospy.loginfo("%s passed", self.test_name)
                self.set_tag(
                    name=self.test_name + "_Status", value="Passed")
                self.cancel_job()

    #def odom_to_map_array(self, position):
    #    """
    #    Converts an Odometry message in odom frame to map frame and returns the [x,y] indexes of its
    #    position in the map array.
    #    """
    #    # Getting an extrapolation error, even when asking if it canTransform
    #    if self.tl.canTransform(self.map_topic, self.odom_topic, rospy.Time(0)) and not self.is_cancelled:
    #        point_in_odom_frame = self.odometry_to_pose_stamped(position)
    #        point_in_map_frame = self.tl.transformPose(self.map_topic, point_in_odom_frame)
    #        return self.point_to_map_array(point_in_map_frame.pose.position.x,
    #                                       point_in_map_frame.pose.position.y)

    def point_to_map_array(self, point_x, point_y):
        """
        Given a point in map frame it returns the [x,y] indexes of it in the map array.
        """
        map_pos_in_array_x = floor((point_x / self.resolution) + self.map_origin_x)
        map_pos_in_array_y = floor((point_y / self.resolution) + self.map_origin_y)
        norm_map_pos_in_array_x = int(round(map_pos_in_array_x + self.map_height/2))
        norm_map_pos_in_array_y = int(round(map_pos_in_array_y + self.map_width/2))
        return [norm_map_pos_in_array_x, norm_map_pos_in_array_y]

    #def odometry_to_pose_stamped(self, msg):
    #    """
    #    Creates a PoseStamped message from an Odometry message.
    #    """
    #    pose_in_odom = PoseStamped()
    #    pose_in_odom.header = msg.header
    #    pose_in_odom.pose = msg.pose.pose
    #    return pose_in_odom

    def calculate_current_percentage_covered(self):
        free_painting_map_spaces = np.count_nonzero(self.painting_map == self.FREE_PGM_VALUE)
        return float(( (self.map_free_spaces - free_painting_map_spaces) / self.map_free_spaces ) * 100)

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
        rospy.loginfo("Canceling job ->")
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
        self.footprint = self.process_footprint(rospy.get_param("/footprint"))

    def get_map(self):
        """
        Save the raw map and convert it to a numpy array. Saves useful data.
        """
        try:
            raw_map = rospy.wait_for_message(self.map_topic, OccupancyGrid, timeout=5)
            self.map_array = np.array(raw_map.data)
            self.map_array = np.reshape(self.map_array, (raw_map.info.height, raw_map.info.width))
            self.map_height = raw_map.info.height
            self.map_width = raw_map.info.width
            self.resolution = raw_map.info.resolution
            self.map_origin_x = raw_map.info.origin.position.x
            self.map_origin_y = raw_map.info.origin.position.y
            self.map_free_spaces = np.count_nonzero(self.map_array == self.FREE_PGM_VALUE)
        except Exception as e:
            rospy.logerr("Error %s", e)
            self.set_tag(name=self.test_name, value="Failed")
            self.cancel_job()

    def get_time(self):
        """
        Returns current sim time in seconds
        """
        return rospy.Time.now().secs

    def process_footprint(self, footprint_dict):
        footprint_array = []
        for point in footprint_dict:
            pair = (point["x"], point["y"])
            footprint_array.append(pair)
        return footprint_array

    def transform_footprint_to_map_frame(self):
        return

    def print_map(self, map_array):
        map_array = map_array.astype(np.uint8)
        im = Image.fromarray(map_array)
        im.show()

if __name__ == "__main__":
    rospy.init_node("coverage_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "coverage_test", CoverageTest)
