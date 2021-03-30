#!/usr/bin/env python

"""
Floor coverage test to be used within AWS RoboMaker
"""

import unittest
import time
from math import floor, cos, sin
from PIL import Image, ImageDraw

import rospy
import rostest
import tf
import numpy as np

from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry, OccupancyGrid

from robomaker_simulation_msgs.msg import Tag
from robomaker_simulation_msgs.srv import Cancel, AddTags

class CoverageTest(unittest.TestCase):
    """
    Integration coverage test
    """

    TEST_NAME = "Coverage_Test"

    BLOCKED_PGM_VALUE = 100     # Wall value in pgm map.
    FREE_PGM_VALUE = 0          # Free known value in pgm map.
    UNKNOWN_PGM_VALUE = -1      # Unkown value in pgm map.

    def setUp(self):
        """
        Overloaded unittest method to set up test variables
        """
        self.test_name = self.TEST_NAME + "_" + str(time.time()).split(".")[0]
        self.is_cancelled = False
        while self.get_time() == 0:
            pass
        self.sim_start_time_secs = self.get_time()
        self.percentage_covered = 0
        self.tl = tf.TransformListener()
        self.get_params()
        self.get_map()
        self.painting_image = Image.fromarray(self.map_array.astype(np.uint8))
        self.map_free_spaces = len([px for px in list(self.painting_image.getdata()) if px == 255])

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
            # Odom frame to Map frame.
            pose_in_map_frame = self.odom_frame_to_map_frame(msg)

            # Get Odom msg in map frame current yaw angle, use that to rotate the footprint.
            yaw_angle = self.get_yaw_angle(pose_in_map_frame)
            rotated_footprint = self.rotate_footprint(yaw_angle)

            # Add the footprint to the Odom message.
            footprint_array = self.create_footprint_array(pose_in_map_frame, rotated_footprint)

            # Get the map array indexes corresponding to the footprint points
            footprint_map_indexes = self.get_map_indexes(footprint_array)

            # Paint the map
            draw = ImageDraw.Draw(self.painting_image)
            draw.polygon(footprint_map_indexes, fill=0)

            self.painting_image.save("/home/aws-cloudwatch/simulation_ws/test.pgm")

            current_percentage_covered = self.calculate_current_percentage_covered()

            if current_percentage_covered >= self.coverage_threshold:
                rospy.loginfo("%s passed", self.test_name)
                self.set_tag(
                    name=self.test_name + "_Status", value="Passed")
                self.cancel_job()

    def create_footprint_array(self, pose_in_map_frame, rotated_footprint):
        """
        Given a PoseStamped position in the map frame and the rotated footprint, it creates an array of
        x,y points made by adding the footprint to the position.
        """
        footprint_array = []
        for point in rotated_footprint:
            footprint_array.append((pose_in_map_frame.pose.position.x + point[0],
                                    pose_in_map_frame.pose.position.y + point[1]))
        return footprint_array

    def get_yaw_angle(self,msg):
        """
        Given a PosedStamped msg this function gets the yaw angle from it.
        """
        quaternion = (
                      msg.pose.orientation.x,
                      msg.pose.orientation.y,
                      msg.pose.orientation.z,
                      msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]

    def rotate_footprint(self, yaw_angle):
        """
        Rotates the footprint points given an angle in radians.
        """
        rotated_footprint = []
        c = cos(yaw_angle)
        s= sin(yaw_angle)
        for point in self.footprint:
            rotated_footprint.append((point[0]*c - point[1]*s,
                                      point[0]*s + point[1]*c ))
        return rotated_footprint

    def odom_frame_to_map_frame(self, msg):
        """
        Converts an Odometry message in odom frame to map frame.
        """
        point_in_odom_frame = self.odometry_to_pose_stamped(msg)
        #if self.tl.canTransform(self.map_topic, self.odom_topic, rospy.Time()):
        self.tl.waitForTransform(self.map_topic, self.odom_topic, rospy.Time(), rospy.Duration(300))
        point_in_map_frame = self.tl.transformPose(self.map_topic, point_in_odom_frame)
        return point_in_map_frame

    def odometry_to_pose_stamped(self, msg):
        """
        Creates a PoseStamped message from an Odometry message.
        """
        pose_in_odom = PoseStamped()
        pose_in_odom.header = msg.header
        pose_in_odom.pose = msg.pose.pose
        return pose_in_odom

    def get_map_indexes(self, odom_footprint_array):
        """
        Given an array of x,y points this function gets the indexes of said points from the map.
        """
        odom_footprint_map_indexes = []
        for point in odom_footprint_array:
            odom_footprint_map_indexes.append(self.point_to_map_array(point))
        return odom_footprint_map_indexes

    def point_to_map_array(self, point):
        """
        Given a point in map frame it returns the [x,y] indexes of it in the map array.
        """
        map_pos_in_array_x = floor((point[0] / self.resolution) + self.map_origin_x)
        map_pos_in_array_y = floor((point[1] / self.resolution) + self.map_origin_y)
        norm_map_pos_in_array_x = int(round(map_pos_in_array_x + self.map_height/2))
        norm_map_pos_in_array_y = int(round(map_pos_in_array_y + self.map_width/2))
        return (norm_map_pos_in_array_x, norm_map_pos_in_array_y)

    def calculate_current_percentage_covered(self):
        """
        Calculates the percentage of originally free spaces and the free spaces in the map being painted.
        """
        free_painting_map_spaces = len([px for px in list(self.painting_image.getdata()) if px == 255])
        return float(( float(self.map_free_spaces - free_painting_map_spaces) / self.map_free_spaces ) * 100)

    def timeout_cb(self, msg):
        """
        Cancel the test if the simulation times out
        Timeouts are based on simulation time, published on /clock topic
        """
        if (msg.clock.secs - self.sim_start_time_secs) > self.sim_timeout and \
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
            self.format_map_colors()
            self.map_height = raw_map.info.height
            self.map_width = raw_map.info.width
            self.resolution = raw_map.info.resolution
            self.map_origin_x = raw_map.info.origin.position.x
            self.map_origin_y = raw_map.info.origin.position.y
        except Exception as e:
            rospy.logerr("Error %s", e)
            self.set_tag(name=self.test_name, value="Failed")
            self.cancel_job()

    def get_time(self):
        """
        Returns current sim time in seconds
        """
        return rospy.Time.now().secs

    def format_map_colors(self):
        """
        Formats the map array with the correct values for the PIL module.
        """
        self.map_array[self.map_array == self.UNKNOWN_PGM_VALUE] = 150
        self.map_array[self.map_array == self.FREE_PGM_VALUE] = 255
        self.map_array[self.map_array == self.BLOCKED_PGM_VALUE] = 0

    def process_footprint(self, footprint_dict):
        """
        Generates an array of x,y points from the read footprint.yaml file
        """
        footprint_array = []
        for point in footprint_dict:
            pair = (point["x"], point["y"])
            footprint_array.append(pair)
        return footprint_array

if __name__ == "__main__":
    rospy.init_node("coverage_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "coverage_test", CoverageTest)
