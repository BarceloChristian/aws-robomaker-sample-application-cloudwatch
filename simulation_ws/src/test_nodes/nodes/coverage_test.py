#!/usr/bin/env python

"""
Floor coverage test to be used within AWS RoboMaker
"""

import unittest
import time
from math import floor, cos, sin
from PIL import Image, ImageDraw, ImageOps
from os import path

import rospy
import rostest
import tf
import numpy as np
import yaml

from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry, OccupancyGrid

from robomaker_simulation_msgs.msg import Tag
from robomaker_simulation_msgs.srv import Cancel, AddTags


class Map(object):
    OBSTACLE = 100
    FREE_AREA = 0
    UNKNOWN_AREA = -1

    IMAGE_OBSTACLE = 0
    IMAGE_FREE_AREA = 255
    IMAGE_UNKNOWN = 150

    PATH_COLOR = (240, 41, 240)

    def __init__(self, map_2d, width, height, resolution, origin_x, origin_y):
        self.map = map_2d
        self.map_width = width
        self.map_height = height
        self.resolution = resolution
        self.map_origin_x = origin_x
        self.map_origin_y = origin_y
        self.last_robot_poses = None

    def format_map(self):
        """
        Formats the map array with the correct values for the PIL module.
        """
        self.map[self.map == self.UNKNOWN_AREA] = self.IMAGE_UNKNOWN
        self.map[self.map == self.FREE_AREA] = self.IMAGE_FREE_AREA
        self.map[self.map == self.OBSTACLE] = self.IMAGE_OBSTACLE

    def format_image(self):
        """
        Formats the image with the correct values for the PIL module.
        """
        self.map[self.map == 205] = self.IMAGE_UNKNOWN
        self.map[self.map == 254] = self.IMAGE_FREE_AREA
        self.map[self.map == 0] = self.IMAGE_OBSTACLE

    def map_to_index(self, point):
        """
        Given a point in map frame it returns the [x,y] indexes of it in the map array.
        """
        map_pos_in_array_x = floor(
            (point[0] / self.resolution) + self.map_origin_x)
        map_pos_in_array_y = floor(
            (point[1] / self.resolution) + self.map_origin_y)
        norm_map_pos_in_array_x = int(
            round(map_pos_in_array_x + self.map_height/2))
        norm_map_pos_in_array_y = int(
            round(map_pos_in_array_y + self.map_width/2))
        return (norm_map_pos_in_array_x, norm_map_pos_in_array_y)

    def init_coverage_map(self):
        initial_map = Image.fromarray(self.map.astype(np.uint8))
        self.coverage_map = initial_map.copy()
        self.path_map = initial_map.convert('RGB')
        self.initial_free_cells = len(
            [px for px in list(initial_map.getdata()) if px == self.IMAGE_FREE_AREA])
        #rospy.loginfo(self.path_map.getcolors())

    def add_coverage(self, robot_pose, footprint):
        robot_pose_index = self.map_to_index((robot_pose.x, robot_pose.y))
        if self.last_robot_poses is not None:
            last_robot_pose_index = self.map_to_index((self.last_robot_poses.x,
                                                       self.last_robot_poses.y))
            draw = ImageDraw.Draw(self.path_map)
            draw.line([robot_pose_index, last_robot_pose_index], fill=self.PATH_COLOR, width=2)
        self.last_robot_poses = robot_pose
        draw = ImageDraw.Draw(self.coverage_map)
        if type(footprint) is list:
            footprint_map_indexes = []
            for point in footprint:
                footprint_map_indexes.append(
                    self.map_to_index(point))
            draw.polygon(footprint_map_indexes, fill=0)
        else:  # Assume circular footprint
            radius = int(round(footprint / self.resolution))
            draw.ellipse(robot_pose_index[0] - radius, robot_pose_index[1] - radius,
                         robot_pose_index[0] + radius, robot_pose_index[1] + radius)

    def calculate_coverage(self):
        """
        Calculates the percentage of originally free spaces and the free spaces in the map being painted.
        """
        free_painting_map_spaces = len(
            [px for px in list(self.coverage_map.getdata()) if px == self.IMAGE_FREE_AREA])
        return (float(self.initial_free_cells - free_painting_map_spaces) / self.initial_free_cells) * 100

    def get_coverage_image(self):
        return self.coverage_map

    def get_path_image(self):
        return self.path_map

    @staticmethod
    def FromOccupancyGrid(occ_grid):
        map_array = np.array(occ_grid.data)
        map_array = np.reshape(
            map_array, (occ_grid.info.height, occ_grid.info.width))
        map_width = occ_grid.info.width
        map_height = occ_grid.info.height
        resolution = occ_grid.info.resolution
        map_origin_x = occ_grid.info.origin.position.x
        map_origin_y = occ_grid.info.origin.position.y
        map_2d = Map(map_array, map_width, map_height,
                     resolution, map_origin_x, map_origin_y)
        map_2d.format_map()
        map_2d.init_coverage_map()
        return map_2d

    @staticmethod
    def FromImageConfigPath(image_config_path):
        with open(image_config_path, "r") as file:
            map_config = yaml.load(file)
        # Assume map config file and image are in the same directory
        map_image_path = path.join(path.dirname(image_config_path),
                                   map_config["image"])
        map_image = Image.open(map_image_path)
        map_width, map_height = map_image.size
        map_array = np.array(map_image)
        resolution = map_config["resolution"]
        map_origin = map_config["origin"]
        map_2d = Map(map_array, map_width, map_height,
                     resolution, map_origin[0], map_origin[1])
        map_2d.format_image()
        map_2d.init_coverage_map()
        return map_2d

class CoverageTest(unittest.TestCase):
    """
    Integration coverage test
    """
    TEST_NAME = "Coverage_Test"

    MAP_TOPIC = "/map"
    ODOM_TOPIC = "/odom"

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
        self.tf_listener = tf.TransformListener()
        self.get_params()

    def runTest(self):
        """
        Start the coverage test
        """
        try:
            if not self.is_cancelled:
                rospy.loginfo(self.test_name)
                self.set_tag(
                    name="Test_Start", value=str(self.sim_start_time_secs))
                self.clock_sub = rospy.Subscriber(
                    "/clock", Clock, callback=self.timeout_cb)
                self.odom_sub = rospy.Subscriber(
                    self.ODOM_TOPIC, Odometry, callback=self.odom_cb)
                rospy.spin()
        except Exception as ex:
            rospy.logerr("Error %s", ex)
            self.set_tag(name=self.test_name, value="Failed")
            self.cancel_job()

    def odom_cb(self, msg):
        if not self.is_cancelled:
            # Odom frame to Map frame.
            pose_in_map_frame = self.odom_frame_to_map_frame(msg)
            # Get robot's orientation in map frame
            yaw = self.get_orientation(pose_in_map_frame)
            # Rotates footprint
            rotated_footprint = self.rotate_footprint(yaw)
            # Translates footprint to map frame
            footprint_array = self.footprint_to_map(
                pose_in_map_frame, rotated_footprint)

            self.map.add_coverage(
                pose_in_map_frame.pose.position, footprint_array)

            if self.map.calculate_coverage() >= self.coverage_threshold:
                rospy.loginfo("%s passed", self.test_name)
                self.set_tag(
                    name=self.test_name + "_Status", value="Passed")
                self.cancel_job()

    def footprint_to_map(self, pose_in_map_frame, rotated_footprint):
        """
        Given a PoseStamped position in the map frame and the rotated footprint, it creates an array of
        x,y points made by adding the footprint to the position.
        """
        footprint_array = []
        for point in rotated_footprint:
            footprint_array.append((pose_in_map_frame.pose.position.x + point[0],
                                    pose_in_map_frame.pose.position.y + point[1]))
        return footprint_array

    def get_orientation(self, msg):
        """
        Given a PosedStamped msg this function gets the yaw angle from it.
        """
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return yaw

    def rotate_footprint(self, yaw):
        """
        Rotates the footprint points given an angle in radians.
        """
        rotated_footprint = []
        c = cos(yaw)
        s = sin(yaw)
        for point in self.footprint:
            rotated_footprint.append((point[0] * c - point[1] * s,
                                      point[0] * s + point[1] * c))
        return rotated_footprint

    def odom_frame_to_map_frame(self, msg):
        """
        Converts an Odometry message from odom to map frame.
        """
        point_in_odom_frame = self.odometry_to_pose_stamped(msg)
        self.tf_listener.waitForTransform(
            self.MAP_TOPIC, self.ODOM_TOPIC, rospy.Time(), rospy.Duration(1.))
        point_in_map_frame = self.tf_listener.transformPose(
            self.MAP_TOPIC, point_in_odom_frame)
        return point_in_map_frame

    def odometry_to_pose_stamped(self, msg):
        """
        Creates a PoseStamped message from an Odometry message.
        """
        pose_in_odom = PoseStamped()
        pose_in_odom.header = msg.header
        pose_in_odom.pose = msg.pose.pose
        return pose_in_odom

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
        # Save robot route image before canceling
        if self.output_map_image_path:
            self.map.get_path_image().save(self.output_map_image_path)
        # TODO: REMOVE
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
                tags, response.message)

    def get_params(self):
        """
        Loads configuration parameters for the coverage test
        """
        self.output_map_image_path = rospy.get_param("output_image_path")
        self.coverage_threshold = rospy.get_param("coverage_threshold")
        if 0 >= self.coverage_threshold > 100:
            self.set_tag(name=self.test_name + "_Status", value="Failed")
            self.set_tag(name="Invalid_Threshold",
                         value=str(self.coverage_threshold))
            rospy.logerr("[coverage_threshold] should be a value in the range (0, 100]. Received %s",
                         self.coverage_threshold)
            self.cancel_job()
        self.sim_timeout = rospy.get_param("sim_timeout")
        if rospy.get_param("use_map_topic"):
            self.load_occupancy_grid()
        elif rospy.get_param("load_map_file"):
            self.load_map_from_file()
        else:
            self.set_tag(name=self.test_name + "_Status", value="Failed")
            self.set_tag(name="Map_Source_Not_Provided", value=str(True))
            rospy.logerr(
                "Either [use_map_topic] or [load_map_file] should be set")
            self.cancel_job()
        self.footprint = self.get_footprint()

    def load_occupancy_grid(self):
        try:
            occupancy_grid = rospy.wait_for_message(
                self.MAP_TOPIC, OccupancyGrid, timeout=5)
            self.map = Map.FromOccupancyGrid(occupancy_grid)
        except Exception as e:
            rospy.logerr("Error %s", e)
            self.set_tag(name=self.test_name, value="Failed")
            self.cancel_job()

    def load_map_from_file(self):
        self.map = Map.FromImageConfigPath(rospy.get_param("map_config_path"))

    def get_time(self):
        """
        Returns current sim time in seconds
        """
        return rospy.Time.now().secs

    def get_footprint(self):
        """
        Generates an array of x,y points from the read footprint.yaml file
        """
        footprint_type = rospy.get_param("/footprint_type")
        if footprint_type == "polygon":
            footprint_array = []
            if rospy.has_param("/polygon") and len(rospy.get_param("/polygon")) > 2:
                for point in rospy.get_param("/polygon"):
                    footprint_array.append((point["x"], point["y"]))
                return footprint_array
            else:
                self.set_tag(name=self.test_name + "_Status", value="Failed")
                self.set_tag(
                    name="Empty_Footprint_Polygon_Points", value=str(True))
                rospy.logerr(
                    "[polygon] not set or empty for a polygonal footprint")
                self.cancel_job()
        elif footprint_type == "circular":
            if rospy.has_param("/radius"):
                return rospy.get_param("/radius")
            else:
                self.set_tag(name=self.test_name + "_Status", value="Failed")
                self.set_tag(name="Empty_Footprint_Radius", value=str(True))
                rospy.logerr("[radius] not set for a circular footprint")
                self.cancel_job()
        else:
            self.set_tag(name=self.test_name + "_Status", value="Failed")
            self.set_tag(name="Wrong_Footprint_Type", value=footprint_type)
            rospy.logerr(
                "[footprint_type] should be set either as 'polygon' or 'circular'")
            self.cancel_job()

if __name__ == "__main__":
    rospy.init_node("coverage_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "coverage_test", CoverageTest)
