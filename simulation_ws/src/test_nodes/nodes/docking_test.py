#!/usr/bin/env python

"""
Integration docking test to be used within AWS RoboMaker
"""

import unittest
import time
from math import hypot, degrees

import rospy
import rostest
import tf

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerRequest

from robomaker_simulation_msgs.msg import Tag
from robomaker_simulation_msgs.srv import Cancel, AddTags


class DockingTest(unittest.TestCase):
    """
    Integration docking test
    It lets the robot move around during a customizable amount of time,
        before triggering a service on <TRIGGER_DOCKING_SERVICE> to send the robot to dock
    Once the robot is requested to dock, there are two optional strategies that can be used
        to tag the test as 'Passed'.
        1- The robot can publish a Bool(True) on the <DOCKED_TOPIC> (which can be remapped).
        2- If the dock is implemented as a spot in the map, the test must receive the dock position
            and orientation (in 'map' frame), and the succeed condition will be the robot arriving
            to that spot within a customizable threshold (in meters and degrees). Robot possition
            is read from <ODOM_TOPIC> (which can be remapped)
    """

    TEST_NAME = "Docking_Test"
    DOCKED_TOPIC = "/docked"
    ODOM_TOPIC = "/odom"
    TRIGGER_DOCKING_SERVICE = "trigger_docking_service"

    def setUp(self):
        """
        Overloaded unittest method to set up test variables
        """
        self.test_name = self.TEST_NAME + "_" + str(time.time()).split(".")[0]
        rospy.loginfo("Test Name: %s", self.test_name)
        self.is_docked = False
        self.is_docking = False
        self.is_cancelled = False
        while rospy.Time.now().secs == 0:
            pass
        self.sim_start_time_secs = rospy.Time.now().secs
        # Initialize in 0 because it is used in timeout_cb method
        self.dock_start_time = 0
        self.get_params()
        self.trigger = rospy.Timer(
            rospy.Duration(self.wait_time_before_docking),
            callback=self.trigger_docking_client, oneshot=True)

    def runTest(self):
        """
        Start the docking test
        """
        try:
            rospy.loginfo(self.test_name)
            self.set_tag(name="Test_Start",
                         value=str(self.sim_start_time_secs))
            if self.use_docked_topic is True:
                rospy.loginfo("%s topic listening enabled.", self.DOCKED_TOPIC)
                rospy.Subscriber(self.DOCKED_TOPIC, Bool, self.docked_cb)
            elif self.use_odom_topic is True:
                rospy.loginfo("%s topic listening enabled.", self.ODOM_TOPIC)
                rospy.Subscriber(self.ODOM_TOPIC, Odometry, self.odom_cb)
            else:
                rospy.logerr(
                    "Neither 'USE_DOCKED_TOPIC' nor 'USE_ODOM_TOPIC' are set")
                self.set_tag(name=self.test_name + "_Status", value="Failed")
                self.set_tag(name="No_Topics_Enabled",
                             value=str(self.get_time()))
                self.cancel_job()
            rospy.Subscriber("/clock", Clock, self.timeout_cb)
            rospy.spin()
        except Exception as ex:
            rospy.logerr("Error %s", ex)
            self.set_tag(name=self.test_name, value="Failed")
            self.cancel_job()

    def trigger_docking_client(self, _):
        """
        Triggers docking service on the robot
        """
        try:
            rospy.wait_for_service(self.TRIGGER_DOCKING_SERVICE, timeout=5)
            client_service_trigger = rospy.ServiceProxy(
                self.TRIGGER_DOCKING_SERVICE, Trigger)
            response = client_service_trigger(TriggerRequest())
            if response.success is True:
                self.set_tag(name="Docking_Process_Start",
                             value=str(self.get_time()))
                self.is_docking = True
                self.dock_start_time = rospy.Time.now().secs
            else:
                raise rospy.ServiceException("Robot docking service failed")
        except rospy.ServiceException as ex:
            self.set_tag(name=self.test_name + "_Failed",
                         value=str(self.get_time()))
            self.cancel_job()
            rospy.logerr("Service call failed: %s" % ex)

    def docked_cb(self, msg):
        """
        Finishes the test if "True" is published in the /docked topic
        """
        if not self.is_docking or self.is_docked:
            return
        if msg.data is True and not self.is_docked and not self.is_cancelled:
            rospy.loginfo("Robot docked at %s", str(self.get_time()))
            self.set_tag(name=self.test_name + "_Status", value="Passed")
            self.set_tag(name="Docked", value=str(self.get_time()))
            self.is_docked = True
            self.cancel_job()

    def odom_cb(self, msg):
        """
        Finishes successfully the test if robot is within threshold distance of the dock
        """
        if not self.is_docking or self.is_docked:
            return
        translational_dist, rotational_dist = self.distance_to_dock(msg)
        rospy.loginfo("(%s, %s)", str(translational_dist),
                      str(rotational_dist))
        if translational_dist < self.threshold and \
                rotational_dist <= self.rotational_threshold and \
                not self.is_docked and not self.is_cancelled:
            rospy.loginfo("Robot docked at %s", str(self.get_time()))
            self.set_tag(name=self.test_name + "_Status", value="Passed")
            self.set_tag(name="Docked", value=str(self.get_time()))
            self.is_docked = True
            self.cancel_job()

    def distance_to_dock(self, msg):
        """
        Returns (translational, rotational) distance from robot to dock in map frame
        """
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        pose_in_map = self.listener.transformPose("map", pose_stamped)
        position_in_map = pose_in_map.pose.position
        translational = hypot(self.dock_x - position_in_map.x,
                              self.dock_y - position_in_map.y)
        rotational = 0
        if self.dock_yaw != 0:
            orientation = pose_in_map.pose.orientation
            _, _, yaw = tf.transformations.euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w])
            rotational = abs(self.dock_yaw - degrees(yaw))
        return translational, rotational

    def timeout_cb(self, msg):
        """
        Cancel the test if the simulation or the docking process time out
        Timeouts are based on simulation time, published on /clock topic
        """
        if (msg.clock.secs - self.dock_start_time) > self.docking_timeout and \
                self.is_docking and not self.is_cancelled:
            rospy.loginfo("Docking timed out, cancelling job")
            self.set_tag(name=self.test_name + "_Status", value="Failed")
            self.set_tag(name="Docking_Timed_Out", value=str(True))
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
        Loads configuration parameters to set docking condition strategy
        """
        self.use_docked_topic = rospy.get_param("use_docked_topic")
        self.use_odom_topic = rospy.get_param("use_odom_topic")
        if self.use_odom_topic is True:
            self.threshold = rospy.get_param("dock_threshold")
            self.rotational_threshold = rospy.get_param(
                "dock_rotational_threshold")
            self.dock_x = rospy.get_param("dock_x")
            self.dock_y = rospy.get_param("dock_y")
            self.dock_yaw = rospy.get_param("dock_yaw")
            # Ros transformer to transform odom -> map
            self.listener = tf.TransformListener()

        self.sim_timeout = rospy.get_param("sim_timeout")
        self.docking_timeout = rospy.get_param("docking_timeout")
        self.wait_time_before_docking = rospy.get_param("wait_before_docking")

    def get_time(self):
        """
        Returns current sim time in seconds
        """
        return rospy.Time.now().secs


if __name__ == "__main__":
    rospy.init_node("docking_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "docking_test", DockingTest)
