#!/usr/bin/env python

"""
Floor coverage test to be used within AWS RoboMaker
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
    Integration coverage test

    """

    TEST_NAME = "Coverage_Test"

    def setUp(self):
        """
        Overloaded unittest method to set up test variables
        """
        self.test_name = self.TEST_NAME + "_" + str(time.time()).split(".")[0]
        rospy.loginfo("Test Name: %s", self.test_name)
        self.is_cancelled = False
        while self.get_time() == 0:
            pass
        self.sim_start_time_secs = self.get_time()
        self.coverage_start_time = self.get_time()
        self.percentage_covered = 0
        self.get_params()

    def runTest(self):
        """
        Start the coverage test
        """
        try:
            rospy.loginfo(self.test_name)
            self.set_tag(name="Test_Start",
                         value=str(self.sim_start_time_secs))
            rospy.Subscriber("/clock", Clock, self.timeout_cb)
            # TO DO

            rospy.spin()
        except Exception as ex:
            rospy.logerr("Error %s", ex)
            self.set_tag(name=self.test_name, value="Failed")
            self.cancel_job()


    def odom_cb(self, msg):
        # TO DO
        return

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

    def get_time(self):
        """
        Returns current sim time in seconds
        """
        return rospy.Time.now().secs


if __name__ == "__main__":
    rospy.init_node("coverage_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "coverage_test", CoverageTest)
