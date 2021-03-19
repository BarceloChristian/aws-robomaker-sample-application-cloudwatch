#!/usr/bin/env python

import rospy
import rostest
import time
import os
import unittest
import tf

from math import hypot
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerRequest

from robomaker_simulation_msgs.msg import Tag
from robomaker_simulation_msgs.srv import Cancel, AddTags

class DockingTest(unittest.TestCase):

    def runTest(self):
        # Start the docking test
        self.test_docking()

    def setUp(self):
        self.test_name = "Docking_Test_" + \
            str(self.get_time())
        rospy.loginfo("Test Name: %s", self.test_name)
        self.is_docked = False
        self.is_docking = False
        self.is_cancelled = False
        self.get_params()
        while rospy.Time.now().secs == 0:
            pass
        self.sim_start_time_secs = rospy.Time.now().secs
        self.dock_start_time_secs = 0 # Initialize in 0 because it is used in timeout_cb method
        self.trigger = rospy.Timer(rospy.Duration(self.wait_time_before_docking), callback=self.trigger_docking_client, oneshot=True)

    def trigger_docking_client(self, _):
        try:
            rospy.loginfo("TRIGGER_DOCKING:CLIENT") ##############################################
            rospy.wait_for_service('trigger_docking_service', timeout=5)
            client_service_trigger = rospy.ServiceProxy('trigger_docking_service', Trigger)
            trigger = TriggerRequest()
            response = client_service_trigger(trigger)
            self.set_tag(name="Docking_Process_Start", value=str(self.get_time()) )
            self.is_docking = True
            self.dock_start_time_secs = rospy.Time.now().secs
            return response.success
        except rospy.ServiceException as e:
            self.set_tag(name=self.test_name + "_Failed", value=str(self.get_time()) )
            self.cancel_job()
            rospy.logerr("Service call failed: %s"%e)

    def test_docking(self):
        try:
            rospy.loginfo(self.test_name)
            self.set_tag(name="Time_Elapsed_Start",
                         value=str(self.sim_start_time_secs))
            if self.docked_topic_enabled is True:
                rospy.loginfo("%s topic listening enabled.", self.docked_topic_name)
                rospy.Subscriber(self.docked_topic_name, Bool, self.is_docked_cb)
            elif self.odom_topic_enabled is True:
                rospy.loginfo("%s topic listening enabled.", self.odom_topic_name)
                rospy.Subscriber(self.odom_topic_name, Odometry, self.odom_cb)
            else:
                rospy.logerr("No topics are enabled")
                self.set_tag(name="Status", value="Failed")
                self.set_tag(name="No_Topics_Enabled", value=str(self.get_time()))
                self.cancel_job()
            rospy.Subscriber("/clock", Clock, self.timeout_cb)
            rospy.spin()
        except Exception as e:
            rospy.logerr("Error %s", e)
            self.set_tag(name=self.test_name, value="Failed")
            self.cancel_job()

    def is_docked_cb(self, msg):
        """
        Cancel the test if "True" is published in the <docked_topic>.
        """
        if not self.is_docking or self.is_docked:
            return
        if msg.data is True and not self.is_docked and not self.is_cancelled:
            rospy.loginfo("Docked")
            self.set_tag(name="Status", value="Passed")
            self.set_tag(name="Docked", value=str(self.get_time()) )
            self.is_docked = True
            self.cancel_job()

    def timeout_cb(self, msg):
        """
        Cancel the test if it times out.
        Cancel the test if the docking process times out.
        The sim_timeout is based on the /clock topic (simulation time).
        """
        if (msg.clock.secs - self.dock_start_time_secs) > self.dock_timeout and self.is_docking and not self.is_cancelled:
            rospy.loginfo("Docking timed out, cancelling job")
            self.set_tag(name="Status", value="Failed")
            self.set_tag(name="Docking_Timed_Out", value=str(True))
            self.cancel_job()
        if (msg.clock.secs - self.sim_start_time_secs) > self.sim_timeout and not self.is_cancelled:
            rospy.loginfo("Test timed out, cancelling job")
            self.set_tag(name="Status", value="Failed")
            self.set_tag(name="Simulation_Timed_Out", value=str(True))
            self.cancel_job()

    def odom_cb(self, msg):
        """
        Cancel the test if robot is within threshold distance of the dock.
        The distance is based on the <odom_topic>.
        """
        if not self.is_docking or self.is_docked:
            return
        if self.distance_to_dock(msg) < self.threshold and not self.is_docked and not self.is_cancelled:
            rospy.loginfo("Docked")
            self.set_tag(name="Status", value="Passed")
            self.set_tag(name="Docked", value=str(self.get_time()) )
            self.is_docked = True
            self.cancel_job()
        return

    def cancel_job(self):
        rospy.loginfo("Canceling job ->")
        return
        rospy.wait_for_service("/robomaker/job/cancel")
        requestCancel = rospy.ServiceProxy("/robomaker/job/cancel", Cancel)
        response = requestCancel()
        if response.success:
            self.is_cancelled = True
            rospy.loginfo("Successfully requested cancel job")
            self.set_tag(name=self.test_name + "_Time_Elapsed_End", value=str(self.get_time()) )
        else:
            rospy.logerr("Cancel request failed: %s", response.message)

    def set_tag(self, name, value):
        rospy.loginfo("Setting tag -> %s -> %s", name, value)
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

    def distance_to_dock(self, msg):
        self.ros_tf = tf.TransformerROS()
        p = PoseStamped()
        p.pose.position.x = msg.pose.pose.position.x
        p.pose.position.y = msg.pose.pose.position.y
        p.header.frame_id = "odom"
        pose_in_map = self.ros_tf.transformPose("world", p)
        position_in_map = pose_in_map.pose.position
        return hypot(self.dock_x - position_in_map.x, self.dock_y - position_in_map.y)

    def get_params(self):
        self.docked_topic_enabled = rospy.get_param("enable_is_docked_topic")
        if self.docked_topic_enabled is True:
            self.docked_topic_name = rospy.get_param("docked_topic_name")

        self.odom_topic_enabled = rospy.get_param("enable_odom_topic")
        if self.odom_topic_enabled is True:
            self.threshold = rospy.get_param("dock_threshold_m")
            self.dock_x = rospy.get_param("dock_x")
            self.dock_y = rospy.get_param("dock_y")
            self.odom_topic_name = rospy.get_param("odom_topic_name")

        self.sim_timeout = rospy.get_param("sim_timeout_sec")
        self.dock_timeout = rospy.get_param("dock_timeout_sec")
        self.wait_time_before_docking = rospy.get_param("wait_time_before_docking")

    def get_time(self):
        return rospy.Time.now().secs

if __name__ == "__main__":
    rospy.init_node("docking_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "docking_test", DockingTest)
