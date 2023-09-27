#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_system_default

class JointStateManager(Node):
    def __init__(self):
        super().__init__("joint_state_manager_node")

        self.msg_name = []
        self.msg_position = []

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            "joint_states",
            self.callback,
            qos_profile_system_default)

    def callback(self, msg):
        # self.msg_name = msg.name
        self.msg_position = msg.position

    def get_joint_state(self):
        rclpy.spin_once(self)
        return self.msg_position

