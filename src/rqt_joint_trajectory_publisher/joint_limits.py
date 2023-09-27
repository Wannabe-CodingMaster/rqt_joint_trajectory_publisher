#!/usr/bin/env python

# Copyright 2022 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

import xml.etree.ElementTree as ET
from math import pi
from std_msgs.msg import String
from rclpy.qos import qos_profile_action_status_default

class JointLimitManager(Node):
    def __init__(self):
        self.description = ""
        self.subscription_completed = False

        super().__init__("joint_limit_manager_node")
        self.joint_limit_subscriber = self.create_subscription(String, "/robot_description", self.callback, qos_profile_action_status_default)

    def callback(self, msg):
        self.description = msg.data
        self.subscription_completed = True

    def get_joint_limits(self, use_safety_controller=True, use_mimic=True):
        joint_names = []
        free_joints = {}
        dependent_joints = {}

        while not self.subscription_completed:
            rclpy.spin_once(self)

        if self.description != "":

            robot = ET.fromstring(self.description)

            # get limits of each joint
            for child in robot.findall("joint"):
                joint = {}
                type = child.get("type")
                if type == "fixed":
                    continue

                name = child.get("name")
                limit = child.find("limit")
                if limit is None:
                    self.get_logger().warning("No limit tag: {}".format(name))
                    continue
                else:
                    if type == "continuous":
                        lower = -pi
                        upper = pi
                    else:
                        try:
                            lower = float(limit.get("lower"))
                            upper = float(limit.get("upper"))
                        except ValueError:
                            self.get_logger().warning("Invalid lower or upper limit: {}".format(name))
                    try:
                            maxvel = float(limit.get("velocity"))
                    except ValueError:
                            self.get_logger().warning("Invalid velocity limit: {}".format(name))

                # get safety_controller tag
                if use_safety_controller:
                    safety_controller = child.find("safety_controller")
                    if safety_controller is None:
                        self.get_logger().warning("No safety_controller tag: {}".format(name))
                    else:
                        try:
                            lower = float(safety_controller.get("soft_lower_limit"))
                            upper = float(safety_controller.get("soft_upper_limit"))
                        except ValueError:
                            self.get_logger().warning("Invalid soft_lower and upper_limit: {}".format(name))

                if use_mimic:
                    mimic = child.find("mimic")
                    if mimic is not None:
                        self.get_logger().warning("Mimic tag found: {}".format(name))

                        entry = {"joint": mimic.get("joint")}
                        if "multiplier" in mimic.attrib:
                            entry["multiplier"] = float(mimic.get("multiplier"))
                        if "offset" in mimic.attrib:
                            entry["offset"] = float(mimic.get("offset"))

                        dependent_joints[name] = entry
                        continue

                joint_names.append(name)
                joint = {
                    "lower": lower,
                    "upper": upper,
                    "velocity": maxvel
                    }
                free_joints[name] = joint
        return joint_names, free_joints, dependent_joints
