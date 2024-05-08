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

from rclpy.node import Node
from controller_manager import controller_manager_services


class ControllerLister:
    def __init__(self):
        self.cm_node = Node("get_controller_manager_node")
        self.jtc_node = Node("get_joint_trajectory_controller_node")

    def get_controller_manager(self):
        """
        get "namespace/controller_manager"
        """
        srv_list = self.cm_node.get_service_names_and_types()

        name = []
        if srv_list:
            for srv in srv_list:
                match = "/list_controllers"
                if match in srv[0]:
                    ns = srv[0].split(match)[0]
                    name.append(ns)
        else:
            self.cm_node.get_logger().warning("ControllerLister: No service found")
        return name

    def get_jtc(self, cm):
        """
        @param cm: selected "namespace/controller_manager"
        get all controller list from controller manager
        """
        result = controller_manager_services.list_controllers(self.jtc_node, cm)
        jtc_list = self._filter_jtc(result)
        return jtc_list

    def _filter_jtc(self, controller_list):
        """
        @param controller_list: Response of 'controller_list_client'
        filter joint trajectory controller
        """
        jtc_list = []
        for controller in controller_list.controller:
            if controller.type == 'joint_trajectory_controller/JointTrajectoryController':
                jtc_list.append(controller.name)
        return jtc_list
