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
#
# Modifications made by Wannabe-CodingMaster in 2023:
# - Modified to use topic instead of action to control the robot's movement.
# - This portion of the code is subject to the same Apache License 2.0 terms.

import os
import math
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFormLayout, QLabel
from PyQt5.QtCore import Qt

from .controller_lister import ControllerLister
from .joint_limits import JointLimitManager
from .joint_states import JointStateManager
from .double_editor import DoubleEditor

class JointTrajectoryPublisher(Plugin):

    def __init__(self, context):
        super().__init__(context)
        # Give QObjects reasonable names
        self.setObjectName("JointTrajectoryPublisher")

        self._node = Node("rqt_joint_trajectory_publisher")

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(
            get_package_share_directory("rqt_joint_trajectory_publisher"),
            "resource",
            "rqt_joint_trajectory_publisher.ui"
        )
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName("JointTrajectoryPublisherUi")
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # initialize
        self._use_safety_controller = True
        self._use_mimic = True

        self._initialize_class()
        self._clear_plugin()
        self._safety_controller_check()
        self._mimic_check()

        # Signals
        self._widget.reload_button.clicked.connect(self._clear_plugin)
        self._widget.publish_button.clicked.connect(self._publish_command)
        self._widget.controller_manager_combo.currentIndexChanged.connect(self._controller_manager_changed)
        self._widget.jtc_combo.currentIndexChanged.connect(self._joint_trajectory_controller_changed)
        self._widget.safety_checkbox.stateChanged.connect(self._safety_controller_check)
        self._widget.mimic_checkbox.stateChanged.connect(self._mimic_check)

    def shutdown_plugin(self):
        self._clear_node()
        self._node.destroy_node()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    def _initialize_class(self):
        self._controller_lister = ControllerLister()
        self._joint_limit_manager = JointLimitManager()
        self._joint_state_manager = JointStateManager()

    def _safety_controller_check(self):
        self._use_safety_controller = self._widget.safety_checkbox.isChecked()
        if self._widget.jtc_combo.currentIndex() != -1:
            self._joint_trajectory_controller_changed()

    def _mimic_check(self):
        self._use_mimic = self._widget.mimic_checkbox.isChecked()
        if self._widget.jtc_combo.currentIndex() != -1:
            self._joint_trajectory_controller_changed()

    def _load_controller_manager_list(self):
        """
        Add controller managers in combo box
        """
        self._widget.controller_manager_combo.blockSignals(True)
        self._widget.controller_manager_combo.addItem("")

        cm_list = self._controller_lister.get_controller_manager()
        if not cm_list:
            self._node.get_logger().warning("Empty controller manager list.")
            return

        for cm in cm_list:
            self._widget.controller_manager_combo.addItem(cm)
        self._widget.controller_manager_combo.setCurrentIndex(0)
        self._widget.controller_manager_combo.blockSignals(False)

    def _load_jtc_list(self, current_cm):
        """
        Add joint trajectory controllers in combo box
        """
        jtc_list = self._controller_lister.get_jtc(current_cm)
        if not jtc_list:
            self._node.get_logger().warning("Empty joint trajectory controller list.")
            return
        for jtc in jtc_list:
            self._widget.jtc_combo.addItem(jtc)

    def _controller_manager_changed(self):
        """
        When controller manager is selected, get joint trajectory controller list
        """
        current_cm = self._widget.controller_manager_combo.currentText()
        # # Debug
        # self._node.get_logger().info("Selected controller manager = " + current_cm)

        if current_cm == "":
            self._node.get_logger().warning("Empty controller manager is selected.")
            self._clear_jtc_combo()
            return

        self._load_jtc_list(current_cm)

    def _joint_trajectory_controller_changed(self):
        """
        when joint trajectory controller is selected, get joint limits and set UI
        """
        self._clear_widget()
        self.current_jtc = self._widget.jtc_combo.currentText()
        # # Debug
        # self._node.get_logger().info("Selected joint trajectory controller = " + self.current_jtc)

        if self.current_jtc == "":
            self._node.get_logger().warning("Empty joint trajectory controller is selected.")
            return

        # Get joint info
        self.joint_names, free_joints, dependent_joints = \
            self._joint_limit_manager.get_joint_limits(self._use_safety_controller, self._use_mimic)

        # # Debug
        # self._node.get_logger().info("joint_names: " + str(self.joint_names))
        # self._node.get_logger().info("free_joints: " + str(free_joints))
        # self._node.get_logger().info("dependent_joints: " + str(dependent_joints))

        current_joint_position = self._joint_state_manager.get_joint_state()
        # # Debug
        # self._node.get_logger().info("current_joint_position: " + str(current_joint_position))

        position_limit = self._get_position_limit(free_joints, dependent_joints)
        # # Debug
        # self._node.get_logger().info("position_limit: " + str(position_limit))

        # Set joint UI
        layout = self._widget.joint_group.layout()

        for index, joint in enumerate(self.joint_names):
            # Add joint name
            label = QLabel(joint)
            layout.addRow(label)

            min_val = position_limit[index][0]
            max_val = position_limit[index][1]

            # Current_joint_position
            position = current_joint_position[index]
            current_val = round(position, 2)

            joint_widget = DoubleEditor(min_val, max_val)
            joint_widget.setValue(current_val)
            layout.addRow(joint_widget)

    def _get_position_limit(self, free_joints, dependent_joints):
        position_limit = []

        for joint in self.joint_names:
            if joint in free_joints:
                min_value = math.ceil(free_joints[joint]["lower"] * 100) / 100
                max_value = math.floor(free_joints[joint]["upper"] * 100) / 100

            elif joint in dependent_joints:
                mimic_what = dependent_joints[joint]["joint"]
                base_joint_lower_position = free_joints[mimic_what]["lower"]
                base_joint_upper_position = free_joints[mimic_what]["upper"]
                multiplier = dependent_joints[joint]["multiplier"]
                offset = dependent_joints[joint]["offset"]

                mimic_joint_position_1 = base_joint_lower_position * multiplier + offset
                mimic_joint_position_2 = base_joint_upper_position * multiplier + offset
                if mimic_joint_position_1 > mimic_joint_position_2:
                    min_value = math.ceil(mimic_joint_position_2 * 100) / 100
                    max_value = math.floor(mimic_joint_position_1 * 100) / 100
                else:
                    min_value = math.ceil(mimic_joint_position_1 * 100) / 100
                    max_value = math.floor(mimic_joint_position_2 * 100) / 100

            position_limit.append([min_value, max_value])
        return position_limit

    def _publish_command(self):
        msg = JointTrajectory()
        point_msg = JointTrajectoryPoint()
        positions = []

        msg.joint_names = self.joint_names
        joint_widgets = self._joint_widgets()

        for id in range(len(joint_widgets)):
            positions.append(joint_widgets[id].getValue())

        point_msg.positions = positions
        point_msg.time_from_start = rclpy.time.Duration(seconds=1.0).to_msg()

        msg.points.append(point_msg)
        self.msg_publisher = self._node.create_publisher(JointTrajectory, self.current_jtc + '/joint_trajectory', 1)
        self.msg_publisher.publish(msg)

    def _clear_plugin(self):
        self._widget.controller_manager_combo.clear()
        self._clear_jtc_combo()
        self._load_controller_manager_list()

    def _clear_jtc_combo(self):
        self._widget.jtc_combo.clear()
        self._clear_widget()
        self._clear_node()
        self._initialize_class()

    def _clear_widget(self):
        layout = self._widget.joint_group.layout()
        if layout is not None:
            while layout.count():
                layout.removeRow(0)
        self._widget.joint_group.setLayout(QFormLayout())

    def _clear_node(self):
        self._controller_lister.jtc_node.destroy_node()
        self._controller_lister.cm_node.destroy_node()
        self._joint_limit_manager.destroy_node()
        self._joint_state_manager.destroy_node()

    def _joint_widgets(self):
        widgets = []
        layout = self._widget.joint_group.layout()
        for row_id in range(layout.rowCount()):
            if row_id % 2 == 1:
                widgets.append(layout.itemAt(row_id, QFormLayout.FieldRole).widget())
        return widgets
