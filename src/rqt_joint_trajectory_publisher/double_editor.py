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

import os
from ament_index_python import get_package_share_directory

import math
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from decimal import Decimal

class DoubleEditor(QWidget):
    """
        Widget that allows to edit the value of a floating-point value.
    """
    def __init__(self, min_val, max_val):
        super().__init__()

        self._min_val = min_val
        self._max_val = max_val

        # Load UI
        ui_file = os.path.join(
                get_package_share_directory("rqt_joint_trajectory_publisher"),
                "resource",
                "double_editor.ui"
            )
        loadUi(ui_file, self)

        # Setup slider and spinbox
        self.slider.setRange(0, self._get_slider_max())
        self.slider.setSingleStep(1)
        self.spinbox.setRange(min_val, max_val)
        self.spinbox.setDecimals(2)
        self.spinbox.setSingleStep(0.01)

        # Signals
        self.slider.valueChanged.connect(self._slider_changed)
        self.spinbox.valueChanged.connect(self._spinbox_changed)

    def _get_slider_max(self):
        return int((self._max_val - self._min_val) * 100)

    def _slider_to_val(self, slider_val):
        return slider_val / 100 + self._min_val

    def _val_to_slider(self, val):
        return int(math.ceil((val - self._min_val) * 100))

    def _slider_changed(self):
        val = self._slider_to_val(self.slider.value())
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(val)
        self.spinbox.blockSignals(False)

    def _spinbox_changed(self):
        val = self.spinbox.value()
        self.slider.blockSignals(True)
        self.slider.setValue(self._val_to_slider(val))
        self.slider.blockSignals(False)

    def setValue(self, val):
        self.slider.setValue(self._val_to_slider(val))

    def getValue(self):
        return self.spinbox.value()

