# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
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

import rospy
import logging
import sys
import rosbag
import numpy as np
from data_parsers import SimulationData
from uuv_trajectory_generator import TrajectoryGenerator, TrajectoryPoint


class Recording:
    __instance = None

    def __init__(self, filename):
        # Setting up the log
        self._logger = logging.getLogger('read_rosbag')
        if len(self._logger.handlers) == 0:
            out_hdlr = logging.StreamHandler(sys.stdout)
            out_hdlr.setFormatter(logging.Formatter('%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
            out_hdlr.setLevel(logging.INFO)
            self._logger.addHandler(out_hdlr)
            self._logger.setLevel(logging.INFO)

        # Bag filename
        self._filename = filename
        self._bag = rosbag.Bag(filename)
        
        self.parsers = dict()

        self._is_init = False

        Recording.__instance = self

    @classmethod
    def get_instance(cls):
        if cls.__instance is None:
            cls.__instance = Recording()
        return cls.__instance

    @property
    def is_init(self):
        return self._is_init

    def init_parsers(self):        
        self._logger.info('Initializing parsers')
        for parser in SimulationData.get_all_parsers():
            self._logger.info('Initializing parser=%s', parser.LABEL)               
            self.parsers[parser.LABEL] = parser(self._bag)
        self._is_init = True

  