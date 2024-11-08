#!/usr/bin/env python
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

PKG = 'uuv_simulation_evaluation'
NAME = 'test_cost_function'

import rospy
import rostest
import unittest
from uuv_cost_function import CostFunction

import roslib; roslib.load_manifest(PKG)


class TestCostFunction(unittest.TestCase):
    def setUp(self):
        self.cost_fcn = CostFunction()
        self.cost_fcn_params = dict(a=1.0, b=2.0, c=3.0)
        self.cost_fcn.from_dict(self.cost_fcn_params)

    def test_save_cost_function_params(self):
        self.assertEqual(self.cost_fcn_params.keys(), self.cost_fcn.get_kpis().keys(), 'The KPI tags were not initialized correctly')
        for tag in self.cost_fcn_params:
            self.assertEqual(self.cost_fcn.get_weight(tag), self.cost_fcn_params[tag], 'Weights have not been correctly initialized')

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, NAME, TestCostFunction)
