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
NAME = 'test_constraint'

import rospy
import rostest
import unittest
from uuv_cost_function import Constraint, PenaltyFunction, LogBarrierMethod, InverseBarrierMethod, DistancePenaltyFunction

import roslib; roslib.load_manifest(PKG)


class TestConstraint(unittest.TestCase):
    def setUp(self):
        self.penalty_params_valid = dict(gain=1.0, offset=10.0, n=1.0, c=1.0)
        self.penalty_params_invalid = dict(gain='f', offset=None, n=0.0, c=0.0)
        self.log_barrier_params = dict(gain=1.0, offset=10.0, c=1.0)
        self.inverse_barrier_params = dict(gain=1.0, offset=10.0, c=1.0)
        self.dist_penalty_params_float = dict(gain=1.0, offset=1.0, n=1.0, c=1.0)
        self.dist_penalty_params_list = dict(gain=1.0, offset=[1.0, 2.0, 3.0], n=1.0, c=1.0)

    def test_create_fcn(self):
        invalid_fcn_tag = 'test'

        with self.assertRaises(Exception):
            Constraint.create(invalid_fcn_tag)

        self.assertIsInstance(Constraint.create('LogBarrierMethod',
                                                'test_log',
                                                'x'),
                              LogBarrierMethod,
                              'Log barrier constraint object could not be created')
        self.assertIsInstance(Constraint.create('InverseBarrierMethod',
                                                'test_inverse',
                                                'x'),
                              InverseBarrierMethod,
                              'Inverse barrier constraint object could not be created')
        self.assertIsInstance(Constraint.create('PenaltyFunction',
                                                'test_penalty',
                                                'x'),
                              PenaltyFunction,
                              'Penalty constraint object could not be created')
        self.assertIsInstance(Constraint.create('DistancePenaltyFunction',
                                                'test_distance_penalty',
                                                'x'),
                              DistancePenaltyFunction,
                              'Distance penalty constraint object could not be created')

    def test_penalty(self):
        p_fcn = Constraint.create('PenaltyFunction',
                                  'test_penalty',
                                  'x')
        p_fcn.from_dict(self.penalty_params_valid)

        self.assertEquals(p_fcn.compute(0), 0,
                          'Invalid output for value in the feasible set')
        self.assertGreater(p_fcn.compute(self.penalty_params_valid['offset'] + 1), 0,
                           'Invalid output for value outside of the feasible set')

    def test_barrier(self):
        pass
    
    def test_distance_penalty(self):
        p_fcn = Constraint.create('DistancePenaltyFunction',
                                  'test_distance_penalty',
                                  'x')
        p_fcn.from_dict(self.dist_penalty_params_float)
        self.assertEquals(p_fcn.compute(1), 0,
                          'Invalid output for value in the feasible set')

        p_fcn = Constraint.create('DistancePenaltyFunction',
                                  'test_distance_penalty',
                                  'x')
        p_fcn.from_dict(self.dist_penalty_params_list)

        self.assertEquals(p_fcn.compute(1), 0,
                          'Invalid output for value in the feasible set')
        self.assertEquals(p_fcn.compute(2), 0,
                          'Invalid output for value in the feasible set')
        self.assertEquals(p_fcn.compute(3), 0,
                          'Invalid output for value in the feasible set')
                          

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, NAME, TestConstraint)
