#!/bin/bash
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

cd $CATKIN_WORKSPACE
/bin/bash -c "source devel/setup.bash"

cd $CATKIN_WORKSPACE/src/uuv_simulation_evaluation/uuv_simulation_wrapper
catkin run_tests --no-deps --this

cd $CATKIN_WORKSPACE/src/uuv_simulation_evaluation/uuv_simulation_evaluation
catkin run_tests --no-deps --this

catkin_test_results
