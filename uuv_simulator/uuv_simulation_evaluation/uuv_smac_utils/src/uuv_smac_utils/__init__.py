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

from opt_configuration import OptConfiguration
from simulation_pool import N_SIMULATION_RUNS, N_CRASHES, N_SUCCESS, \
    run_simulation, start_simulation_pool, stop_simulation_pool
from utils import SIMULATION_LOGGER, init_logger, parse_param_input, \
    SIM_SUCCESS, SIM_CRASHED
