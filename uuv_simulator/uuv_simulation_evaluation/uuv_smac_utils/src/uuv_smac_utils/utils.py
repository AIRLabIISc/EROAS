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
import sys
import os
import logging

# Initializing logger
SIMULATION_LOGGER = logging.getLogger('smac_utils')

# Initialize useful labels
SIM_SUCCESS = 'SUCCESS'
SIM_CRASHED = 'CRASHED'

def parse_param_input(args, input_map):
    if isinstance(args, dict):
        p = args
    else:
        p = vars(args)
    params = dict()
    for tag in input_map:
        if type(input_map[tag]) == list:
            p_cont = list()
            for elem in input_map[tag]:
                if type(elem) == str:
                    p_cont.append(p[elem])
                else:
                    p_cont.append(elem)
        else:
            if type(input_map[tag]) == str:
                p_cont = p[input_map[tag]]
            else:
                p_cont = input_map[tag]
        params[tag] = p_cont

    return params


def init_logger(log_filename=None):
    if len(SIMULATION_LOGGER.handlers) == 0:
        out_hdlr = logging.StreamHandler(sys.stdout)
        out_hdlr.setFormatter(logging.Formatter(
            '%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
        out_hdlr.setLevel(logging.INFO)

        SIMULATION_LOGGER.addHandler(out_hdlr)
        SIMULATION_LOGGER.setLevel(logging.INFO)

        if log_filename is None:
            if not os.path.isdir('logs'):
                os.makedirs('logs')
            log_filename = os.path.join('logs', 'simulation_pool.log')

        file_hdlr = logging.FileHandler(log_filename)
        file_hdlr.setFormatter(logging.Formatter(
            '%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
        file_hdlr.setLevel(logging.INFO)

        SIMULATION_LOGGER.addHandler(file_hdlr)
        SIMULATION_LOGGER.setLevel(logging.INFO)
