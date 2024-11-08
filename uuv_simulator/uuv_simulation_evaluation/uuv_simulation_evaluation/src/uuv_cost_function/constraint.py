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

import numpy as np
import yaml
import os
import random


class Constraint(object):
    def __init__(self, tag='', input_tag=''):
        self.x = 0.0
        self.tag = tag
        self.input_tag = input_tag
        self.params = dict(c=0.0, gain=0.0, offset=0.0)

    @staticmethod
    def create(model_name, *args):
        for fcn in Constraint.__subclasses__():
            if model_name == fcn.__name__:
                return fcn(*args)
        raise Exception('Invalid constraint model')

    def from_dict(self, params):
        for tag in params:
            if tag == 'offset':
                assert type(params[tag]) in [float, int, list], 'Parameter with tag <%s> is not a number nor a list' % tag    
            else:
                assert type(params[tag]) in [float, int], 'Parameter with tag <%s> is not a number' % tag
            assert tag in self.params, 'Invalid parameter, tag=%s' % tag
            self.params[tag] = params[tag]

    def get_params(self):
        params = self.params
        params['function_name'] = self.__class__.__name__
        params['x'] = float(self.x)
        params['tag'] = self.tag
        params['input_tag'] = self.input_tag
        params['result'] = float(self.compute())
        return params

    def save(self, output_dir='.'):
        assert os.path.isdir(output_dir), 'Invalid output directory'
        try:
            with open(os.path.join(output_dir, '%s_%s_%d.yaml' % (self.tag, self.input_tag, int(random.random() % 100))), 'w') as cf_file:
                yaml.dump(self.get_params(), cf_file, default_flow_style=False)
            return True
        except Exception as e:
            print('Error while storing cost function configuration, message=', str(e))
            return False

    def compute(self, x=None):
        raise NotImplementedError()


class LogBarrierMethod(Constraint):
    def __init__(self, tag='', input_tag=''):
        Constraint.__init__(self, tag, input_tag)

    def compute(self, x=None):
        if x is not None:
            self.x = x
        if self.x - self.params['offset'] > 0:
            return 0.0
        return -1 * self.params['c'] * np.log(-1 * self.params['gain'] * (self.x - self.params['offset']))


class InverseBarrierMethod(Constraint):
    def __init__(self, tag='', input_tag=''):
        Constraint.__init__(self, tag, input_tag)

    def compute(self, x=None):
        if x is not None:
            self.x = x
        d = self.params['gain'] * (self.x - self.params['offset'])
        if abs(d) < 1e-5:
            d = 1e-5 * np.sign(d)
        return -1 * self.params['c'] * 1 / d


class PenaltyFunction(Constraint):
    def __init__(self, tag='', input_tag=''):
        Constraint.__init__(self, tag, input_tag)
        self.params['n'] = 0.0

    def compute(self, x=None):
        if x is not None:
            self.x = x
        if self.x - self.params['offset'] < 0:
            return 0
        return self.params['c'] * np.power(max(0, self.params['gain'] * (self.x - self.params['offset'])), self.params['n'])


class DistancePenaltyFunction(Constraint):
    def __init__(self, tag='', input_tag=''):
        Constraint.__init__(self, tag, input_tag)
        self.params['n'] = 0.0

    def compute(self, x=None):
        if x is not None:
            self.x = x
        else:
            return 0

        if isinstance(self.params['offset'], list):
            return np.min([self.params['c'] * np.power(self.params['gain'] * np.abs(x - i), self.params['n']) for i in self.params['offset']])
        else:
            return self.params['c'] * np.power(self.params['gain'] * np.abs(x - self.params['offset']), self.params['n'])

