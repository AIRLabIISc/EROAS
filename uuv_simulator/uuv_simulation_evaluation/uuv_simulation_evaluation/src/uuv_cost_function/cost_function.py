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
from __future__ import print_function
import yaml
import os
import sys
import logging
import numpy as np
from .constraint import Constraint


class CostFunction(object):
    def __init__(self, norm=1):
        self.logger = logging.getLogger('cost_function')
        if len(self.logger.handlers) == 0:
            out_hdlr = logging.StreamHandler(sys.stdout)
            out_hdlr.setFormatter(logging.Formatter('%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
            out_hdlr.setLevel(logging.INFO)
            self.logger.addHandler(out_hdlr)
            self.logger.setLevel(logging.INFO)
            
            if not os.path.isdir('logs'):
                os.makedirs('logs')
            log_filename = os.path.join('logs', 'cost_function.log')

            file_hdlr = logging.FileHandler(log_filename)
            file_hdlr.setFormatter(logging.Formatter(
                '%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
            file_hdlr.setLevel(logging.INFO)
            self.logger.addHandler(file_hdlr)
            self.logger.setLevel(logging.INFO)

        # Load default KPIs
        if norm == 'inf':
            self.norm = np.inf
        else:
            self.norm = norm
        self.kpis = dict()
        self.weights = dict()
        self.constraints = list()
        self.export_data = dict(norm=self.norm)

    def set_norm(self, norm):
        if norm == 'inf':
            self.norm = np.inf
        else:
            assert isinstance(norm, int)
            self.norm = norm
            
    def add_constraints(self, constraints):
        for c in constraints:
            if not self.add_constraint(c['type'], c['tag'], c['input_tag'], c['params']):
                return False
        return True

    def add_constraint(self, fcn_name, tag, input_tag, params):
        try:
            c_fcn = Constraint.create(fcn_name, tag, input_tag)
            c_fcn.from_dict(params)
            self.constraints.append(c_fcn)
            self.logger.info('Constraint model <%s> added' % fcn_name)
            self.logger.info('\tTag=' + tag)
            self.logger.info('\tInput tag=' + input_tag)
            self.logger.info('\tParameters=' + str(params))
            return True
        except Exception as e:
            self.logger.error('Error adding constraint ' + fcn_name + ', message=' + str(e))
            return False

    def get_constraint_tags(self):
        if len(self.constraints) == 0:
            return list()
        return [c.tag for c in self.constraints]

    def from_dict(self, params):
        for tag in params:
            self.kpis[tag] = 0.0
            self.weights[tag] = params[tag]

    def is_kpi(self, tag):
        return tag in self.kpis

    def get_kpis(self):
        return self.kpis

    def get_cost_fcn_kpis(self):
        output = dict()
        for tag in self.weights:
            output[tag] = self.kpis[tag]
        return output

    def get_kpi(self, tag):
        if not self.is_kpi(tag):
            self.logger.info('<' + tag + '> KPI tag does not exist')
            return None
        else:
            return self.kpis

    def add_kpi(self, tag, value=0.0):
        if not self.is_kpi(tag):
            self.kpis[tag] = value
        return True
        
    def set_kpi(self, tag, value):
        self.kpis[tag] = value

    def set_weight(self, tag, weight):
        if not self.is_kpi(tag):
            self.logger.info('<' + tag + '> KPI tag does not exist')
            return False
        else:
            self.weights[tag] = weight
            return True

    def get_weight(self, tag):
        if not self.is_kpi(tag):
            self.logger.info('<' + tag + '> KPI tag does not exist')
            return None
        else:
            return self.weights[tag]

    def set_weights(self, weights):
        assert type(weights) == dict, 'Input weights must be listed in a dict'
        for tag in weights:
            if not self.is_kpi(tag):
                self.logger.info('<' + tag + '> KPI tag does not exist')
            else:
                self.weights[tag] = weights[tag]

    def set_kpis(self, kpis):
        self.logger.debug('start set_kpis')        
        assert type(kpis) == dict, 'Input KPIs must be listed in a dict'
        for tag in kpis:
            self.kpis[tag] = kpis[tag]
        self.logger.debug('end set_kpis')        

    def compute(self):        
        costs = list()
        self.logger.info('Calculating cost function=')    
        w = 1. / len(self.weights.keys())    
        for tag in sorted(self.weights.keys()):
            if self.kpis[tag] < 0:
                raise Exception('KPI <%s> has an invalid value=%.2f' % (tag, self.kpis[tag]))
            costs += [w * self.weights[tag] * self.kpis[tag]]

            self.logger.info('\t {} - Weight: {} * {} - KPI: {}'.format(tag, w, self.weights[tag], self.kpis[tag]))
            self.logger.info('\t\t Result: {}'.format(costs[-1]))

            self.export_data['weight_' + tag] = float(w * self.weights[tag])
            self.export_data[tag] = float(self.kpis[tag])
            self.export_data['cost_' + tag] = float(w * self.weights[tag] * self.kpis[tag])
            
        self.logger.info('Computing cost function from cost vector norm=%s' % str(self.norm))
        total_cost = np.linalg.norm(costs, ord=self.norm)
        
        self.logger.info('Cost (before constraints)=' + str(total_cost))        
        total_cost += self.compute_constraints()        

        self.export_data['total_cost'] = float(total_cost)
        self.logger.info('Cost (after constraints)=' + str(total_cost))
        self.export_data['norm'] = str(self.norm)
        return total_cost

    def compute_constraints(self):
        value = 0.0
        if len(self.constraints) > 0:
            for c in self.constraints:
                self.logger.info('\tConstraint=' + c.__class__.__name__)
                self.logger.info('\t\tTag=' + c.tag)
                self.logger.info('\t\tInput tag=' + c.input_tag)
                
                if c.input_tag not in self.kpis:
                    self.logger.error('Error computing constraint <%s>: '
                                      '%s tag not in KPIs list' % (c.tag, c.input_tag))
                    self.logger.error(self.kpis.keys())
                    raise Exception('%s tag not in KPIs list' % c.input_tag)
                c_fcn = c.compute(self.kpis[c.input_tag])

                self.export_data[c.tag] = float(c_fcn)
                
                self.logger.info('\t\tValue=' + str(c_fcn))                                
                value += c_fcn
        return value

    def save(self, output_dir='.'):
        assert os.path.isdir(output_dir), 'Invalid output directory'
        try:
            filename = os.path.join(output_dir, 'cost_function.yaml')
            with open(filename, 'w') as cf_file:
                yaml.dump(self.weights, cf_file, default_flow_style=False)

            for c in self.constraints:
                c.save(output_dir)
            return True
        except Exception as e:
            self.logger.error('Error while storing cost function configuration, message=' + str(e))
            return False

    def get_data(self):
        return self.export_data

if __name__ == '__main__':
    cf = CostFunction()
    for tag in cf.get_kpis():
        print(tag)

    cf.set_weight('rmse_yaw', 10.0)
    cf.set_kpi('rmse_yaw', 10.0)
    cf.set_kpi('rmse_pitch', 12.0)

    print(cf.compute())
