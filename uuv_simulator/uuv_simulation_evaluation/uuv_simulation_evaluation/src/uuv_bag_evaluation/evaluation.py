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
import os
import sys
import numpy as np
import yaml
import tf.transformations as trans
from copy import deepcopy
from .recording import Recording
from .error import ErrorSet
from .metrics import KPI
import matplotlib.pyplot as plt
import logging
from mpl_toolkits.mplot3d import Axes3D

try:
    plt.rc('text', usetex=True)
    plt.rc('font', family='sans-serif')
except Exception as e:
    print('Cannot use Latex configuration with matplotlib, message=' + str(e))

class Evaluation(object):
    def __init__(self, filename, output_dir='.', time_offset=0.0):
        # Setting up the log
        self._logger = logging.getLogger('run_evaluation')
        if len(self._logger.handlers) == 0:
            out_hdlr = logging.StreamHandler(sys.stdout)
            out_hdlr.setFormatter(logging.Formatter('%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
            out_hdlr.setLevel(logging.INFO)
            self._logger.addHandler(out_hdlr)
            self._logger.setLevel(logging.INFO)

        self._logger.info('Opening bag: %s' % filename)
        self.recording = Recording(filename)
        
        self.recording.init_parsers()

        # Create error set object
        self._error_set = ErrorSet.get_instance()
        if self._error_set is None:
            self._logger.error('Error set has not been correctly initialized')
            raise Exception('Error set has not been correctly initialized')

        self._error_set.compute_errors()        

        # Assigning the output directory for the results
        if not os.path.isdir(output_dir):
            self._logger.error('Invalid output directory, dir=%s' % str(output_dir) )
            raise Exception('Invalid output directory')

        # Simulation time offset to start the computation of each KPI
        if time_offset >= 0.0:
            self._time_offset = time_offset
        else:
            self._logger.error('Invalid time offset, setting time offset to zero')
            self._time_offset = 0.0

        self._logger.info('Time offset for KPI evaluation [s]=' + str(self._time_offset))

        self._output_dir = output_dir
        # Table of configuration parameters (set per default all KPIs)
        self._kpis = list()
        for kpi in KPI.get_all_kpi_tags():
            if KPI.get_kpi_target(kpi) == 'error':
                for error_tag in self._error_set.get_tags():
                    self._kpis.append(dict(func=KPI.get_kpi(kpi, error_tag),
                                           value=0.0))
            else:
                self._kpis.append(dict(func=KPI.get_kpi(kpi),
                                       value=0.0))

        self._cost_fcn_terms = dict()

        # Calculating the KPIs for this bag
        self.compute_kpis()

    def __del__(self):
      if self.recording is not None:
        del self.recording

    @property
    def error_set(self):
        return self._error_set

    def calc_cost_fcn(self):
        cost = 0.0
        for tag in self._cost_fcn_terms:
            cost += self._cost_fcn_terms[tag] * self.get_kpi(tag)
        return cost

    def save_cost_fcn_config(self, filename):
        try:
            with open(filename, 'w') as cost_file:
                yaml.dump(self._cost_fcn_terms, cost_file,
                          default_flow_style=False)
        except Exception as e:
            self._logger.error('Error exporting cost function configuration, message=' + str(e))

    def load_cost_fcn(self, filename):
        if not os.path.isfile(filename):
            self._logger.error('Invalid filename, file=%s' % filename)
            return False
        with open(filename, 'w') as fcn_file:
            fcn = yaml.load(fcn_file)
        try:
            for item in fcn:
                self.add_cost_fcn_term(item, fcn[item])
                self._logger.info('Cost function term (tag, weight): (%s, %.4f)' % (item, fcn[item]))
        except Exception as e:
            self._logger.error('Error loading cost function configuration')
            self._logger.error(e)
            return False
        return True

    def add_cost_fcn_term(self, kpi, weight):
        if weight <= 0:
            self._logger.error('Weight must be a positive value')
            return False
        if not self.has_kpi(kpi):
            self._logger.error('KPI tag is invalid, tag=%s' % kpi)
            return False
        if kpi in self._cost_fcn_terms:
            self._logger.error('KPI already added to the cost function, tag=%s' % kpi)
            return False

        self._cost_fcn_terms[kpi] = weight
        return True

    def has_kpi(self, tag):
        for kpi in self._kpis:
            if tag == kpi.full_tag:
                return True
        return False

    def set_kpis_from_file(self, filename):
        assert os.path.isfile(filename), 'Invalid evaluation configuration file'
        assert '.yaml' in filename or '.yml' in filename, 'Configuration file should be a YAML file'

        with open(filename, 'r') as config_file:
            config = yaml.load(config_file)

        self.set_kpis(config)

    def set_kpis(self, config):
        assert type(config) == list, 'Invalid configuration structure for KPIs'
        kpi_tags = KPI.get_all_kpi_tags()

        self._kpis = list()
        for item in config:
            if item['func'] not in kpi_tags:
                self._logger.error('Invalid KPI tag, value=' + item)
            else:
                self._logger.info('KPI created=' + item['func'])
                if 'args' in item:
                    self._kpis.append(dict(func=KPI.get_kpi(item['func'], item['args']),
                                           value=0.0))
                    self._logger.info('\tArguments: ' + str(item['args']))
                else:
                    self._kpis.append(dict(func=KPI.get_kpi(item['func']),
                                           value=0.0))

    def get_kpis(self):
        kpis = dict()
        for kpi in self._kpis:
            item = kpi['func']
            try:
                kpis[item.full_tag] = float(item.kpi_value)
            except:
                kpis[item.full_tag] = -1000.0
        return kpis

    def get_kpi(self, tag):
        for kpi in self._kpis:
            if kpi['func'].full_tag == tag:
                return kpi['value']
        return None

    def compute_kpis(self):
        if len(self._kpis):
            for i in range(len(self._kpis)):
                try:
                    self._kpis[i]['value'] = self._kpis[i]['func'].compute()
                except Exception as e:
                    self._logger.error('Error calculating KPI %s, message=%s' % (self._kpis[i]['func'].full_tag, str(e)))

    def print_kpis(self):
        for item in self._kpis:
            print(item['func'].full_tag + '= ' + item['value'])

    def get_trajectory_coord(self, tag):
        return self.recording.get_trajectory_coord(tag)

    def export_to_txt(self, tag, output_dir):
        pass

    def save_dataframes(self, output_dir=None):
        if output_dir is not None:
            if not os.path.isdir(output_dir):
                self._logger.error('Invalid output directory, dir=' + str(output_dir))
                raise Exception('Invalid output directory')
        output_path = (self._output_dir if output_dir is None else output_dir)
        try:        
            for tag in self.recording.parsers:
                self._logger.info('Reading data frame for ' + tag)
                df = self.recording.parsers[tag].get_as_dataframe()

                if df is None:
                    continue
                
                if not os.path.isdir(os.path.join(output_path, 'data')):
                    os.makedirs(os.path.join(output_path, 'data'))

                if isinstance(df, dict):
                    for k in df:
                        with open(os.path.join(output_path, 'data', '%s_%s.yaml' % (tag, k)), 'w') as data_file:
                            yaml.dump(df[k].to_dict(), data_file, default_flow_style=False)        
                else:
                    with open(os.path.join(output_path, 'data', '%s.yaml' % tag), 'w') as data_file:
                        yaml.dump(df.to_dict(), data_file, default_flow_style=False)
                self._logger.info('Data frame <%s> stored=%s' % (tag, os.path.join(output_path, 'data', '%s.yaml' % tag)))
        except Exception as e:
            self._logger.error('Error storing dataframes file, message=' + str(e))

    def save_evaluation(self, output_dir=None):
        if output_dir is not None:
            if not os.path.isdir(output_dir):
                self._logger.error('Invalid output directory, dir=' + str(output_dir))
                raise Exception('Invalid output directory')
        self.save_kpis(output_dir)

        for tag in self.recording.parsers:
            self.recording.parsers[tag].plot(self._output_dir)
        self._logger.info('Evaluation stored!')

    def save_kpis(self, output_dir=None):
        if output_dir is not None:
            if not os.path.isdir(output_dir):
                self._logger.error('Invalid output directory, dir=' + str(output_dir))
                raise Exception('Invalid output directory')
        try:
            output_path = (self._output_dir if output_dir is None else output_dir)

            kpis = dict()
            kpi_labels = dict()
            for kpi in self._kpis:
                item = kpi['func']
                try:
                    value = float(item.kpi_value)
                except Exception as e:
                    value = 0.0
                kpis[item.full_tag] = value
                kpi_labels[item.full_tag] = kpi['func'].label
            with open(os.path.join(output_path, 'computed_kpis.yaml'), 'w') as kpi_file:
                yaml.dump(kpis, kpi_file, default_flow_style=False)
            self._logger.info('Calculated KPIs stored in <%s>' % os.path.join(output_path, 'computed_kpis.yaml'))

            with open(os.path.join(output_path, 'kpi_labels.yaml'), 'w') as kpi_file:
                yaml.dump(kpi_labels, kpi_file, default_flow_style=False)
            self._logger.info('KPI labels stored in <%s>' % os.path.join(output_path, 'kpi_labels.yaml'))
        except Exception as e:
            self._logger.error('Error storing KPIs file, message=' + str(e))
