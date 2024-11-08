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
import rospy
import numpy as np
import os
import matplotlib.pyplot as plt
from simulation_data import SimulationData, COLOR_RED


class SalinityData(SimulationData):
    LABEL = 'salinity_sensor'

    def __init__(self, bag):
        super(SalinityData, self).__init__()

        self._unit = None
        for x in bag.get_type_and_topic_info():
            for k in x:
                if 'uuv_sensor_plugins_ros_msgs/Salinity' in x[k][0]:
                    self._topic_name = k
                    self._logger.info('Particle salinity topic found <%s>' % k)
                    break
            if self._topic_name is not None:
                break

        self._time = list()

        try:
            self._recorded_data['salinity'] = list()
            for topic, msg, time in bag.read_messages(self._topic_name):
                    time = msg.header.stamp.to_sec()
                    self._time.append(time)
                    self._recorded_data['salinity'].append(msg.salinity)
                    if self._unit is None:
                        self._unit = msg.unit
            self._logger.info('%s=loaded' % self._topic_name)
        except Exception as e:
            self._logger.error('Error reading salinity topic, message=' + str(e))

    def get_as_dataframe(self, add_group_name=None):
        try:
            import pandas

            if len(self._recorded_data['salinity']) == 0:
                return None
                
            data = dict()
            data[self.LABEL + '_time'] = self._time
            data[self.LABEL + '_salinity'] = self._recorded_data['salinity']

            if add_group_name is not None:
                data['group'] = [add_group_name for _ in range(len(self._time))]

            return pandas.DataFrame(data)

        except Exception as ex:
            print('Error while exporting as pandas.DataFrame, message=' + str(ex))
            return None

    def plot(self, output_dir):
        if not os.path.isdir(output_dir):
            self._logger.error('Invalid output directory, dir=' + str(output_dir))
            raise Exception('Invalid output directory')

        fig = self.get_figure()
        try:
            output_path = (self._output_dir if output_dir is None else output_dir)
            
            ax = fig.add_subplot(111)

            ax.plot(
                self._time, 
                self._recorded_data['salinity'], 
                color=COLOR_RED, 
                label=r'Salinity [%s]' % self._unit,
                linewidth=self._plot_configs['linewidth'])
            
            ax.set_ylim([np.min(self._recorded_data['salinity']) - 0.1, 
                         np.max(self._recorded_data['salinity']) + 0.1])

            self.config_2dplot(
                ax=ax,
                title='',
                xlabel=r'Time [s]',
                ylabel=r'Salinity [%s]' % self._unit,
                legend_on=False)  
            
            ax.grid(True)
            plt.autoscale(enable=True, axis='x', tight=True)

            plt.tight_layout()
            plt.savefig(os.path.join(output_path, 'salinity.pdf'))
            plt.close(fig)
            del fig

            if len(self._time) == 0:
                self._logger.error('No salinity information found')
                return
        except Exception as e:
            self._logger.error('Error while plotting salinity, message=' + str(e))
            plt.close(fig)
            del fig
