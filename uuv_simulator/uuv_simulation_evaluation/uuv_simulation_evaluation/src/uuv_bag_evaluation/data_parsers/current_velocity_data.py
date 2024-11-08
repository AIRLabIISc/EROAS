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
from simulation_data import SimulationData, COLOR_RED, COLOR_GREEN, COLOR_BLUE


class CurrentVelocityData(SimulationData):
    LABEL = 'current_velocity'

    def __init__(self, bag):
        super(CurrentVelocityData, self).__init__()

        for x in bag.get_type_and_topic_info():
            for k in x:
                if 'current_velocity' in k:
                    self._topic_name = k
                    self._logger.info('Current velocity topic found <%s>' % k)
                    break
            if self._topic_name is not None:
                break

        self._time = list()

        try:
            self._recorded_data['vel'] = list()
            for topic, msg, time in bag.read_messages(self._topic_name):
                self._time.append(msg.header.stamp.to_sec())
                self._recorded_data['vel'].append(
                    [msg.twist.linear.x,  msg.twist.linear.y,  msg.twist.linear.z])
            self._logger.info('%s=loaded' % self._topic_name)
        except Exception as e:
            self._logger.error('Error retrieving current velocity data from rosbag, message=' + str(e))

    def get_as_dataframe(self, add_group_name=None):
        try:
            import pandas

            data = dict()
            data[self.LABEL + '_time'] = self._time
            data[self.LABEL + '_vel_x'] = [x[0] for x in self._recorded_data['vel']]
            data[self.LABEL + '_vel_y'] = [x[1] for x in self._recorded_data['vel']]
            data[self.LABEL + '_vel_z'] = [x[2] for x in self._recorded_data['vel']]

            if add_group_name is not None:
                data['group'] = [add_group_name for _ in range(len(self._time))]

            return pandas.DataFrame(data)

        except Exception as ex:
            print('Error while exporting as pandas.DataFrame, message=' + str(ex))
            return None

    @property
    def current_velocity(self):
        return self._time, self._recorded_data['vel']

    def plot(self, output_dir):
        if not os.path.isdir(output_dir):
            self._logger.error('Invalid output directory, dir=' + str(output_dir))
            raise Exception('Invalid output directory')

        try:
            output_path = (self._output_dir if output_dir is None else output_dir)
            
            fig = self.get_figure()        
            ax = fig.gca()

            ax.plot(
                self._time, 
                [v[0] for v in self._recorded_data['vel']], 
                color=COLOR_RED, 
                label=r'$u_C$',
                linewidth=self._plot_configs['linewidth'])
            ax.plot(
                self._time, 
                [v[1] for v in self._recorded_data['vel']], 
                color=COLOR_GREEN, 
                label=r'$v_C$',
                linewidth=self._plot_configs['linewidth'])
            ax.plot(
                self._time, 
                [v[2] for v in self._recorded_data['vel']], 
                color=COLOR_BLUE, 
                label=r'$w_C$',
                linewidth=self._plot_configs['linewidth'])

            min_y = [np.min([v[0] for v in self._recorded_data['vel']]),
                     np.min([v[1] for v in self._recorded_data['vel']]),
                     np.min([v[2] for v in self._recorded_data['vel']])]

            max_y = [np.max([v[0] for v in self._recorded_data['vel']]),
                     np.max([v[1] for v in self._recorded_data['vel']]),
                     np.max([v[2] for v in self._recorded_data['vel']])]

            lim_y = max(np.abs(np.min(min_y)), np.abs(np.max(max_y)))
            ax.set_ylim([np.min(min_y) - 0.1, np.max(max_y) + 0.1])

            self.config_2dplot(
                ax=ax,
                title='',
                xlabel='Time [s]',
                ylabel='Velocity [m/s]',
                legend_on=True)  
            # ax.set_yticks(np.linspace(np.floor(np.min(min_y)), np.floor(np.max(max_y)), 0.2))

            ax.grid(True)
            plt.autoscale(enable=True, axis='x', tight=True)

            plt.tight_layout()
            plt.savefig(os.path.join(output_path, 'current_velocity.pdf'))
            plt.close(fig)
            del fig
        except Exception as e:
            self._logger.error('Error while plotting current velocity data, message=' + str(e))
            plt.close(fig)
            del fig