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


class AUVCommandData(SimulationData):
    LABEL = 'auv_control'

    def __init__(self, bag):
        super(AUVCommandData, self).__init__()

        for x in bag.get_type_and_topic_info():
            for k in x:
                if 'uuv_auv_control_allocator/AUVCommand' in x[k][0]:
                    self._topic_name = k
                    self._logger.info('AUV wrench command topic found <%s>' % k)
                    break
            if self._topic_name is not None:
                break

        try:
            self._time = list()
            self._recorded_data['force'] = list()
            self._recorded_data['torque'] = list()
            self._recorded_data['surge_speed'] = list()
            for topic, msg, time in bag.read_messages(self._topic_name):
                    time = msg.header.stamp.to_sec()
                    self._time.append(time)
                    self._recorded_data['force'].append([msg.command.force.x, msg.command.force.y, msg.command.force.z])
                    self._recorded_data['torque'].append([msg.command.torque.x, msg.command.torque.y, msg.command.torque.z])
                    self._recorded_data['surge_speed'].append(float(msg.surge_speed))
            self._logger.info('%s=loaded' % self._topic_name)
        except Exception as e:
            self._logger.warning('Error reading AUV command input topic, message=' + str(e))

    def get_as_dataframe(self, add_group_name=None):
        try:
            import pandas

            if len(self._recorded_data['force']) == 0:
                return None

            data = dict()
            data[self.LABEL + '_time'] = self._time
            data[self.LABEL + '_force_x'] = [x[0] for x in self._recorded_data['force']]
            data[self.LABEL + '_force_y'] = [x[1] for x in self._recorded_data['force']]
            data[self.LABEL + '_force_z'] = [x[2] for x in self._recorded_data['force']]

            data[self.LABEL + '_torque_x'] = [x[0] for x in self._recorded_data['torque']]
            data[self.LABEL + '_torque_y'] = [x[1] for x in self._recorded_data['torque']]
            data[self.LABEL + '_torque_z'] = [x[2] for x in self._recorded_data['torque']]

            data[self.LABEL + '_surge_speed'] = self._recorded_data['surge_speed']

            if add_group_name is not None:
                data['group'] = [add_group_name for _ in range(len(self._time))]

            return pandas.DataFrame(data)

        except Exception as ex:
            self._logger.error('Error while exporting as pandas.DataFrame, message=' + str(ex))
            return None

    def plot(self, output_dir):
        if not os.path.isdir(output_dir):
            self._logger.error('Invalid output directory, dir=' + str(output_dir))
            raise Exception('Invalid output directory')
        
        try:
            fig_tm = None
            if len(self._recorded_data['force']):
                ##############################################################################
                # Plot AUV input command
                ##############################################################################            
                fig_tm, ax_tm = plt.subplots(3, 1,
                    figsize=(self._plot_configs['figsize'][0],
                    3 * self._plot_configs['figsize'][1]))

                self._logger.info('Plotting the input wrench of the thruster manager')

                min_y = 0
                max_y = 0
                
                ax_tm[0].plot(
                    self._time, 
                    [f[0] for f in self._recorded_data['force']],
                    color=COLOR_RED,
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$X$')

                min_y = min(min_y, np.min([f[0] for f in self._recorded_data['force']]))
                max_y = max(max_y, np.max([f[0] for f in self._recorded_data['force']]))
                
                ax_tm[0].plot(
                    self._time, 
                    [f[1] for f in self._recorded_data['force']],
                    color=COLOR_GREEN,
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$Y$')

                min_y = min(min_y, np.min([f[1] for f in self._recorded_data['force']]))
                max_y = max(max_y, np.max([f[1] for f in self._recorded_data['force']]))

                ax_tm[0].plot(
                    self._time, 
                    [f[2] for f in self._recorded_data['force']],
                    color=COLOR_BLUE,
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$Z$')

                min_y = min(min_y, np.min([f[2] for f in self._recorded_data['force']]))
                max_y = max(max_y, np.max([f[2] for f in self._recorded_data['force']]))

                ax_tm[0].set_xlim(np.min(self._time), np.max(self._time))            
                ax_tm[0].set_ylim(min_y, max_y)

                self.config_2dplot(
                    ax=ax_tm[0],
                    title='',
                    xlabel=r'Time [s]',
                    ylabel=r'Forces [N]',
                    legend_on=True)  

                min_y = 0
                max_y = 0

                ax_tm[1].plot(
                    self._time, 
                    [x[0] for x in self._recorded_data['torque']],
                    color=COLOR_RED,
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$K$')
                
                min_y = min(min_y, np.min([x[0] for x in self._recorded_data['torque']]))
                max_y = max(max_y, np.max([x[0] for x in self._recorded_data['torque']]))
                
                ax_tm[1].plot(
                    self._time, 
                    [x[1] for x in self._recorded_data['torque']],
                    color=COLOR_GREEN,
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$M$')
                
                min_y = min(min_y, np.min([x[1] for x in self._recorded_data['torque']]))
                max_y = max(max_y, np.max([x[1] for x in self._recorded_data['torque']]))
                
                ax_tm[1].plot(
                    self._time, 
                    [x[2] for x in self._recorded_data['torque']],
                    color=COLOR_BLUE,
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$N$')     

                min_y = min(min_y, np.min([x[2] for x in self._recorded_data['torque']]))
                max_y = max(max_y, np.max([x[2] for x in self._recorded_data['torque']]))               
                
                ax_tm[1].set_xlim(np.min(self._time), np.max(self._time))            
                ax_tm[1].set_ylim(min_y, max_y)

                self.config_2dplot(
                    ax=ax_tm[1],
                    title='',
                    xlabel=r'Time [s]',
                    ylabel=r'Torques [Nm]',
                    legend_on=True)  

                min_y = 0
                max_y = 0

                ax_tm[2].plot(self._time, self._recorded_data['surge_speed'],
                            linewidth=self._plot_configs['linewidth'],
                            label=r'$U$')
                min_y = np.min(self._recorded_data['surge_speed'])
                max_y = np.max(self._recorded_data['surge_speed'])
                
                ax_tm[2].set_xlim(np.min(self._time), np.max(self._time))            
                ax_tm[2].set_ylim(min_y, max_y)

                self.config_2dplot(
                    ax=ax_tm[2],
                    title='',
                    xlabel=r'Time [s]',
                    ylabel=r'Surge speed [m/s]',
                    legend_on=True) 

                plt.tight_layout()
                output_path = (self._output_dir if output_dir is None else output_dir)
                filename = os.path.join(output_path, 'auv_control_input.pdf')
                fig_tm.savefig(filename)
                plt.close(fig_tm)
                del fig_tm
        except Exception as e:
            self._logger.error('Error plotting AUV command input command wrench, message=' + str(e))
            if fig_tm is not None:
                plt.close(fig_tm)
                del fig_tm