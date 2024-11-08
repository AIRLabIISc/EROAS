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

try:
    plt.rc('text', usetex=True)
    plt.rc('font', family='sans-serif')
except Exception as e:
    print('Cannot use Latex configuration with matplotlib, message=', str(e))

class ThrusterManagerData(SimulationData):
    LABEL = 'thruster_manager'

    def __init__(self, bag):
        super(ThrusterManagerData, self).__init__()

        for x in bag.get_type_and_topic_info():
            for k in x:
                if 'thruster_manager' in k and 'geometry_msgs/WrenchStamped' in x[k][0]:
                    self._topic_name = k
                    self._logger.info('Thruster manager input topic found <%s>' % k)
                    break
            if self._topic_name is not None:
                break

        self._time = list()

        try:
            self._recorded_data['force'] = list()
            self._recorded_data['torque'] = list()
            # Find thruster manager input topic
            for topic, msg, time in bag.read_messages(self._topic_name):
                time = msg.header.stamp.to_sec()
                self._time.append(time)
                self._recorded_data['force'].append([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
                self._recorded_data['torque'].append([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])            
            self._logger.info('%s=loaded' % self._topic_name)
        except Exception as e:
            self._logger.error('Error retrieving thruster manager input wrench data from rosbag, message=' + str(e))
            self._recorded_data['force'] = None
            self._recorded_data['torque'] = None

    def get_as_dataframe(self, add_group_name=None):
        try:
            import pandas
            
            data = dict()
            data[self.LABEL + '_time'] = self._time
            data[self.LABEL + '_force_x'] = [x[0] for x in self._recorded_data['force']]
            data[self.LABEL + '_force_y'] = [x[1] for x in self._recorded_data['force']]
            data[self.LABEL + '_force_z'] = [x[2] for x in self._recorded_data['force']]

            data[self.LABEL + '_torque_x'] = [x[0] for x in self._recorded_data['torque']]
            data[self.LABEL + '_torque_y'] = [x[1] for x in self._recorded_data['torque']]
            data[self.LABEL + '_torque_z'] = [x[2] for x in self._recorded_data['torque']]

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

        fig_tm, ax_tm = plt.subplots(
                2, 1,
                figsize=(self._plot_configs['figsize'][0],
                         2 * self._plot_configs['figsize'][1]))
        try:
            ##############################################################################
            # Plot thruster manager input data
            ##############################################################################            
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

            ax_tm[0].set_xlim(np.min(t), np.max(t))            
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
            
            ax_tm[1].set_xlim(np.min(t), np.max(t))            
            ax_tm[1].set_ylim(min_y, max_y)

            self.config_2dplot(
                ax=ax_tm[1],
                title='',
                xlabel=r'Time [s]',
                ylabel=r'Torques [Nm]',
                legend_on=True)    

            plt.tight_layout()
            output_path = (self._output_dir if output_dir is None else output_dir)
            filename = os.path.join(output_path, 'thruster_manager_input.pdf')
            fig_tm.savefig(filename)
            plt.close(fig_tm)
            del fig_tm
        except Exception as e:
            self._logger.error('Error plotting thruster manager input command wrench, message=' + str(e))
            plt.close(fig_tm)
            del fig_tm