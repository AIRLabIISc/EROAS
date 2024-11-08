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


class ThrusterData(SimulationData):
    LABEL = 'thrusters'

    def __init__(self, bag):
        super(ThrusterData, self).__init__()
        
        for x in bag.get_type_and_topic_info():
            for k in x:
                if self._prefix is None:
                    if 'thrusters' in k:
                        self._logger.info('Thruster topic prefix found <%s>' % k)
                        i = len('thrusters')
                        i_max = k.find('thrusters') + i
                        self._prefix = k[:i_max]
                        break
            if self._prefix is not None:
                break

        try:
            self._recorded_data = dict()
            # Find all thruster output topics
            if self._prefix is not None:
                for i in range(16):
                    for topic, msg, time in bag.read_messages('%s/%d/thrust' % (self._prefix, i)):
                        if i not in self._recorded_data:
                            self._recorded_data[i] = dict(thrust=dict(time=list(), values=list()))
                        t = msg.header.stamp.to_sec()
                        self._recorded_data[i]['thrust']['time'].append(t)
                        self._recorded_data[i]['thrust']['values'].append(float(msg.data))
                    if i in self._recorded_data:
                        self._logger.info('%s/%d/thrust=loaded' % (self._prefix, i))
        except Exception as e:
            self._logger.warning('Error retrieving thrust output from rosbag, message=' + str(e))

        try:
            # Find all thruster input topics
            if self._prefix is not None:
                for i in range(16):
                    for topic, msg, time in bag.read_messages('%s/%d/input' % (self._prefix, i)):
                        if 'input' not in self._recorded_data[i]:
                            self._recorded_data[i]['input'] = dict(time=list(), values=list())
                        t = msg.header.stamp.to_sec()
                        self._recorded_data[i]['input']['time'].append(t)
                        self._recorded_data[i]['input']['values'].append(float(msg.data))
                    if i in self._recorded_data:
                        self._logger.info('%s/%d/input=loaded' % (self._prefix, i))
        except Exception as e:
            self._logger.warning('Error retrieving thruster input data from rosbag, message=' + str(e))

    def get_as_dataframe(self, add_group_name=None):
        try:
            import pandas

            data = dict()
            data[self.LABEL + '_id'] = list()
            data[self.LABEL + '_output_time'] = list()
            data[self.LABEL + '_output_values'] = list()
            if add_group_name is not None:
                data['group'] = list()
            for i in self._recorded_data:
                data[self.LABEL + '_id'] += [i for _ in range(len(self._recorded_data[i]['thrust']['time']))]
                if add_group_name is not None:
                    data['group'] += [add_group_name for _ in range(len(self._recorded_data[i]['thrust']['time']))]
                
                data[self.LABEL + '_output_time'] += self._recorded_data[i]['thrust']['time']
                data[self.LABEL + '_output_values'] += self._recorded_data[i]['thrust']['values']

            df_output = pandas.DataFrame(data)        

            data = dict()
            data[self.LABEL + '_id'] = list()
            data[self.LABEL + '_input_time'] = list()
            data[self.LABEL + '_input_values'] = list()
            if add_group_name is not None:
                data['group'] = list()
            for i in self._recorded_data:
                data[self.LABEL + '_id'] += [i for _ in range(len(self._recorded_data[i]['input']['time']))]
                if add_group_name is not None:
                    data['group'] += [add_group_name for _ in range(len(self._recorded_data[i]['input']['time']))]
                
                data[self.LABEL + '_input_time'] += self._recorded_data[i]['input']['time']
                data[self.LABEL + '_input_values'] += self._recorded_data[i]['input']['values']        

            df_input = pandas.DataFrame(data)

            return dict(input=df_input, output=df_output)

        except Exception as ex:
            self._logger.error('Error while exporting as pandas.DataFrame, message=' + str(ex))
            return None

    @property
    def n_thrusters(self):
        return len(self._recorded_data.keys())

    def get_input_data(self, idx):
        if idx < 0 or idx >= self.n_thrusters:
            return None
        return self._recorded_data[idx]['input']['time'], self._recorded_data[idx]['input']['values']

    def get_thrust_data(self, idx):
        if idx < 0 or idx >= self.n_thrusters:
            return None
        return self._recorded_data[idx]['thrust']['time'], self._recorded_data[idx]['thrust']['values']

    def plot(self, output_dir):
        if not os.path.isdir(output_dir):
            self._logger.error('Invalid output directory, dir=' + str(output_dir))
            raise Exception('Invalid output directory')

        fig, ax = plt.subplots(self.n_thrusters, 1,
                               figsize=(self._plot_configs['figsize'][0],
                                        self.n_thrusters * self._plot_configs['figsize'][1]))
        try:
            ##############################################################################
            # Plot individual thruster outputs
            ##############################################################################
            self._logger.info('# thrusters=%d' % self.n_thrusters)
            
            max_y = 0.0
            if self.n_thrusters > 1:
                for i in range(self.n_thrusters):
                    # Find largest absolute thrust force value
                    max_y = np.max([max_y, np.max(np.abs(self._recorded_data[i]['thrust']['values']))])

                    ax[i].plot(
                        self._recorded_data[i]['thrust']['time'],
                        self._recorded_data[i]['thrust']['values'],
                        linewidth=self._plot_configs['linewidth'],
                        label='%d' % i)

                    ax[i].set_xlim(
                        np.min(self._recorded_data[i]['thrust']['time']), 
                        np.max(self._recorded_data[i]['thrust']['time']))

                    self.config_2dplot(
                        ax=ax[i],
                        title='',
                        xlabel='Time [s]',
                        ylabel=r'$\tau_%d$ [N]' % i,
                        legend_on=False)                      

                for i in range(self.n_thrusters):
                    ax[i].set_ylim(-max_y, max_y)
            else:
                # Find largest absolute thrust force value
                max_y = np.max(np.abs(self._recorded_data[0]['thrust']['values']))

                ax.plot(
                    self._recorded_data[0]['thrust']['time'],
                    self._recorded_data[0]['thrust']['values'],
                    linewidth=self._plot_configs['linewidth'],
                    label='%d' % 0)

                ax.set_xlim(
                    np.min(self._recorded_data[0]['thrust']['time']), 
                    np.max(self._recorded_data[0]['thrust']['time']))

                ax.set_ylim(-max_y, max_y)

                self.config_2dplot(
                    ax=ax,
                    title='',
                    xlabel='Time [s]',
                    ylabel=r'$\tau_%d$ [N]' % 0,
                    legend_on=False)     


            plt.tight_layout()
            output_path = (self._output_dir if output_dir is None else output_dir)
            filename = os.path.join(output_path, 'thrusts.pdf')
            fig.savefig(filename)
            plt.close(fig)
            del fig
        except Exception as e:
            self._logger.error('Error plotting individual thruster outputs, message=' + str(e))
            plt.close(fig)
            del fig

        fig_all = self.get_figure()
        try:
            ##############################################################################
            # All thrust outputs
            ##############################################################################
            
            ax_all = fig_all.gca()

            for i in self._recorded_data:                
                ax_all.plot(self._recorded_data[i]['thrust']['time'],
                            self._recorded_data[i]['thrust']['values'],
                            linewidth=self._plot_configs['linewidth'],
                            label='%d' % i)

            ax_all.set_xlim(np.min(self._recorded_data[i]['thrust']['time']), np.max(self._recorded_data[i]['thrust']['time']))
            ax_all.set_ylim(-max_y, max_y)

            ax_all.legend(fancybox=True, framealpha=1,
                          loc=self._plot_configs['legend']['loc'],
                          fontsize=self._plot_configs['legend']['fontsize'])

            self.config_2dplot(
                ax=ax_all,
                title='',
                xlabel='Time [s]',
                ylabel=r'Thrust output [N]',
                legend_on=True)  

            plt.gcf().subplots_adjust(left=0.15, bottom=0.15)
            plt.tight_layout()

            output_path = (self._output_dir if output_dir is None else output_dir)
            filename = os.path.join(output_path, 'thrusts_all.pdf')
            fig_all.savefig(filename)
            plt.close(fig_all)
            del fig_all
        except Exception as e:
            self._logger.error('Error plotting all thruster output, message=' + str(e))
            plt.close(fig_all)
            del fig_all

        fig_avg = self.get_figure()
        try:
            ##############################################################################
            # Average thruster output
            ##############################################################################            
            ax_avg = fig_avg.gca()
            
            t0 = np.array(self._recorded_data[0]['thrust']['time'])
            thrust_sum = np.zeros(t0.shape)
            thrust_max = np.zeros(t0.shape)
            self._logger.info('Computing sum and maximum element-wise values for the thrust forces')
            for i in range(self.n_thrusters):                
                thrust_sum += np.interp(
                    t0, 
                    self._recorded_data[i]['thrust']['time'], 
                    np.abs(self._recorded_data[i]['thrust']['values']))
                thrust_max = np.maximum(
                    thrust_max, 
                    np.interp(
                        t0, 
                        self._recorded_data[i]['thrust']['time'], 
                        np.abs(self._recorded_data[i]['thrust']['values'])))

            thrust_sum /= self.n_thrusters
            ax_avg.plot(t0,
                        thrust_sum,
                        linewidth=self._plot_configs['linewidth'],
                        label=r'$%d$' % i)

            ax_avg.set_xlim(np.min(t0), np.max(t0))

            self.config_2dplot(
                ax=ax_avg,
                title='',
                xlabel='Time [s]',
                ylabel=r'$\frac{1}{N} \sum_{i=1}^{N} \tau_i$ [N]',
                legend_on=False) 

            plt.tight_layout()
            plt.gcf().subplots_adjust(left=0.15, bottom=0.15)

            output_path = (self._output_dir if output_dir is None else output_dir)
            filename = os.path.join(output_path, 'thrusts_avg.pdf')
            fig_avg.savefig(filename)
            plt.close(fig_avg)
            del fig_avg
        except Exception as e:
            self._logger.error('Error plotting average thrust force output, message=' + str(e))
            plt.close(fig_avg)
            del fig_avg

        fig_max = self.get_figure()
        try:
            ##############################################################################
            # Maximum thruster output for each time step
            ##############################################################################            
            ax_max = fig_max.gca()
            self._logger.info('Plotting maximum element-wise values for the thrust forces')
            ax_max.plot(t0,
                        thrust_max,
                        linewidth=self._plot_configs['linewidth'],
                        label=r'$%d$' % i)

            ax_max.set_xlim(np.min(t0), np.max(t0))

            self.config_2dplot(
                ax=ax_max,
                title='',
                xlabel='Time [s]',
                ylabel=r'max $| \tau_i |$ [N]',
                legend_on=True)  

            plt.tight_layout()
            plt.gcf().subplots_adjust(left=0.15, bottom=0.15)

            output_path = (self._output_dir if output_dir is None else output_dir)
            filename = os.path.join(output_path, 'thrusts_max.pdf')
            fig_max.savefig(filename)
            plt.close(fig_max)
            del fig_max
        except Exception as e:
            self._logger.error('Error plotting maximum thruster output, message=' + str(e))
            plt.close(fig_max)
            del fig_max

        fig_in = plt.figure(figsize=(self._plot_configs['figsize'][0],
                                     self._plot_configs['figsize'][1]))
        try:
            ##############################################################################
            # Plot thruster command input data
            ##############################################################################            
            ax_min = fig_in.gca()
            self._logger.info('Plotting the input command signals for each thruster unit')

            min_t = 0
            max_t = 0

            min_y = 0
            max_y = 0
            for i in range(self.n_thrusters):                
                ax_min.plot(
                    self._recorded_data[i]['input']['time'],
                    self._recorded_data[i]['input']['values'],
                    linewidth=self._plot_configs['linewidth'],
                    label='%d' % i)
                max_t = max(max_t, np.max(self._recorded_data[i]['input']['time']))
                min_y = min(min_y, np.min(self._recorded_data[i]['input']['values']))
                max_y = max(max_y, np.max(self._recorded_data[i]['input']['values']))
            
            ax_min.set_xlim(min_t, max_t)            
            ax_min.set_ylim(min_y, max_y)

            self.config_2dplot(
                ax=ax_min,
                title='',
                xlabel='Time [s]',
                ylabel=r'Thrust input [rad/s]',
                legend_on=True)  

            plt.tight_layout()
            output_path = (self._output_dir if output_dir is None else output_dir)
            filename = os.path.join(output_path, 'thruster_input.pdf')
            fig_in.savefig(filename)
            plt.close(fig_in)
            del fig_in
        except Exception as e:
            self._logger.error('Error plotting thruster input command, message=' + str(e))
            plt.close(fig_in)
            del fig_in

