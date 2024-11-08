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


class ConcentrationSensorData(SimulationData):
    LABEL = 'concentration_sensor'

    def __init__(self, bag):
        super(ConcentrationSensorData, self).__init__()

        for x in bag.get_type_and_topic_info():
            for k in x:
                if 'uuv_sensor_plugins_ros_msgs/ChemicalParticleConcentration' in x[k][0]:
                    self._topic_name = k
                    self._logger.info('Particle concentration topic found <%s>' % k)
                    break
            if self._topic_name is not None:
                break

        self._time = list()

        try:
            self._recorded_data['conc'] = list()
            self._recorded_data['pos'] = list()
            for topic, msg, time in bag.read_messages(self._topic_name):
                    time = msg.header.stamp.to_sec()
                    self._time.append(time)
                    self._recorded_data['conc'].append(msg.concentration)
                    self._recorded_data['pos'].append([msg.position.x, msg.position.y, msg.position.z])
            self._logger.info('%s=loaded' % self._topic_name)
        except Exception as e:
            self._logger.warning('Error reading particle concentration topic, message=' + str(e))

    def get_as_dataframe(self, add_group_name=None):
        try:
            import pandas

            if len(self._recorded_data['conc']) == 0:
                return None

            data = dict(
                concentration_time=self._time,
                concentration=self._recorded_data['conc'],
                concentration_pos_x=[x[0] for x in self._recorded_data['pos']],
                concentration_pos_y=[x[1] for x in self._recorded_data['pos']],
                concentration_pos_z=[x[2] for x in self._recorded_data['pos']])

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
            fig = None
            output_path = (self._output_dir if output_dir is None else output_dir)            

            if len(self._recorded_data['conc']):            
                fig = self.get_figure()        
                ax = fig.gca()

                ax.plot(
                    self._time, 
                    self._recorded_data['conc'], 
                    color=COLOR_RED, 
                    label=r'Particle concentration',
                    linewidth=self._plot_configs['linewidth'])
                                
                ax.set_ylim([np.min(self._recorded_data['conc']) - 0.1, 
                            np.max(self._recorded_data['conc']) + 0.1])

                self.config_2dplot(
                    ax=ax,
                    title='',
                    xlabel=r'Time [s]',
                    ylabel=r'Particle concentration',
                    legend_on=False)
                
                ax.grid(True)
                plt.autoscale(enable=True, axis='x', tight=True)

                plt.tight_layout()
                plt.savefig(os.path.join(output_path, 'particle_conc.pdf'))
                plt.close(fig)
                del fig
                
                if len(self._time) == 0:
                    self._logger.error('No particle concentration information found')
                    return            
        except Exception as e:
            self._logger.error('Error while plotting particle concentration, message=' + str(e))
            if fig is not None:
                plt.close(fig)
                del fig

        try:       
            fig = None     
            if len(self._recorded_data['pos']):
                fig = self.get_figure()       
                ax = fig.gca(projection='3d')

                x = [p[0] for p in self._recorded_data['pos']]
                y = [p[1] for p in self._recorded_data['pos']]
                z = [p[2] for p in self._recorded_data['pos']]

                p = ax.scatter(x, y, z, c=self._recorded_data['conc'])
                
                # Calculating the boundaries of the paths
                min_x = np.min(x)
                max_x = np.max(x)

                min_y = np.min(y)
                max_y = np.max(y)

                min_z = np.min(z)
                max_z = np.max(z)

                ax.set_xlabel('X [m]',
                            fontsize=self._plot_configs['label_fontsize'])
                ax.set_ylabel('Y [m]',
                            fontsize=self._plot_configs['label_fontsize'])
                ax.set_zlabel('Z [m]',
                            fontsize=self._plot_configs['label_fontsize'])

                if self._plot_configs['xlim'] is not None:
                    ax.set_xlim(self._plot_configs['xlim'][0],
                                self._plot_configs['xlim'][1])
                else:
                    ax.set_xlim(min_x - 1, max_x + 1)

                if self._plot_configs['ylim'] is not None:
                    ax.set_ylim(self._plot_configs['ylim'][0],
                                self._plot_configs['ylim'][1])
                else:
                    ax.set_ylim(min_y - 1, max_y + 1)

                if self._plot_configs['zlim'] is not None:
                    ax.set_zlim(self._plot_configs['zlim'][0],
                                self._plot_configs['zlim'][1])
                else:
                    ax.set_zlim(min_z - 1, max_z + 1)

                ax.tick_params(
                    axis='x',
                    labelsize=self._plot_configs['tick_labelsize'])
                ax.tick_params(
                    axis='y',
                    labelsize=self._plot_configs['tick_labelsize'])
                ax.tick_params(
                    axis='z',
                    labelsize=self._plot_configs['tick_labelsize'])

                ax.xaxis.labelpad = self._plot_configs['labelpad']
                ax.yaxis.labelpad = self._plot_configs['labelpad']
                ax.zaxis.labelpad = self._plot_configs['labelpad']

                cbar = fig.colorbar(p)
                cbar.ax.tick_params(labelsize=self._plot_configs['tick_labelsize'])
                ax.grid(True)
                plt.tight_layout()

                # Invert axis if the pose of the vehicle is represented wrt NED
                # inertial reference frame
                if min_z >= 0 and max_z >= 0:
                    plt.gca().invert_zaxis()

                ax.view_init(elev=15, azim=30)

                output_path = (self._output_dir if output_dir is None else output_dir)
                filename = os.path.join(output_path, 'particle_conc_3d.pdf')
                plt.savefig(filename)
                plt.close(fig)
        except Exception as e:
            self._logger.error('Error plotting 3D particle concentration, message=' + str(e))
            if fig is not None:
                plt.close(fig)
                del fig
