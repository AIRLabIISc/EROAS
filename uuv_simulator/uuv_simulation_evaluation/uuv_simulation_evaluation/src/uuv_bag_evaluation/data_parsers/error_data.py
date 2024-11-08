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
import uuv_bag_evaluation.error 
from uuv_trajectory_generator import TrajectoryGenerator, TrajectoryPoint


class ErrorData(SimulationData):
    LABEL = 'error'

    def __init__(self, bag):
        super(ErrorData, self).__init__(message_type='uuv_control_msgs/TrajectoryPoint')

        for x in bag.get_type_and_topic_info():
            for k in x:
                if 'error' in k and self._message_type in x[k][0]:
                    self._topic_name = k
                    self._logger.info('Error topic found <%s>' % k)
                    break
            if self._topic_name is not None:
                break

        try:
            self._recorded_data['error'] = TrajectoryGenerator()
            for topic, msg, time in bag.read_messages(self._topic_name):
                self._recorded_data['error'].add_trajectory_point_from_msg(msg)
            self._logger.info('%s=loaded' % self._topic_name)
        except Exception as e:
            self._logger.warning('Error retrieving error data from rosbag, message=' + str(e))
            self._recorded_data['error'] = None

        self._error_set = None

    def get_as_dataframe(self, add_group_name=None):
        try:
            import pandas

            # Create error set object
            if self._error_set is None:
                self._error_set = uuv_bag_evaluation.error.ErrorSet.get_instance()

            data = dict()
            data['time'] = self._error_set.get_time()            

            for tag in self._error_set.get_tags():
                if tag == 'cross_track':
                    continue
                elif tag in ['position', 'linear_velocity', 'angular_velocity', 'quaternion']:
                    data[tag + '_x'] = [x[0] for x in self._error_set.get_data(tag)]
                    data[tag + '_y'] = [x[1] for x in self._error_set.get_data(tag)]
                    data[tag + '_z'] = [x[2] for x in self._error_set.get_data(tag)]
                else:
                    data[tag] = self._error_set.get_data(tag)

            if add_group_name is not None:
                data['group'] = [add_group_name for _ in range(len(data['time']))]            

            df_error = pandas.DataFrame(data)

            data = dict()
            data[tag + 'cross_track_time'] = self._error_set.get_time('desired')
            data[tag + 'cross_track'] = self._error_set.get_data('cross_track')
            
            if add_group_name is not None:
                data['group'] = [add_group_name for _ in range(len(data['time']))]

            df_error_ct = pandas.DataFrame(data)

            return dict(error=df_error, error_cross_track=df_error_ct)

        except Exception as ex:
            self._logger.error('Error while exporting as pandas.DataFrame, message=' + str(ex))
            return None

    @property
    def error(self):
        return self._recorded_data['error']
    
    def plot(self, output_dir):
        if not os.path.isdir(output_dir):
            self._logger.error('Invalid output directory, dir=' + str(output_dir))
            raise rospy.ROSException('Invalid output directory')
        
        try:
            # Create error set object
            self._error_set = uuv_bag_evaluation.error.ErrorSet.get_instance()

            if self._error_set is None:
                self._logger.info('Error set has not been correctly initialized')
                raise rospy.ROSException('Error set has not been correctly initialized')

            ##################################################################################
            # Plotting position and orientation errors
            ##################################################################################
            output_path = (self._output_dir if output_dir is None else output_dir)

            t = self._error_set.get_time()
            
            fig = self.get_figure()        
            ax = fig.gca()
            
            ax.plot(
                t, 
                self._error_set.get_data('x'), 
                color=COLOR_RED, 
                label=r'$X$',
                linewidth=self._plot_configs['linewidth'])
            ax.plot(
                t, 
                self._error_set.get_data('y'), 
                color=COLOR_GREEN, 
                label=r'$Y$',
                linewidth=self._plot_configs['linewidth'])
            ax.plot(
                t, 
                self._error_set.get_data('z'), 
                color=COLOR_BLUE, 
                label=r'$Z$',
                linewidth=self._plot_configs['linewidth'])
            
            self.config_2dplot(
                ax=ax,
                title='Position error',
                xlabel='Time [s]',
                ylabel='Error [m]',
                legend_on=True)  

            ax.set_xlim(np.min(t), np.max(t))

            plt.tight_layout()
            plt.savefig(os.path.join(output_path, 'errors_position.pdf'))
            plt.close(fig)
            del fig

            fig = self.get_figure()        
            ax = fig.gca()
            
            ax.plot(
                t, 
                self._error_set.get_data('roll'), 
                color=COLOR_RED, 
                label=r'$\phi$',
                linewidth=self._plot_configs['linewidth'])
            ax.plot(
                t, 
                self._error_set.get_data('pitch'), 
                color=COLOR_GREEN, 
                label=r'$\theta$',
                linewidth=self._plot_configs['linewidth'])
            ax.plot(
                t, 
                self._error_set.get_data('yaw'), 
                color=COLOR_BLUE, 
                label=r'$\psi$',
                linewidth=self._plot_configs['linewidth'])
            
            ax.set_xlim(np.min(t), np.max(t))

            self.config_2dplot(
                ax=ax,
                title='Orientation error',
                xlabel='Time [s]',
                ylabel='Error [rad]',
                legend_on=True)  

            plt.tight_layout()
            plt.savefig(os.path.join(output_path, 'errors_orientation.pdf'))
            plt.close(fig)
            del fig
        except Exception as e:
            self._logger.error('Error while plotting pose errors, message=' + str(e))
            plt.close(fig)
            del fig

        try:
            output_path = (self._output_dir if output_dir is None else output_dir)
            
            fig = self.get_figure()        
            ax = fig.gca()

            error = np.sqrt([e.dot(e) for e in self._error_set.get_data('position')]) 

            # self.add_disturbance_activation_spans(ax, 0, error.max())

            t = self._error_set.get_time()
            ax.plot(
                t, 
                error, 
                linewidth=self._plot_configs['linewidth'],
                color=COLOR_RED, 
                label=r'Euc. position error')

            ax.set_xlim(np.min(t), np.max(t))
            ax.set_ylim(0, np.max(error) * 1.05)

            self.config_2dplot(
                ax=ax,
                title='',
                xlabel='Time [s]',
                ylabel='Euc. position error [m]',
                legend_on=False)  

            plt.tight_layout()
            plt.savefig(os.path.join(output_path, 'error_euc_position.pdf'))
            plt.close(fig)
            del fig

            # Plot heading error
            fig = self.get_figure()        
            ax = fig.gca()

            error = self._error_set.get_data('yaw')

            # self.add_disturbance_activation_spans(ax, np.min(error), np.max(error))

            t = self._error_set.get_time()
            ax.plot(
                t, 
                error, 
                color=COLOR_RED,
                linewidth=self._plot_configs['linewidth'],
                label='Heading error')

            ax.set_xlim(np.min(t), np.max(t))
            ax.set_ylim(- np.pi, np.pi)

            self.config_2dplot(
                ax=ax,
                title='',
                xlabel='Time [s]',
                ylabel='Heading error [rad]',
                legend_on=False)            

            plt.tight_layout()
            plt.savefig(os.path.join(output_path, 'error_heading.pdf'))
            plt.close(fig)
            del fig
        except Exception as e:
            self._logger.error('Error while plotting position and heading error plots, message=' + str(e))
            plt.close(fig)
            del fig

        try:
            ##################################################################################
            # Plotting velocity errors
            ##################################################################################                        
            fig = self.get_figure()        
            ax = fig.gca()

            ax.plot(
                t, 
                [e[0] for e in self._error_set.get_data('linear_velocity')], 
                label=r'$\dot{X}$',
                linewidth=self._plot_configs['linewidth'],
                color=COLOR_RED)
            ax.plot(
                t, 
                [e[1] for e in self._error_set.get_data('linear_velocity')], 
                label=r'$\dot{Y}$',
                linewidth=self._plot_configs['linewidth'],
                color=COLOR_GREEN)
            ax.plot(
                t, 
                [e[2] for e in self._error_set.get_data('linear_velocity')], 
                label=r'$\dot{Z}$',
                linewidth=self._plot_configs['linewidth'],
                color=COLOR_BLUE)
            
            self.config_2dplot(
                ax=ax,
                title='',
                xlabel='Time [s]',
                ylabel='Error [m/s]',
                legend_on=True)

            ax.set_xlim(np.min(t), np.max(t))

            plt.tight_layout()
            plt.savefig(os.path.join(output_path, 'errors_lin_vel.pdf'))
            plt.close(fig)
            del fig

            fig = self.get_figure()        
            ax = fig.gca()

            ax.plot(
                t, 
                [e[0] for e in self._error_set.get_data('angular_velocity')], 
                label=r'$\omega_x$',
                linewidth=self._plot_configs['linewidth'],
                color=COLOR_RED)
            ax.plot(
                t, 
                [e[1] for e in self._error_set.get_data('angular_velocity')], 
                label=r'$\omega_y$',
                linewidth=self._plot_configs['linewidth'],
                color=COLOR_GREEN)
            ax.plot(
                t, 
                [e[2] for e in self._error_set.get_data('angular_velocity')], 
                label=r'$\omega_z$',
                linewidth=self._plot_configs['linewidth'],
                color=COLOR_BLUE)
            
            ax.set_xlim(np.min(t), np.max(t))

            self.config_2dplot(
                ax=ax,
                title='',
                xlabel='Time [s]',
                ylabel='Error [rad/s]',
                legend_on=True)

            plt.tight_layout()
            plt.savefig(os.path.join(output_path, 'errors_ang_vel.pdf'))
            plt.close(fig)
            del fig
        except Exception as e:
            self._logger.error('Error while plotting velocity errors, message=' + str(e))
            plt.close(fig)
            del fig
        
        try:    
            ##################################################################################
            # Plotting quaternion vector errors
            ##################################################################################

            t = self._error_set.get_time()            
            fig = self.get_figure()        
            ax = fig.gca()

            ax.plot(
                t, 
                [e[0] for e in self._error_set.get_data('quaternion')], 
                label=r'$\epsilon_x$',
                linewidth=self._plot_configs['linewidth'],
                color=COLOR_RED)
            ax.plot(
                t, 
                [e[1] for e in self._error_set.get_data('quaternion')], 
                label=r'$\epsilon_y$',
                linewidth=self._plot_configs['linewidth'],
                color=COLOR_GREEN)
            ax.plot(
                t, 
                [e[2] for e in self._error_set.get_data('quaternion')], 
                label=r'$\epsilon_z$',
                linewidth=self._plot_configs['linewidth'],
                color=COLOR_BLUE)

            self.config_2dplot(
                ax=ax,
                title='',
                xlabel='Time [s]',
                ylabel='Error',
                legend_on=True)
            ax.set_xlim(np.min(t), np.max(t))

            plt.tight_layout()
            plt.savefig(os.path.join(output_path, 'errors_quat.pdf'))
            plt.close(fig)
            del fig
        except Exception as e:
            self._logger.error('Error while plotting quaternion vector error, message=' + str(e))
            plt.close(fig)
            del fig

        try:    
            ##################################################################################
            # Plotting cross-track errors
            ##################################################################################

            t = self._error_set.get_time('desired')
            
            fig = self.get_figure()        
            ax = fig.gca()

            ax.set_title(
                'Cross-track error', 
                fontsize=self._plot_configs['title_fontsize'])
            ax.plot(
                t, 
                self._error_set.get_data('cross_track'), 
                linewidth=self._plot_configs['linewidth'],
                color=COLOR_RED)
            self.config_2dplot(
                ax=ax,
                title='',
                xlabel='Time [s]',
                ylabel='Error [m]',
                legend_on=False)
            
            ax.set_xlim(np.min(t), np.max(t))

            plt.tight_layout()
            plt.savefig(os.path.join(output_path, 'errors_cross_track.pdf'))
            plt.close(fig)
            del fig
        except Exception as e:
            self._logger.error('Error while plotting cross-track error, message=' + str(e))
            plt.close(fig)
            del fig