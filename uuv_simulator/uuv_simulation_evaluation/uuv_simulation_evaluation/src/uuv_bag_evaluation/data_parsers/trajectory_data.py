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
from mpl_toolkits.mplot3d import Axes3D
from simulation_data import SimulationData, COLOR_RED, COLOR_GREEN, COLOR_BLUE
from uuv_trajectory_generator import TrajectoryGenerator, TrajectoryPoint

try:
    plt.rc('text', usetex=True)
    plt.rc('font', family='sans-serif')
except Exception as e:
    print('Cannot use Latex configuration with matplotlib, message=', str(e))


class TrajectoryData(SimulationData):
    LABEL = 'trajectory'

    def __init__(self, bag):
        super(TrajectoryData, self).__init__(message_type='nav_msgs/Odometry')

        self._topic_name = dict()
        for x in bag.get_type_and_topic_info():
            for k in x:
                if 'nav_msgs/Odometry' in x[k][0]:
                    self._topic_name['odometry'] = k
                    self._logger.info('Odometry topic found <%s>' % k)
                    
                if 'reference' in k and 'uuv_control_msgs/TrajectoryPoint' in x[k][0]:
                    self._topic_name['reference'] = k
                    self._logger.info('Trajectory topic found <%s>' % k)

        try:
            self._recorded_data['desired'] = TrajectoryGenerator()
            for topic, msg, time in bag.read_messages(self._topic_name['reference']):
                self._recorded_data['desired'].add_trajectory_point_from_msg(msg)
            self._logger.info('%s=loaded' % self._topic_name['reference'])
        except Exception as e:
            self._logger.error('Error trajectories from rosbag, message=' + str(e))
            self._recorded_data['desired'] = None

        try:
            self._recorded_data['actual'] = TrajectoryGenerator()
            for topic, msg, time in bag.read_messages(self._topic_name['odometry']):
                t = msg.header.stamp.to_sec()

                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                v = msg.twist.twist.linear
                w = msg.twist.twist.angular

                point = TrajectoryPoint(
                    t, np.array([p.x, p.y, p.z]),
                    np.array([q.x, q.y, q.z, q.w]),
                    np.array([v.x, v.y, v.z]),
                    np.array([w.x, w.y, w.z]),
                    np.array([0, 0, 0]),
                    np.array([0, 0, 0]))
                # Store sampled trajectory point
                self._recorded_data['actual'].add_trajectory_point(point)
            self._logger.info('%s=loaded' % self._topic_name['odometry'])
        except Exception as e:
            self._logger.error('Error retrieving odometry data from rosbag, message=' + str(e))
            self._recorded_data['actual'] = None

    def get_as_dataframe(self, add_group_name=None):
        try:
            import pandas

            data = dict()

            data[self.LABEL + '_ref_time'] = self._recorded_data['desired'].time            

            for i, tag in zip(range(3), ['x', 'y', 'z']):                
                data[self.LABEL + '_pos_ref_' + tag] = \
                    [e.p[i] for e in self._recorded_data['desired'].points]

                data[self.LABEL + '_lin_vel_ref_' + tag] = \
                    [e.vel[i] for e in self._recorded_data['desired'].points]

                data[self.LABEL + '_ang_vel_ref_' + tag] = \
                    [e.vel[i + 3] for e in self._recorded_data['desired'].points]
            
            for i, tag in zip(range(3), ['roll', 'pitch', 'yaw']):
                data[self.LABEL + '_rot_ref_' + tag] = \
                    [e.rot[i] for e in self._recorded_data['desired'].points]            
            
            for i, tag in zip(range(4), ['x', 'y', 'z', 'w']):
                data[self.LABEL + '_rotq_ref_' + tag] = \
                    [e.rotq[i] for e in self._recorded_data['desired'].points]            
            
            if add_group_name is not None:
                data['group'] = [add_group_name for _ in range(len(self._recorded_data['desired'].points))]

            df_ref = pandas.DataFrame(data)

            data = dict()

            data[self.LABEL + '_actual_time'] = self._recorded_data['actual'].time
            
            for i, tag in zip(range(3), ['x', 'y', 'z']):                
                data[self.LABEL + '_pos_actual_' + tag] = \
                    [e.p[i] for e in self._recorded_data['actual'].points]
                data[self.LABEL + '_lin_vel_actual_' + tag] = \
                    [e.vel[i] for e in self._recorded_data['actual'].points]
                data[self.LABEL + '_ang_vel_actual_' + tag] = \
                    [e.vel[i + 3] for e in self._recorded_data['actual'].points]
            
            for i, tag in zip(range(3), ['roll', 'pitch', 'yaw']):
                data[self.LABEL + '_rot_actual_' + tag] = \
                    [e.rot[i] for e in self._recorded_data['actual'].points]
            
            for i, tag in zip(range(4), ['x', 'y', 'z', 'w']):
                data[self.LABEL + '_rotq_actual_' + tag] = \
                    [e.rotq[i] for e in self._recorded_data['actual'].points]
            
            if add_group_name is not None:
                data['group'] = [add_group_name for _ in range(len(self._recorded_data['actual'].points))]

            df_actual = pandas.DataFrame(data)

            return dict(ref=df_ref, actual=df_actual)
        except Exception as ex:
            print('Error while exporting as pandas.DataFrame, message=' + str(ex))
            return None

    @property
    def start_time(self):
        if self._recorded_data['desired'] is None:
            return None
        else:
            return self._recorded_data['desired'].points[0].t

    @property
    def end_time(self):
        if self._recorded_data['desired'] is None:
            return None
        else:
            return self._recorded_data['desired'].points[-1].t

    @property
    def reference(self):
        return self._recorded_data['desired']

    @property
    def odometry(self):
        return self._recorded_data['actual']

    def plot(self, output_dir):
        if not os.path.isdir(output_dir):
            self._logger.error('Invalid output directory, dir=' + str(output_dir))
            raise Exception('Invalid output directory')

        try:            
            fig = self.get_figure(n_rows=2)
            ax = fig.gca(projection='3d')

            ax.plot([e.p[0] for e in self._recorded_data['desired'].points],
                    [e.p[1] for e in self._recorded_data['desired'].points],
                    [e.p[2] for e in self._recorded_data['desired'].points],
                    color=COLOR_BLUE, 
                    linestyle='dashed',
                    label='Reference path',
                    linewidth=self._plot_configs['linewidth'])

            ax.plot([e.p[0] for e in self._recorded_data['actual'].points],
                    [e.p[1] for e in self._recorded_data['actual'].points],
                    [e.p[2] for e in self._recorded_data['actual'].points],
                    color=COLOR_GREEN, 
                    label='Actual path',
                    linewidth=self._plot_configs['linewidth'])

            ax.plot([self._recorded_data['actual'].points[0].p[0]],
                    [self._recorded_data['actual'].points[0].p[1]],
                    [self._recorded_data['actual'].points[0].p[2]],
                    color=COLOR_RED, 
                    marker='o',
                    linestyle='None',
                    label='Starting position',
                    linewidth=self._plot_configs['linewidth'])

            # Calculating the boundaries of the paths
            min_x = np.min([np.min([e.p[0] for e in self._recorded_data['desired'].points]),
                            np.min([e.p[0] for e in self._recorded_data['actual'].points])])

            max_x = np.max([np.max([e.p[0] for e in self._recorded_data['desired'].points]),
                            np.max([e.p[0] for e in self._recorded_data['actual'].points])])

            min_y = np.min([np.min([e.p[1] for e in self._recorded_data['desired'].points]),
                            np.min([e.p[1] for e in self._recorded_data['actual'].points])])

            max_y = np.max([np.max([e.p[1] for e in self._recorded_data['desired'].points]),
                            np.max([e.p[1] for e in self._recorded_data['actual'].points])])

            min_z = np.min([np.min([e.p[2] for e in self._recorded_data['desired'].points]),
                            np.min([e.p[2] for e in self._recorded_data['actual'].points])])

            max_z = np.max([np.max([e.p[2] for e in self._recorded_data['desired'].points]),
                            np.max([e.p[2] for e in self._recorded_data['actual'].points])])

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

            ax.tick_params(axis='x',
                           labelsize=self._plot_configs['tick_labelsize'])
            ax.tick_params(axis='y',
                           labelsize=self._plot_configs['tick_labelsize'])
            ax.tick_params(axis='z',
                           labelsize=self._plot_configs['tick_labelsize'])

            ax.xaxis.labelpad = 30
            ax.yaxis.labelpad = 30
            ax.zaxis.labelpad = 30

            leg = ax.legend(
                fancybox=True, 
                framealpha=1, 
                loc=self._plot_configs['legend']['loc'], 
                fontsize=self._plot_configs['legend']['fontsize'])
            leg.get_frame().set_facecolor('white')

            ax.grid(True)
            plt.tight_layout()

            # Invert axis if the pose of the vehicle is represented wrt NED
            # inertial reference frame
            if min_z >= 0 and max_z >= 0:
                plt.gca().invert_zaxis()

            ax.view_init(elev=15, azim=30)

            output_path = (self._output_dir if output_dir is None else output_dir)
            filename = os.path.join(output_path, 'paths.pdf')
            plt.savefig(filename)
            plt.close(fig)
            del fig
        except Exception as e:
            self._logger.error('Error while plotting 3D path plot, message=' + str(e))
            plt.close(fig)
            del fig
        
        try:
            output_path = (self._output_dir if output_dir is None else output_dir)
                        
            fig = self.get_figure()        
            ax = fig.gca()

            min_value = np.min([np.min([e.pos[0] for e in self._recorded_data['actual'].points]),
                                np.min([e.pos[0] for e in self._recorded_data['desired'].points]),
                                np.min([e.pos[1] for e in self._recorded_data['actual'].points]),
                                np.min([e.pos[1] for e in self._recorded_data['desired'].points]),
                                np.min([e.pos[2] for e in self._recorded_data['actual'].points]),
                                np.min([e.pos[2] for e in self._recorded_data['desired'].points])])

            max_value = np.max([np.max([e.pos[0] for e in self._recorded_data['actual'].points]),
                                np.max([e.pos[0] for e in self._recorded_data['desired'].points]),
                                np.max([e.pos[1] for e in self._recorded_data['actual'].points]),
                                np.max([e.pos[1] for e in self._recorded_data['desired'].points]),
                                np.max([e.pos[2] for e in self._recorded_data['actual'].points]),
                                np.max([e.pos[2] for e in self._recorded_data['desired'].points])])

        #     self.add_disturbance_activation_spans(ax, min_value, max_value)

            ax.plot(self._recorded_data['desired'].time, 
                    [e.pos[0] for e in self._recorded_data['desired'].points], 
                    color=COLOR_RED,
                    linestyle='dashed',
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$X_d$')
            ax.plot(self._recorded_data['desired'].time, 
                    [e.pos[1] for e in self._recorded_data['desired'].points], 
                    color=COLOR_GREEN,
                    linestyle='dashed',
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$Y_d$')
            ax.plot(self._recorded_data['desired'].time, 
                    [e.pos[2] for e in self._recorded_data['desired'].points], 
                    color=COLOR_BLUE,
                    linestyle='dashed',
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$Z_d$')

            ax.plot(self._recorded_data['actual'].time, 
                    [e.pos[0] for e in self._recorded_data['actual'].points], 
                    color=COLOR_RED,
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$X$')
            ax.plot(self._recorded_data['actual'].time, 
                    [e.pos[1] for e in self._recorded_data['actual'].points], 
                    color=COLOR_GREEN,
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$Y$')
            ax.plot(self._recorded_data['actual'].time, 
                    [e.pos[2] for e in self._recorded_data['actual'].points], 
                    color=COLOR_BLUE,
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$Z$')

            ax.set_xlim(
                np.min(self._recorded_data['desired'].time), 
                np.max(self._recorded_data['desired'].time))
            ax.set_ylim(1.05 * min_value, 1.05 * max_value)

            self.config_2dplot(
                ax=ax,
                title='',
                xlabel=r'Time [s]',
                ylabel=r'Position [m]',
                legend_on=True)              

            plt.tight_layout()
            plt.savefig(os.path.join(output_path, 'trajectories_position.pdf'))
            plt.close(fig)
            del fig

            fig = self.get_figure()        
            ax = fig.gca()

            min_value = np.min([np.min([e.rot[0] for e in self._recorded_data['actual'].points]),
                                np.min([e.rot[0] for e in self._recorded_data['desired'].points]),
                                np.min([e.rot[1] for e in self._recorded_data['actual'].points]),
                                np.min([e.rot[1] for e in self._recorded_data['desired'].points]),
                                np.min([e.rot[2] for e in self._recorded_data['actual'].points]),
                                np.min([e.rot[2] for e in self._recorded_data['desired'].points])])

            max_value = np.max([np.max([e.rot[0] for e in self._recorded_data['actual'].points]),
                                np.max([e.rot[0] for e in self._recorded_data['desired'].points]),
                                np.max([e.rot[1] for e in self._recorded_data['actual'].points]),
                                np.max([e.rot[1] for e in self._recorded_data['desired'].points]),
                                np.max([e.rot[2] for e in self._recorded_data['actual'].points]),
                                np.max([e.rot[2] for e in self._recorded_data['desired'].points])])

        #     self.add_disturbance_activation_spans(ax, min_value, max_value)
            
            ax.plot(self._recorded_data['desired'].time, 
                    [e.rot[0] for e in self._recorded_data['desired'].points],                     
                    color=COLOR_RED,
                    linestyle='dashed',
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\phi_d$')
            ax.plot(self._recorded_data['desired'].time, 
                    [e.rot[1] for e in self._recorded_data['desired'].points],                     
                    color=COLOR_GREEN,
                    linestyle='dashed',
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\theta_d$')
            ax.plot(self._recorded_data['desired'].time, 
                    [e.rot[2] for e in self._recorded_data['desired'].points], 
                    color=COLOR_BLUE,
                    linestyle='dashed',
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\psi_d$')

            ax.plot(self._recorded_data['actual'].time, 
                    [e.rot[0] for e in self._recorded_data['actual'].points], 
                    color=COLOR_RED,
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\phi$')
            ax.plot(self._recorded_data['actual'].time, 
                    [e.rot[1] for e in self._recorded_data['actual'].points], 
                    color=COLOR_GREEN,
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\theta$')
            ax.plot(self._recorded_data['actual'].time, 
                    [e.rot[2] for e in self._recorded_data['actual'].points], 
                    color=COLOR_BLUE,
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\psi$')

            ax.set_xlim(
                np.min(self._recorded_data['desired'].time), 
                np.max(self._recorded_data['desired'].time))
            ax.set_ylim(1.05 * min_value, 1.05 * max_value)

            self.config_2dplot(
                ax=ax,
                title='',
                xlabel=r'Time [s]',
                ylabel=r'Angles [rad]',
                legend_on=True)   

            plt.tight_layout()
            plt.savefig(os.path.join(output_path, 'trajectories_orientation.pdf'))
            plt.close(fig)
            del fig
        except Exception as e:
            self._logger.error('Error plotting output pose vectors, error=' + str(e))
            plt.close(fig)
            del fig

        try:
            ###################################################################
            # Plot quaternion trajectories
            ###################################################################            
            fig = self.get_figure()        
            ax = fig.gca()

            ax.plot(self._recorded_data['desired'].time, 
                    [e.rotq[0] for e in self._recorded_data['desired'].points], 
                    color=COLOR_RED,
                    linestyle='dashed',
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$\epsilon_{x_d}$')
            ax.plot(self._recorded_data['desired'].time, 
                    [e.rotq[1] for e in self._recorded_data['desired'].points], 
                    color=COLOR_GREEN,
                    linestyle='dashed',
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$\epsilon_{y_d}$')
            ax.plot(self._recorded_data['desired'].time, 
                    [e.rotq[2] for e in self._recorded_data['desired'].points], 
                    color=COLOR_BLUE,
                    linestyle='dashed',
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$\epsilon_{z_d}$')

            ax.plot(self._recorded_data['actual'].time, 
                    [e.rotq[0] for e in self._recorded_data['actual'].points], 
                    color=COLOR_RED,
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$\epsilon_{x}$')
            ax.plot(self._recorded_data['actual'].time, 
                    [e.rotq[1] for e in self._recorded_data['actual'].points], 
                    color=COLOR_GREEN,
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$\epsilon_{y}$')
            ax.plot(self._recorded_data['actual'].time, 
                    [e.rotq[2] for e in self._recorded_data['actual'].points], 
                    color=COLOR_BLUE,
                    linewidth=self._plot_configs['linewidth'],
                    label=r'$\epsilon_{z}$')

            min_value = np.min([np.min([e.rotq[0] for e in self._recorded_data['actual'].points]),
                                np.min([e.rotq[0] for e in self._recorded_data['desired'].points]),
                                np.min([e.rotq[1] for e in self._recorded_data['actual'].points]),
                                np.min([e.rotq[1] for e in self._recorded_data['desired'].points]),
                                np.min([e.rotq[2] for e in self._recorded_data['actual'].points]),
                                np.min([e.rotq[2] for e in self._recorded_data['desired'].points])])

            max_value = np.max([np.max([e.rotq[0] for e in self._recorded_data['actual'].points]),
                                np.max([e.rotq[0] for e in self._recorded_data['desired'].points]),
                                np.max([e.rotq[1] for e in self._recorded_data['actual'].points]),
                                np.max([e.rotq[1] for e in self._recorded_data['desired'].points]),
                                np.max([e.rotq[2] for e in self._recorded_data['actual'].points]),
                                np.max([e.rotq[2] for e in self._recorded_data['desired'].points])])

            ax.set_xlim(np.min(self._recorded_data['desired'].time), np.max(self._recorded_data['desired'].time))
            ax.set_ylim(1.05 * min_value, 1.05 * max_value)

            self.config_2dplot(
                ax=ax,
                title='',
                xlabel=r'Time [s]',
                ylabel=r'Euler parameters',
                legend_on=True)   

            plt.tight_layout()
            plt.savefig(os.path.join(output_path, 'trajectories_quat.pdf'))
            plt.close(fig)
            del fig
        except Exception as e:
            self._logger.error('Error plotting output quaternion vectors, error=' + str(e))
            plt.close(fig)
            del fig

        try:
            ###################################################################
            # Plot velocities
            ###################################################################    

            fig = self.get_figure()        
            ax = fig.gca()
            ax.plot(self._recorded_data['desired'].time, 
                    [e.vel[0] for e in self._recorded_data['desired'].points],                     
                    color=COLOR_RED,
                    linestyle='dashed',
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\dot{X}_d$')
            ax.plot(self._recorded_data['desired'].time, 
                    [e.vel[1] for e in self._recorded_data['desired'].points], 
                    color=COLOR_GREEN,
                    linestyle='dashed',
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\dot{Y}_d$')
            ax.plot(self._recorded_data['desired'].time, 
                    [e.vel[2] for e in self._recorded_data['desired'].points], 
                    color=COLOR_BLUE,
                    linestyle='dashed',
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\dot{Z}_d$')

            ax.plot(self._recorded_data['actual'].time, 
                    [e.vel[0] for e in self._recorded_data['actual'].points], 
                    color=COLOR_RED,
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\dot{X}$')
            ax.plot(self._recorded_data['actual'].time, 
                    [e.vel[1] for e in self._recorded_data['actual'].points], 
                    color=COLOR_GREEN,
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\dot{Y}$')
            ax.plot(self._recorded_data['actual'].time, 
                    [e.vel[2] for e in self._recorded_data['actual'].points], 
                    color=COLOR_BLUE,
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\dot{Z}$')

            ax.set_xlim(
                np.min(self._recorded_data['desired'].time), 
                np.max(self._recorded_data['desired'].time))

            self.config_2dplot(
                ax=ax,
                title='',
                xlabel=r'Time [s]',
                ylabel=r'Velocity [m/s]',
                legend_on=True)   

            plt.tight_layout()
            plt.savefig(os.path.join(output_path, 'trajectories_lin_vel.pdf'))
            plt.close(fig)
            del fig

            fig = self.get_figure()        
            ax = fig.gca()
            
            ax.plot(self._recorded_data['desired'].time, 
                    [e.vel[3] for e in self._recorded_data['desired'].points],                     
                    color=COLOR_RED,
                    linestyle='dashed',
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\omega_{x_d}$')
            ax.plot(self._recorded_data['desired'].time, 
                    [e.vel[4] for e in self._recorded_data['desired'].points], 
                    color=COLOR_GREEN,
                    linestyle='dashed',
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\omega_{y_d}$')
            ax.plot(self._recorded_data['desired'].time, 
                    [e.vel[5] for e in self._recorded_data['desired'].points], 
                    color=COLOR_BLUE,
                    linestyle='dashed',
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\omega_{z_d}$')

            ax.plot(self._recorded_data['actual'].time, 
                    [e.vel[3] for e in self._recorded_data['actual'].points], 
                    color=COLOR_RED,
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\omega_x$')
            ax.plot(self._recorded_data['actual'].time, 
                    [e.vel[4] for e in self._recorded_data['actual'].points], 
                    color=COLOR_GREEN,
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\omega_y$')
            ax.plot(self._recorded_data['actual'].time, 
                    [e.vel[5] for e in self._recorded_data['actual'].points], 
                    color=COLOR_BLUE,
                    linewidth=self._plot_configs['linewidth'], 
                    label=r'$\omega_z$')
            
            ax.set_xlim(
                np.min(self._recorded_data['desired'].time), 
                np.max(self._recorded_data['desired'].time))

            self.config_2dplot(
                ax=ax,
                title='',
                xlabel=r'Time [s]',
                ylabel=r'Ang. velocity [rad/s]',
                legend_on=True)   

            plt.tight_layout()
            plt.savefig(os.path.join(output_path, 'trajectories_ang_vel.pdf'))
            plt.close(fig)
            del fig
        except Exception as e:
            self._logger.error('Error plotting velocities, error=' + str(e))
            plt.close(fig)
            del fig
