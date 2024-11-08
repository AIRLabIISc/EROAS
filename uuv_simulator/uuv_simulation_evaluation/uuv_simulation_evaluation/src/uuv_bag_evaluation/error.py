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

import numpy as np
import tf.transformations as trans
from recording import Recording


class TrajectoryError(object):
    def __init__(self, p_des, p_act):
        self.p_des = p_des
        self.p_act = p_act

        self._time = p_act.t
        self._errors = dict()

        self._errors['x'] = p_des.p[0] - p_act.p[0]
        self._errors['y'] = p_des.p[1] - p_act.p[1]
        self._errors['z'] = p_des.p[2] - p_act.p[2]

        self._errors['position'] = p_des.p - p_act.p
        self._errors['linear_velocity'] = p_des.v - p_act.v
        self._errors['angular_velocity'] = p_des.w - p_act.w

        frame = trans.quaternion_matrix(p_des.q)[0:3, 0:3]
        e_pos_inertial = p_des.pos - p_act.pos
        e_pos_des = np.dot(frame.T, e_pos_inertial)        
        self._errors['cross_track'] = e_pos_des[1]
        
        # Error quaternion wrt body frame
        err_quat = trans.quaternion_multiply(trans.quaternion_conjugate(p_des.q),
                                             p_act.q)

        # Overall angle from quaternion
        ca = err_quat[3]
        sa = np.linalg.norm(err_quat[0:3])
        self._errors['angle'] = np.arctan2(sa, ca)

        [roll_des, pitch_des, yaw_des] = trans.euler_from_quaternion(p_des.q)
        [roll_act, pitch_act, yaw_act] = trans.euler_from_quaternion(p_act.q)

        self._errors['roll'] = self.wrap(roll_des - roll_act)
        self._errors['pitch'] = self.wrap(pitch_des - pitch_act)
        self._errors['yaw'] = self.wrap(yaw_des - yaw_act)

    @staticmethod
    def wrap(x):
        return (x + np.pi) % (2.0*np.pi) - np.pi

    @property
    def t(self):
        return self._time

    @property
    def tags(self):
        return self._errors.keys()

    def get_data(self, tag):
        if tag in self._errors:
            return self._errors[tag]
        else:
            return None


class ErrorSet(object):
    __instance = None
    TAGS = ['x',
            'y',
            'z',
            'position',
            'cross_track',
            'linear_velocity',
            'angular_velocity',            
            'roll',
            'pitch',
            'yaw',
            'quaternion']

    def __init__(self):
        self._bag = None
        self._errors = list()
        self.compute_errors()
        ErrorSet.__instance = self

    @classmethod
    def get_instance(cls):
        if cls.__instance is None:
            cls.__instance = ErrorSet()
        return cls.__instance

    def compute_errors(self):
        self._bag = Recording.get_instance()
        assert self._bag is not None, 'Recording has not been created'
        # assert self._bag.is_init, 'Topics have not been sorted from the rosbag'

        if self._bag.parsers['error'].error is None:
            t_start = self._bag.parsers['trajectory'].start_time
            t_end = self._bag.parsers['trajectory'].end_time

            self._errors = list()

            for p_act in self._bag.parsers['trajectory'].odometry.points:
                if t_start <= p_act.t and p_act.t <= t_end:
                    if len(self._errors):
                        if p_act.t <= self._errors[-1].t:
                            continue
                    p_des = self._bag.parsers['trajectory'].reference.interpolate(p_act.t)
                    self._errors.append(TrajectoryError(p_des, p_act))

    @property
    def errors(self):
        return self._errors

    def get_time(self, tag='error'):
        if tag == 'error':
            if self._bag.parsers['error'].error is None:
                return np.array([e.t for e in self._errors])
            else:
                return np.array([e.t for e in self._bag.parsers['error'].error.points])
        else:
            return np.array([e.t for e in self._bag.parsers['trajectory'].odometry.points])
    def get_tags(self):
        return self.TAGS

    def get_data(self, tag, time_offset=0.0):
        if tag not in self.TAGS:
            return None

        if self._bag.parsers['error'].error is None and len(self._errors):
            assert time_offset >= 0.0 and time_offset <= self._errors[-1].t, 'Time offset is off limits'
            return [e.get_data(tag) for e in self._errors if e.t >= time_offset]
        elif self._bag.parsers['error'].error is not None:
            assert time_offset >= 0.0 and time_offset <= self._bag.parsers['error'].error.time[-1], 'Time offset is off limits'

            vec = None
            if tag == 'x':
                vec = [e.pos[0] for e in self._bag.parsers['error'].error.points if e.t >= time_offset]
            elif tag == 'y':
                vec = [e.pos[1] for e in self._bag.parsers['error'].error.points if e.t >= time_offset]
            elif tag == 'z':
                vec = [e.pos[2] for e in self._bag.parsers['error'].error.points if e.t >= time_offset]
            elif tag == 'position':
                vec = [e.pos for e in self._bag.parsers['error'].error.points if e.t >= time_offset]
            elif tag == 'linear_velocity':
                vec = [e.vel[0:3] for e in self._bag.parsers['error'].error.points if e.t >= time_offset]
            elif tag == 'angular_velocity':
                vec = [e.vel[3:6] for e in self._bag.parsers['error'].error.points if e.t >= time_offset]
            elif tag == 'roll':
                vec = [e.rot[0] for e in self._bag.parsers['error'].error.points if e.t >= time_offset]
            elif tag == 'pitch':
                vec = [e.rot[1] for e in self._bag.parsers['error'].error.points if e.t >= time_offset]
            elif tag == 'yaw':
                vec = [e.rot[2] for e in self._bag.parsers['error'].error.points if e.t >= time_offset]            
            elif tag == 'cross_track':
                vec = list()                
                for p_act in self._bag.parsers['trajectory'].odometry.points:
                    p_des = self._bag.parsers['trajectory'].reference.interpolate(p_act.t)
                    if p_des.t >= time_offset:
                        frame = trans.quaternion_matrix(p_des.q)[0:3, 0:3]
                        e_pos_inertial = p_des.pos - p_act.pos
                        e_pos_des = np.dot(frame.T, e_pos_inertial)
                        vec.append(e_pos_des[1])
            elif tag == 'quaternion':
                vec = [e.rotq[0:3] for e in self._bag.parsers['error'].error.points if e.t >= time_offset]
            return vec
            
