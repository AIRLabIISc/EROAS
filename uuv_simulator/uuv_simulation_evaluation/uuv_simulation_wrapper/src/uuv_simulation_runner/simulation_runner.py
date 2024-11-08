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

import logging
import os
import sys
import yaml
import time
import random
import shutil
import psutil
import datetime
import signal
import socket
import signal
from threading import Timer
from time import gmtime, strftime, sleep

ROS_DEFAULT_HOST = 'localhost'
ROS_DEFAULT_PORT = 11311
GAZEBO_DEFAULT_HOST = 'localhost'
GAZEBO_DEFAULT_PORT = 11345
ROS_HOME = './'
ROS_LOG_DIR = 'log'
PORT_LOCK_FILE = 'uuv_port_lock'


class SimulationRunner(object):
    """
    This class can run a simulation scenario by calling roslaunch with
    configurable parameters and create a folder to store the simulation's ROS
    bag and configuration files.
    """

    def __init__(self, params, task_filename, results_folder='./results',
                 record_all_results=False, add_folder_timestamp=True,
                 log_filename=None, log_dir='logs'):
        # Setting up the logging
        self._task_name = task_filename.split('/')[-1]
        self._task_name = self._task_name.split('.')[0]

        self.record_all_results = record_all_results

        self._log_dir = log_dir
        if not os.path.isdir(log_dir):
            os.makedirs(log_dir)

        self._logger = logging.getLogger(
            'run_simulation_wrapper_%s' % self._task_name)
        if len(self._logger.handlers) == 0:
            self._out_hdlr = logging.StreamHandler(sys.stdout)
            self._out_hdlr.setFormatter(logging.Formatter(
                '%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
            self._out_hdlr.setLevel(logging.INFO)
            self._logger.addHandler(self._out_hdlr)
            self._logger.setLevel(logging.INFO)

            if log_filename is None:
                if not os.path.isdir('logs'):
                    os.makedirs('logs')
                log_filename = os.path.join(
                    'logs', 'simulation_task_%s.log' % self._task_name)

            self._file_hdlr = logging.FileHandler(log_filename)
            self._file_hdlr.setFormatter(
                logging.Formatter(
                    '%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
            self._file_hdlr.setLevel(logging.INFO)

            self._logger.addHandler(self._file_hdlr)
            self._logger.setLevel(logging.INFO)

        parent_pid = os.getppid()
        if not psutil.pid_exists(parent_pid):
            self._logger.error('Parent process has died, exiting simulation '
                               'runner...')
            sys.exit(0)
        else:
            self._logger.info('Parent process is still alive')

        if os.path.isfile('UUV_TERMINATE'):
            self._logger.error('UUV_TERMINATE file found, exiting simulation '
                               'runner...')
            sys.exit(0)

        self._logger.info('Record all results=' + str(record_all_results))

        assert type(params) is dict, 'Parameter structure must be a dict'
        self._params = params

        self._sim_counter = 0

        random.seed()

        assert os.path.isfile(task_filename), 'Invalid task file'
        self._task_filename = task_filename

        with open(self._task_filename, 'r') as task_file:
            self._task_text = task_file.read()

        self._logger.info('Task file <%s>' % self._task_filename)

        # Create results folder, if not existent
        self._results_folder = results_folder

        if not os.path.isdir(self._results_folder):
            os.makedirs(self._results_folder)

        self._results_folder = os.path.abspath(self._results_folder)

        self._logger.info('Results folder <%s>' % self._results_folder)

        # Filename for the ROS bag
        self._recording_filename = None

        self._process_timeout_triggered = False
        self.processes_interrupted = False

        self._add_folder_timestamp = add_folder_timestamp
        # Output directory
        self._sim_results_dir = None

        self._ros_port = self._get_random_open_port(15000, 20000)
        self._gazebo_port = self._get_random_open_port(25000, 30000)

        # Default timeout for the process
        self._timeout = 1e5
        self._simulation_timeout = None
        # POpen object to be instantiated
        self._process = None
        self._process_children = list()

        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)

    def __del__(self):
        self._logger.warning('Destroying simulation runner')
        self._kill_process()
        if self._recording_filename is None:
            self._logger.warning('Recording filename was not initialized')
        else:
            self._logger.warning('Recording filename=%s',
                                 str(self._recording_filename))
        if not self.record_all_results:
            self.remove_recording_dir()

    @property
    def recording_filename(self):
        return self._recording_filename

    @property
    def current_sim_results_dir(self):
        return self._sim_results_dir

    @property
    def process_timeout_triggered(self):
        return self._process_timeout_triggered

    @property
    def timeout(self):
        return self._simulation_timeout

    def signal_handler(self, signal, handler):
        self._logger.warning('SIGNAL RECEIVED=%d', int(signal))
        self.processes_interrupted = True
        self.__del__()

    def _port_open(self, port):
        return_code = 1
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            return_code = sock.connect_ex(('', port))
            sock.close()
        except Exception as exp:
            print(exp)
        return return_code == 0

    def _is_port_locked(self, port):
        return os.path.exists(self._get_port_lock_file(port))

    def _lock_port(self, port):
        with open(self._get_port_lock_file(port), 'a') as lock_file:
            lock_file.close()
        return port

    def _unlock_port(self, port):
        if os.path.exists(self._get_port_lock_file(port)):
            os.remove(self._get_port_lock_file(port))
        self._logger.info('Unlocking port %d' % port)

    def _get_port_lock_file(self, port):
        return os.path.join('/tmp', '%s-%d.lock' % (PORT_LOCK_FILE, port))

    def _get_random_open_port(self, start=1000, end=3000, timeout=10):
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            port = random.randrange(start, end, 1)
            self._logger.info('Testing port %d' % port)
            if not self._port_open(port) and not self._is_port_locked(port):
                self._logger.info('Locking port %d' % port)
                return self._lock_port(port)
            self._logger.info('Port %d is locked' % port)
        raise RuntimeError("Could not find any open port from %d to %d for %ds." %(start, end, timeout))

    def _set_env_variables(self):
        os.environ['ROS_MASTER_URI'] = 'http://localhost:%d' % self._ros_port
        os.environ['GAZEBO_MASTER_URI'] = 'http://localhost:%d' % self._gazebo_port
        ros_home = os.path.join(self._sim_results_dir, 'ros')
        if not os.path.isdir(ros_home):
            os.makedirs(ros_home)
        os.environ['ROS_HOME'] = ros_home

    def _kill_process(self):
        if self._process is None:
            self._logger.warning('Process object is invalid')
            return
        if len(self._process_children) == 0:
            self._logger.warning('No children processes found')
            return
        try:
            self._logger.warning('PROCESS TIMEOUT - killing process tree...')

            for p in self._process_children:
                if psutil.pid_exists(p.pid):
                    self._logger.warning('Sending SIGTERM to child '
                                         'process id=%d', p.pid)
                    p.send_signal(signal.SIGTERM)
                    if psutil.pid_exists(p.pid):
                        self._logger.warning('Child process %d '
                                             'successfully terminated',
                                             p.pid)
                    else:
                        self._logger.error('Child process %d still '
                                           'running', p.pid)
                else:
                    self._logger.warning('Child process %d is not alive',
                                         p.pid)

            gone, alive = psutil.wait_procs(
                self._process_children,
                timeout=None,
                callback=self._on_terminate)

            self._logger.warning(
                'Kill processes=\n\t - Gone={}\n\t - '
                'Alive{}'.format(str(gone), str(alive)))

            self._process_timeout_triggered = True
            self._logger.warning('PROCESS TIMEOUT - finishing process...')
        except Exception as ex:
            self._logger.error('Error occurred while killing processes, '
                               'message=%s' % str(ex))

    def _on_terminate(self, process):
        try:
            if psutil.pid_exists(process.pid):
                self._logger.warning('Process {} <{}> terminated with exit'
                                     ' code {}'.format(process.pid,
                                                       process.name(),
                                                       process.returncode))
            else:
                self._logger.warning('Process {} already '
                                     'terminated'.format(process.pid))
        except Exception as e:
            self._logger.error('Error in on_terminate function, message=' + str(e))

    def _create_script_file(self, output_dir, cmd):
        try:
            filename = os.path.join(output_dir, 'run_simulation.sh')
            self._logger.info('Creating script file=' + filename)
            with open(filename, 'w+') as script_file:
                script_file.write('#!/usr/bin/env bash\n')
                script_file.write(cmd)
            self._logger.info('Script file created=' + filename)
        except Exception as e:
            self._logger.error('Error while creating script file, message=' + str(e))

    def remove_recording_dir(self):
        if self._recording_filename is not None and not self.record_all_results:
            rec_path = os.path.dirname(self._recording_filename)
            if os.path.isdir(rec_path):
                self._logger.info('Removing recording directory, path=' + rec_path)
                shutil.rmtree(rec_path)
            else:
                self._logger.info('Recording directory has already been deleted, path=' + rec_path)

    def run(self, params=dict(), timeout=None):
        if len(params.keys()) > 0:
            for tag in self._params:
                if tag not in params:
                    raise Exception('Parameter list has the wrong dimension')
                else:
                    if type(params[tag]) == list:
                        self._params[tag] = [float(x) for x in params[tag]]
                    else:
                        self._params[tag] = params[tag]

        self.remove_recording_dir()

        if self._add_folder_timestamp:
            self._sim_results_dir = os.path.join(
                self._results_folder,
                self._task_name + '_' + \
                strftime("%Y-%m-%d %H-%M-%S", gmtime()) + '_' + \
                str(random.randrange(0, 1000, 1))).replace(' ', '_')
        else:
            self._sim_results_dir = self._results_folder

        if not os.path.isdir(self._sim_results_dir):
            os.makedirs(self._sim_results_dir)

        self._set_env_variables()

        task_filename = os.path.join(self._sim_results_dir,
                                     'task.yml')

        if len(self._params.keys()):
            with open(os.path.join(self._sim_results_dir,
                                   'params_%d.yml' % self._sim_counter), 'w') as param_file:
                yaml.safe_dump(self._params, param_file, default_flow_style=False, encoding='utf-8', allow_unicode=True)

        self._logger.info('Running the simulation through system call')

        try:
            with open(task_filename, 'w') as task_file:
                task_file.write(self._task_text)

            with open(task_filename, 'r') as task_file:
                task = yaml.load(task_file)
                self._logger.info('Running task: ' + task['id'])
                # Setting the filename to the resulting rosbag
                self._recording_filename = os.path.join(self._sim_results_dir, 'recording.bag')
                self._logger.info('ROS bag: ' + self._recording_filename)
                cmd = task['execute']['cmd'] + ' '
                for param in task['execute']['params']:
                    if param in self._params:
                        continue
                    # Adding parameters to the command line string
                    cmd += param + ':='
                    if type(task['execute']['params'][param]) == bool:
                        cmd += str(int(task['execute']['params'][param])) + ' '
                    else:
                        cmd += str(task['execute']['params'][param]) + ' '
                    if 'timeout' in param:
                        # Setting the process timeout
                        if task['execute']['params'][param] > 0 and timeout is None:
                            # Set the process timeout to 5 times the given simulation timeout
                            self._simulation_timeout = task['execute']['params'][param]
                            self._timeout = 5 * int(self._simulation_timeout)
                            self._logger.info('Simulation timeout t=%.f s' % self._simulation_timeout)
                        else:
                            self._logger.error('Invalid timeout = %.f' % task['execute']['params'][param])

                # If timeout was given as an input argument, take it as process timeout
                if timeout is not None:
                    if timeout > 0:
                        self._timeout = timeout
                self._logger.info('Process timeout t=%.f s' % self._timeout)

                cmd = cmd + 'bag_filename:=\"' + self._recording_filename + '\" '

                for param in self._params:
                    if 'timeout' in param:
                        # Setting the process timeout
                        if task['execute']['params'][param] > 0 and timeout is None:
                            # Set the process timeout to 5 times the given simulation timeout
                            self._simulation_timeout = task['execute']['params'][param]
                            self._timeout = 5 * int(self._simulation_timeout)
                            self._logger.info('Simulation timeout t=%.f s' % self._simulation_timeout)
                        else:
                            self._logger.error('Invalid timeout = %.f' % task['execute']['params'][param])
                            
                    param_values = str(self._params[param])
                    param_values = param_values.replace('[', '')
                    param_values = param_values.replace(']', '')
                    param_values = param_values.replace(' ', '')

                    cmd = cmd + param + ':=' + param_values + ' '
                self._logger.info('Run system call: ' + cmd)

                # Create log file
                timestamp = datetime.datetime.now().isoformat()
                log_dir = os.path.join(self._log_dir, self._task_name)
                if not os.path.isdir(log_dir):
                    os.makedirs(log_dir)
                logfile_name = os.path.join(log_dir, "%s_process_log_%s.log" % (timestamp, self._task_name))
                logfile = open(logfile_name, 'a')

                # Create script with the command being run for eventual manual rerun
                self._create_script_file(self._sim_results_dir, cmd)
                # Start process
                self._process = psutil.Popen(cmd, shell=True, stdout=logfile, stderr=logfile, env=os.environ.copy())

                proc = psutil.Process(self._process.pid)
                self._logger.info('Process created (Name=%s, PID=%d)' % (proc.name(), proc.pid))

                # Loading all the process's children
                sleep(1)
                self._process_children = proc.children(recursive=True)
                self._process_children.append(proc)

                # Start process timeout, which is a security measure in case something happens, e.g. roscore not responding
                # If the process timeout is reached before the simulation process is finished, this function
                # will return false
                timer = Timer(self._timeout, self._kill_process)
                timer.start()
                success = self._process.wait(timeout=self._timeout)

                if success == 0:
                    self._logger.info('Simulation finished successfully')
                    result_ok = True
                else:
                    self._logger.info('Simulation finished with error')
                    result_ok = False
        except Exception as e:
            self._logger.error('Error while running the simulation, message=' + str(e))
            result_ok = False
            self._kill_process()

        self._unlock_port(self._ros_port)
        self._unlock_port(self._gazebo_port)

        self._logger.info('Simulation finished <%s>' % os.path.join(self._sim_results_dir, 'recording.bag'))
        time.sleep(0.05)

        self._sim_counter += 1
        self._process = None
        return result_ok
