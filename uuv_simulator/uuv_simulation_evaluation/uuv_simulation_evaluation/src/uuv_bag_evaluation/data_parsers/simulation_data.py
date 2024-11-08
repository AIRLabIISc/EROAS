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
import logging
import sys
import matplotlib.pyplot as plt

try:
    import seaborn
    plt.style.use('seaborn-ticks')
    plt.rcParams['legend.frameon'] = True
    COLOR_RED = seaborn.xkcd_rgb['pale red']
    COLOR_GREEN = seaborn.xkcd_rgb['medium green']
    COLOR_BLUE = seaborn.xkcd_rgb['denim blue']
except:
    COLOR_RED = '#d62728'
    COLOR_GREEN = '#2ca02c'
    COLOR_BLUE = '#1f77b4'


class SimulationData(object):
    LABEL = ""

    def __init__(self, topic_name=None, message_type=None, prefix=None):
        # Setting up the log
        self._logger = logging.getLogger(self.LABEL)
        if len(self._logger.handlers) == 0:
            out_hdlr = logging.StreamHandler(sys.stdout)
            out_hdlr.setFormatter(logging.Formatter('%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
            out_hdlr.setLevel(logging.INFO)
            self._logger.addHandler(out_hdlr)
            self._logger.setLevel(logging.INFO)

        self._topic_name = topic_name
        self._message_type = message_type
        self._time = None
        self._prefix = prefix
        self._recorded_data = dict()
        self._output_dir = '/tmp'

        self._plot_configs = dict(                                    
            figsize=[12, 6],
            linewidth=3,
            label_fontsize=22,
            title_fontsize=20,
            tick_labelsize=20,
            xlim=None,
            ylim=None,
            zlim=None,                                    
            labelpad=10,
            legend=dict(
                loc='upper right',
                fontsize=22))

    @staticmethod
    def get_all_parsers():
        return SimulationData.__subclasses__()

    @staticmethod
    def get_all_labels():
        return [parser.LABEL for parser in SimulationData.get_all_parsers()]
        
    def read_data(self, bag):
        raise NotImplementedError()

    def get_data(self, *args):
        raise NotImplementedError()

    def plot(self, output_dir):
        raise NotImplementedError()
    
    def get_data(self):
        return self._time, self._recorded_data    

    def get_figure(self, n_rows=1):
        return plt.figure(
            figsize=(self._plot_configs['figsize'][0], n_rows * self._plot_configs['figsize'][1]))

    def config_2dplot(self, ax, title, xlabel, ylabel, legend_on=True):
        if len(title):
            ax.set_title(
                title, 
                fontsize=self._plot_configs['title_fontsize'])
        ax.grid(axis='both', alpha=0.3, linewidth=0.8)
        ax.tick_params(
            axis='both',
            labelsize=self._plot_configs['tick_labelsize'])
        ax.set_xlabel(
            xlabel,
            fontsize=self._plot_configs['label_fontsize'])
        ax.set_ylabel(
            ylabel,
            fontsize=self._plot_configs['label_fontsize'])
        if legend_on:
            leg = ax.legend(
                fancybox=True, 
                framealpha=1, 
                loc=self._plot_configs['legend']['loc'], 
                fontsize=self._plot_configs['legend']['fontsize'])
            leg.get_frame().set_facecolor('white')

    def get_as_dataframe(self, add_group_name=None):
        raise NotImplementedError()
