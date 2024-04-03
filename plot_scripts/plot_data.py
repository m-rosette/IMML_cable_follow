#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

from process_tactile_data import PlotPapillarrayForce
from process_mark10_data import PlotMark10


if __name__ == "__main__":
    tactile = PlotPapillarrayForce()
    mark10 = PlotMark10()
    
    trial_num = 0

    # Plotting the subplots from PlotPapillarrayForce in the first row
    fig, axs = plt.subplots(2, 2, figsize=(18, 10))  # Two rows of subplots

    filename = 'data_01_05_24/0.csv'
    data = tactile.load_data(filename)
    timestamp = np.array(tactile.extract_columns(data, ('timestamp')))
    timestamp = tactile.norm_time(timestamp)

    data_dir = 'data_01_05_24'  # Change this to your data directory path
    
    norm_pillar_force, std_pillar_force = tactile.norm_trial_stats(trial_num, data_dir)

    tactile.plot_norm_force(timestamp, norm_pillar_force, std_pillar_force, ax=axs[0])

    # Set legends and titles for the first row of subplots
    axs[0, 0].set_ylabel('Normalized Force')
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_title('Array 0 - Normalized Force Data Over Time')
    axs[0, 0].legend(loc='upper left')

    axs[0, 1].set_ylabel('Normalized Force')
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_title('Array 1 - Normalized Force Data Over Time')
    axs[0, 1].legend(loc='upper left')

    # Plotting the subplots from PlotMark10 in the second row
    mark10_data_dir = 'mark10_data_01_05_24'
    timestamp, avg_force, std_force, avg_travel, std_travel = mark10.trial_stats(trial_num, mark10_data_dir)
    mark10.plot_avg_force(timestamp, avg_force, avg_travel, std_force, std_travel, ax=axs[1])

    # Set legends and titles for the second row of subplots
    axs[1, 0].set_ylabel('Force (N)')
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_title('Average Mark10 Force')
    axs[1, 0].legend(loc='upper left')

    axs[1, 1].set_ylabel('Travel (in)')
    axs[1, 1].set_xlabel('Time (s)')
    axs[1, 1].set_title('Average Mark10 Travel')
    axs[1, 1].legend(loc='upper left')

    plt.tight_layout()
    plt.show()