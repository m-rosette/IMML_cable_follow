#!/usr/bin/env python3

import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt


class PlotPapillarrayForce:
    def __init__(self) -> None:
        self.num_arrays = 2
        self.num_pillars = 9
        self.num_dim = 3

    def load_data(self, csv_file):
        return pd.read_csv(csv_file)

    def extract_columns(self, data, header_starts_with=('0_fX', '0_fY', '0_fZ')):
        # Convert all column names to strings and then filter columns
        filtered_columns = [col for col in map(str, data.columns) if col.startswith(header_starts_with)]

        # Extract the filtered columns
        extracted_data = data[filtered_columns]

        return extracted_data

    def norm_time(self, timestamp):
        init_time = timestamp[0]
        return timestamp - init_time

    def get_force_data(self, data):
        pillar_force = np.zeros((self.num_arrays, self.num_pillars, data.shape[0], self.num_dim))
        for array in range(self.num_arrays):
            for pillar in range(self.num_pillars):
                column_search = (f'{array}_fX_{pillar}', f'{array}_fY_{pillar}', f'{array}_fZ_{pillar}')
                pillar_force[array, pillar, :, :] = self.extract_columns(data, column_search)
        return pillar_force
    
    def rearrange_force(self, data):
        # 6=0, 7=1, 8=2
        force_data_copy = data.copy()
        force_data_copy[1, [6, 0], :, :] = force_data_copy[1, [0, 6], :, :]
        force_data_copy[1, [7, 1], :, :] = force_data_copy[1, [1, 7], :, :]
        force_data_copy[1, [8, 2], :, :] = force_data_copy[1, [2, 8], :, :]
        return force_data_copy
    
    def plot_force(self, force, ax=None):
        if ax is None:
            fig, axs = plt.subplots(1, self.num_arrays, figsize=(8 * self.num_arrays, 6), sharex=True)
        else:
            axs = ax  # Use provided axes

        for array in range(self.num_arrays):
            if ax is None:
                axs[array].set_ylabel('Force')
                axs[array].set_xlabel('Time (s)')
                axs[array].set_title(f'Array {array} - Force Data Over Time')

            for pillar in range(self.num_pillars):
                # Get the normalized force data for each array and pillar
                force_data = force[array, pillar, :, 0]  # Assuming the dimension is (51, 1)

                # Plot the normalized force over time with normalized timestamps
                if ax is None:
                    axs[array].plot(force_data, label=f'Pillar {pillar}')
                    axs[array].legend(loc='upper left')
                else:
                    ax[array].plot(force_data, label=f'Pillar {pillar}')

        if ax is None:
            plt.tight_layout()
            plt.show()

    def plot_xyz_force(self, avg_force):
        # Assuming 'data' contains x, y, z values for each subplot
        num_rows = 3
        num_cols = 3
        fig, axs = plt.subplots(num_rows, num_cols, figsize=(14, 12))

        # Flatten the axis array for easy iteration
        axs = axs.flatten()

        # Define the order of subplots
        subplot_order = [6, 3, 0, 7, 4, 1, 8, 5, 2]

        for array in range(self.num_arrays):
            if array == 0 :
                x_color = [135/255, 206/255, 235/255] # Light blue
                y_color = [60/255, 179/255, 113/255] # Light green
                z_color = [255/255, 140/255, 10/255] # Light orange
            if array == 1:
                x_color = [30/255, 144/255, 255/255] # Dark blue
                y_color = [50/255, 205/255, 50/255] # Dark green
                z_color = [255/255, 127/255, 120/255] # Dark orange

            for pillar, subplot_idx in enumerate(subplot_order):
                x = avg_force[array, subplot_idx, :, 0]
                y = avg_force[array, subplot_idx, :, 1]
                z = avg_force[array, subplot_idx, :, 2]

                axs[pillar].plot(x, label=f'X_{array}', color=x_color)
                axs[pillar].plot(y, label=f'Y_{array}', color=y_color)
                axs[pillar].plot(z, label=f'Z_{array}', color=z_color)

                axs[pillar].set_title(f'Pillar {subplot_idx}')
                axs[pillar].legend()
                axs[pillar].set_xlabel('Time')
                axs[pillar].set_ylabel('Force')

                # axs[pillar].set_ylim([-3.5, 3.5])

        plt.tight_layout()
        plt.show()



if __name__ == '__main__':
    pltforce = PlotPapillarrayForce()
    
    filename = '/home/marcus/IMML/ros2_ws/src/IMML_cable_follow/trial_control/trial_control/resource/test_slip2.csv'
    data = pltforce.load_data(filename)
    timestamp = np.array(pltforce.extract_columns(data, ('timestamp')))
    pillar_force = pltforce.get_force_data(data)
    pillar_force = pltforce.rearrange_force(pillar_force)
    pltforce.plot_xyz_force(pillar_force)