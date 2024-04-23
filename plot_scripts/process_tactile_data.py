#!/usr/bin/env python3

import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt


class PlotPapillarrayForce:
    def __init__(self, filename):
        self.num_arrays = 2
        self.num_pillars = 9
        self.num_dim = 3
        self.num_global = 6
        self.add_plot_region_bound = 5

        self.data = self.load_data(filename)
        self.timestamp = np.array(self.extract_columns(header_starts_with=('timestamp')))
        self.norm_time() # Convert from computer to trial time

        self.pillar_force = np.zeros((self.num_arrays, self.num_pillars, self.data.shape[0], self.num_dim))
        self.global_force = np.zeros((self.num_arrays, self.data.shape[0], self.num_global))
        self.grip_force = np.zeros((self.data.shape[0], 1))
        self.stepper_pos = np.zeros((self.data.shape[0], 1))

    def load_data(self, csv_file):
        return pd.read_csv(csv_file)

    def extract_columns(self, header_starts_with=('0_fX', '0_fY', '0_fZ')):
        # Convert all column names to strings and then filter columns
        filtered_columns = [col for col in map(str, self.data.columns) if col.startswith(header_starts_with)]

        # Extract the filtered columns
        extracted_data = self.data[filtered_columns]

        return extracted_data

    def norm_time(self):
        init_time = self.timestamp[0]
        self.timestamp = self.timestamp - init_time

    def get_force_data(self):
        for array in range(self.num_arrays):
            for pillar in range(self.num_pillars):
                column_search = (f'{array}_fX_{pillar}', f'{array}_fY_{pillar}', f'{array}_fZ_{pillar}')
                self.pillar_force[array, pillar, :, :] = self.extract_columns(column_search)
    
    def get_global_force_data(self):
        for array in range(self.num_arrays):
            column_search = (f'{array}_gfx', f'{array}_gfy', f'{array}_gfz', f'{array}_gtx', f'{array}_gty', f'{array}_gtz')
            self.global_force[array, :, :] = self.extract_columns(column_search)
    
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

    def plot_xyz_force(self):
        # self.timestamp = np.squeeze(self.timestamp)
        force = self.get_force_data()
        force = self.rearrange_force(force)

        # Assuming 'force' contains x, y, z values for each subplot
        num_rows = 3
        num_cols = 3
        fig, axs = plt.subplots(num_rows, num_cols, figsize=(14, 12))

        # Flatten the axis array for easy iteration
        axs = axs.flatten()

        # Define the order of subplots (according to the tactile array layout)
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
                x = force[array, subplot_idx, :, 0]
                y = force[array, subplot_idx, :, 1]
                z = force[array, subplot_idx, :, 2]

                axs[pillar].plot(self.timestamp, x, label=f'X_{array}', color=x_color)
                axs[pillar].plot(self.timestamp, y, label=f'Y_{array}', color=y_color)
                axs[pillar].plot(self.timestamp, z, label=f'Z_{array}', color=z_color)

                axs[pillar].set_title(f'Pillar {subplot_idx}')
                axs[pillar].legend()
                axs[pillar].set_xlabel('Time')
                axs[pillar].set_ylabel('Force')

                # axs[pillar].set_ylim([-3.5, 3.5])

        plt.tight_layout()
        plt.show()

    def plot_cable_pull(self, grip_or_step='grip'):
        # self.timestamp = np.squeeze(self.timestamp)
        self.get_global_force_data()

        # Get gripper variables
        cable_status = self.extract_columns(header_starts_with=('cable_status'))
        self.grip_force = self.extract_columns(header_starts_with=('gripper_current'))
        self.stepper_pos = self.extract_columns(header_starts_with=('stepper_pos'))

        # res = [i for i in range(len(stepper_pos)) if stepper_pos[i] is None]
 
        # # print result
        # print("The None indices list is : " + str(res))

        # print(stepper_pos)

        # Convert NaN values to zero
        # cable_status = np.nan_to_num(cable_status, nan=1)
        self.grip_force = np.nan_to_num(self.grip_force)
        self.stepper_pos = np.nan_to_num(self.stepper_pos)

        fig, ax1 = plt.subplots()

        if grip_or_step == 'grip':
            # Plot grip_force on the right y-axis
            color = 'tab:red'
            ax1.set_xlabel('Time')
            ax1.set_ylabel('Grip Force', color=color)
            ax1.plot(self.timestamp, self.grip_force, color=color, label='Grip Force', linestyle=':', linewidth=3)
            ax1.tick_params(axis='y', labelcolor=color)
            cable_shade_scale = [np.min(self.grip_force), np.max(self.grip_force)]
        if grip_or_step == 'step':
            # Plot grip_force on the right y-axis
            color = 'tab:red'
            ax1.set_xlabel('Time')
            ax1.set_ylabel('Stepper Position', color=color)
            ax1.plot(self.timestamp, self.stepper_pos, color=color, label='Stepper Position', linestyle=':', linewidth=3)
            ax1.tick_params(axis='y', labelcolor=color) 
            cable_shade_scale = [np.min(self.stepper_pos), np.max(self.stepper_pos)]

        # Create a second y-axis for global force
        ax2 = ax1.twinx()

        # Plot global_force for array 0
        # ax2.plot(self.timestamp, self.global_force[0, :, 0], label='Array 0 - Global X', linestyle='--')
        ax2.plot(self.timestamp, self.global_force[0, :, 1], label='Array 0 - Global Y', linestyle='-.')
        ax2.plot(self.timestamp, self.global_force[0, :, 2], label='Array 0 - Global Z', linestyle='-')

        # Plot global_force for array 1
        # ax2.plot(self.timestamp, self.global_force[1, :, 0], label='Array 1 - Global X', linestyle='--')
        ax2.plot(self.timestamp, self.global_force[1, :, 1], label='Array 1 - Global Y', linestyle='-.')
        ax2.plot(self.timestamp, self.global_force[1, :, 2], label='Array 1 - Global Z', linestyle='-')

        ax2.set_ylabel('Global Force', color='tab:blue')
        ax2.tick_params(axis='y', labelcolor='tab:blue')

        # Fill between cable status 0 and 1
        ax1.fill_between(np.squeeze(self.timestamp), cable_shade_scale[0]-self.add_plot_region_bound, cable_shade_scale[1]+self.add_plot_region_bound, where=(np.squeeze(cable_status) == 0), color='darkgrey', alpha=0.5) #, label='Cable Status 0')
        ax1.fill_between(np.squeeze(self.timestamp), cable_shade_scale[0]-self.add_plot_region_bound, cable_shade_scale[1]+self.add_plot_region_bound, where=(np.squeeze(cable_status) == 1), color='lightgrey', alpha=0.5) #, label='Cable Status 1')

        try:
            # Find the indices where cable status is 0
            cable_status_0_indices = np.where(np.squeeze(cable_status) == 0)[0]
            xmin_status_0 = self.timestamp[cable_status_0_indices[0]]
            xmax_status_0 = self.timestamp[cable_status_0_indices[-1]]

            # Position the text box within the shaded regions
            ax1.text((xmin_status_0 + xmax_status_0) / 2, cable_shade_scale[0]-self.add_plot_region_bound, "Cable Unseated", horizontalalignment='center', verticalalignment='bottom')
            ax1.text((self.timestamp[0] + xmin_status_0) / 2, cable_shade_scale[0]-self.add_plot_region_bound, 'Cable Seated', horizontalalignment='center', verticalalignment='bottom')
            ax1.text((self.timestamp[-1] + xmax_status_0) / 2, cable_shade_scale[0]-self.add_plot_region_bound, 'Cable Seated', horizontalalignment='center', verticalalignment='bottom')
        
        except IndexError:
            pass

        # Combine legend for both axes
        lines, labels = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax2.legend(lines + lines2, labels + labels2, loc='upper right')

        plt.show()

    def plot_slip_slope(self):
        self.get_global_force_data()
        self.stepper_pos = np.squeeze(self.extract_columns(header_starts_with=('stepper_pos')))

        stepper_diff = np.diff(self.stepper_pos)[:250]
        global_y_0_diff = np.diff(self.global_force[0, :, 1])[:250]
        # print(stepper_diff.shape)

        # Reshape the array into a 2D array with 5 columns
        stepper_diff_reshape = stepper_diff.reshape(-1, 5)
        global_y_0_diff_reshape = global_y_0_diff.reshape(-1, 5)

        stepper_diff_avg = np.mean(stepper_diff_reshape, axis=1)
        global_y_0_diff_avg = np.mean(global_y_0_diff_reshape, axis=1)

        slopes = []
        for i, step_diff in enumerate(stepper_diff_avg):
            # If no slope differences resulted in 0
            if step_diff != 0:
                # Calculate slope (change in value / change in time)
                slopes.append(global_y_0_diff_avg[i] / step_diff)

        print(slopes)

        plt.plot(slopes)
        plt.show()

    def plot_slope(self):
        self.get_global_force_data()
        self.stepper_pos = np.squeeze(self.extract_columns(header_starts_with=('stepper_pos')))
        self.stepper_pos = np.nan_to_num(self.stepper_pos)

        # print(self.global_force[0, :, 1])
        # print(self.stepper_pos)

        plt.plot(self.stepper_pos, self.global_force[0, :, 1])
        plt.show()


if __name__ == '__main__':
    # name = 'test_failed_unseated_middle'
    # name = 'test_failed_unseated_top'
    # name = 'global_y_3pt5_exceeded_bottom'
    # name = 'global_z_6pt0_exceeded_bottom'
    # name = 'new_stepper'
    name = '4'

    filename = f'/home/marcus/IMML/ros2_ws/src/IMML_cable_follow/trial_control/trial_control/resource/{name}.csv'    
    pltforce = PlotPapillarrayForce(filename)

    # pltforce.plot_xyz_force()
    pltforce.plot_cable_pull('grip')
    # pltforce.plot_slip_slope()
    # pltforce.plot_slope()