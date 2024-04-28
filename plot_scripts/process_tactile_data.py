#!/usr/bin/env python3

import pandas as pd
import numpy as np
import os
import glob
import matplotlib.pyplot as plt
import textwrap


class PlotPapillarrayForce:
    def __init__(self, filename):
        self.num_arrays = 2
        self.num_pillars = 9
        self.num_dim = 3
        self.num_global = 6
        self.add_plot_region_bound = 5
        self.num_pull_trials = 10
        self.cutoff_index = 220
        self.trial_data_shape = np.zeros((self.num_arrays, self.cutoff_index, self.num_global, self.num_pull_trials))

        self.data = self.load_data(filename)
        self.timestamp = np.array(self.extract_columns(self.data, header_starts_with=('timestamp')))
        self.norm_time() # Convert from computer to trial time

        self.pillar_force = np.zeros((self.num_arrays, self.num_pillars, self.data.shape[0], self.num_dim))
        # self.global_force = np.zeros((self.num_arrays, self.data.shape[0], self.num_global))
        self.global_force = np.zeros((self.num_arrays, self.cutoff_index, self.num_global))
        self.grip_force = np.zeros((self.data.shape[0], 1))
        self.stepper_pos = np.zeros((self.data.shape[0], 1))

    def load_data(self, csv_file):
        return pd.read_csv(csv_file)

    def extract_columns(self, data, header_starts_with=('0_fX', '0_fY', '0_fZ')):
        # Convert all column names to strings and then filter columns
        filtered_columns = [col for col in map(str, data.columns) if col.startswith(header_starts_with)]

        # Extract the filtered columns
        extracted_data = data[filtered_columns]

        return extracted_data

    def norm_time(self):
        init_time = self.timestamp[0]
        self.timestamp = self.timestamp - init_time

    def get_force_data(self, data):
        for array in range(self.num_arrays):
            for pillar in range(self.num_pillars):
                column_search = (f'{array}_fX_{pillar}', f'{array}_fY_{pillar}', f'{array}_fZ_{pillar}')
                self.pillar_force[array, pillar, :, :] = self.extract_columns(data, column_search)
    
    def get_global_force_data(self, data):
        for array in range(self.num_arrays):
            column_search = (f'{array}_gfx', f'{array}_gfy', f'{array}_gfz', f'{array}_gtx', f'{array}_gty', f'{array}_gtz')
            self.global_force[array, :, :] = self.extract_columns(data, column_search)
    
    def rearrange_force(self, data):
        # 6=0, 7=1, 8=2
        force_data_copy = data.copy()
        force_data_copy[1, [6, 0], :, :] = force_data_copy[1, [0, 6], :, :]
        force_data_copy[1, [7, 1], :, :] = force_data_copy[1, [1, 7], :, :]
        force_data_copy[1, [8, 2], :, :] = force_data_copy[1, [2, 8], :, :]
        return force_data_copy
    
    def get_trial_data(self, trial_dir, filename_prefix):
        filepath = [os.path.join(trial_dir, filename) for filename in os.listdir(trial_dir) if filename.startswith(filename_prefix)]

        data = self.trial_data_shape.copy()

        for i, file in enumerate(filepath):
            trial_data = self.load_data(file)
            self.get_global_force_data(trial_data.iloc[:self.cutoff_index, :])
            data[:, :, :, i] = self.global_force

        return data

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
        self.get_force_data()
        force = self.rearrange_force(self.pillar_force)

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

    def plot_single_cable_pull(self, grip_or_step='grip'):
        # self.timestamp = np.squeeze(self.timestamp)
        self.get_global_force_data(self.data)

        # Get gripper variables
        cable_status = self.extract_columns(self.data, header_starts_with=('cable_status'))
        self.grip_force = self.extract_columns(self.data, header_starts_with=('gripper_current'))
        self.stepper_pos = self.extract_columns(self.data, header_starts_with=('stepper_pos'))

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
        
    def plot_pass_fail_trials(self):
        timestamp = np.squeeze(self.timestamp)[:self.cutoff_index]

        trial_directory = '/home/marcus/IMML/ros2_ws/src/IMML_cable_follow/trial_control/trial_control/resource/'

        # Get all passed trials
        large_pass = self.get_trial_data(trial_directory, 'large_pass')
        medium_pass = self.get_trial_data(trial_directory, 'med_pass')
        
        # Get all failed trials
        medium_fail = self.get_trial_data(trial_directory, 'med_fail')
        small_fail = self.get_trial_data(trial_directory, 'small_fail')

        # Create empty plots with labels for each color
        plt.plot([], [], color='orange', label='Large Pass')
        plt.plot([], [], color='red', label='Small Fail')
        plt.plot([], [], color='green', label='Medium Pass')
        plt.plot([], [], color='blue', label='Medium Fail')

        # Start with all 10 trials (all passes or all fails)
        for trial in range(self.num_pull_trials):
            # Plot from the first array
            plt.plot(timestamp, large_pass[0, :, 1, trial], color='orange')
            plt.plot(timestamp, small_fail[0, :, 1, trial], color='red')

            # Plot from the second array
            plt.plot(timestamp, large_pass[1, :, 1, trial], color='orange')
            plt.plot(timestamp, small_fail[1, :, 1, trial], color='red')

        # # There are 5/5 pass/fail of the medium magnet
        # for trial in range(5):
        #     # Plot from the first array
        #     plt.plot(timestamp, medium_pass[0, :, 1, trial], color='green')
        #     plt.plot(timestamp, medium_fail[0, :, 1, trial], color='blue')

        #     # Plot from the second array
        #     plt.plot(timestamp, medium_pass[1, :, 1, trial], color='green')
        #     plt.plot(timestamp, medium_fail[1, :, 1, trial], color='blue')
        
        # Plot the max y global force
        plt.plot(timestamp, [4.0] * self.cutoff_index, linestyle='--', color='black', label='Force Threshold')
        plt.plot(timestamp, [-4.0] * self.cutoff_index, linestyle='--', color='black')
        
        plt.legend()
        plt.show()
    
    def imece_plot(self):
        trial_directory = '/home/marcus/IMML/ros2_ws/src/IMML_cable_follow/trial_control/trial_control/resource/'

        # Get and format timestamp
        timestamp = np.squeeze(self.timestamp)[:self.cutoff_index]
        start_time_idx = 59
        end_time_idx_small = 150
        small_time = timestamp[start_time_idx:end_time_idx_small]
        # small_time = range(len(small_time))

        # Get data for second subplot (small - failed)
        small_fail = self.get_trial_data(trial_directory, 'small_fail')

        # Get data for third subplot (large - passed)
        large_pass = self.get_trial_data(trial_directory, 'large_pass')

        # Initialize subplots
        fig, (ax0, ax1, ax2) = plt.subplots(nrows=1, ncols=3, sharey=True, figsize=(18, 5))

        # Set the scale for the y-axis
        ax0.set_ylim(-5, 5)

        ax0.plot(small_time, small_fail[0, start_time_idx:end_time_idx_small, 1, 9], color='blue')
        ax0.plot(small_time, small_fail[1, start_time_idx:end_time_idx_small, 1, 9], color='cornflowerblue')

        for trial in range(self.num_pull_trials):
            # Plot large pass (cut off first 2 seconds)
            ax2.plot(timestamp[start_time_idx:], large_pass[0, start_time_idx:, 1, trial], color='forestgreen')
            ax2.plot(timestamp[start_time_idx:], large_pass[1, start_time_idx:, 1, trial], color='limegreen')

            # Plot small fail (cut off first 2 seconds and anything after 8 seconds)
            ax1.plot(small_time, small_fail[0, start_time_idx:end_time_idx_small, 1, trial], color='blue')
            ax1.plot(small_time, small_fail[1, start_time_idx:end_time_idx_small, 1, trial], color='cornflowerblue')

        # Plot the max y global force threshold on each subplot
        ax0.plot(small_time, [4.0] * len(small_time), linestyle='--', color='black', label='Threshold')
        ax0.plot(small_time, [-4.0] * len(small_time), linestyle='--', color='black')
        ax1.plot(small_time, [4.0] * len(small_time), linestyle='--', color='black')
        ax1.plot(small_time, [-4.0] * len(small_time), linestyle='--', color='black')
        ax2.plot(timestamp[start_time_idx:], [4.0] * (self.cutoff_index - start_time_idx), linestyle='--', color='black')
        ax2.plot(timestamp[start_time_idx:], [-4.0] * (self.cutoff_index - start_time_idx), linestyle='--', color='black')

        # Plot shaded regions and text (first plot)
        shade_height = 4

        initial_grip = [start_time_idx, start_time_idx + 12]
        initial_grip_center_x = np.sum(timestamp[initial_grip]) / 2
        ax0.fill_between(timestamp[initial_grip[0]:initial_grip[1]], -shade_height, shade_height, color='gray', alpha=0.3)
        ax0.text(initial_grip_center_x, -shade_height + 0.5, textwrap.fill("Initial Grasp", width=10), horizontalalignment='center', verticalalignment='bottom')

        pull = [initial_grip[1] + 1, initial_grip[1] + 41]
        pull_center_x = np.sum(timestamp[pull]) / 2
        ax0.fill_between(timestamp[pull[0]:pull[1]], -shade_height, shade_height, color='gray', alpha=0.3)
        ax0.text(pull_center_x, -shade_height + 0.7, textwrap.fill("Pulling", width=10), horizontalalignment='center', verticalalignment='bottom')

        slip = [pull[1] + 1, pull[1] + 16]
        slip_center_x = np.sum(timestamp[slip]) / 2
        ax0.fill_between(timestamp[slip[0]:slip[1]], -shade_height, shade_height, color='gray', alpha=0.3)
        ax0.text(slip_center_x, -shade_height + 0.7, textwrap.fill("Slipping", width=10), horizontalalignment='center', verticalalignment='bottom')

        separated = [slip[1] + 1, end_time_idx_small]
        separated_center_x = np.sum(timestamp[separated]) / 2
        ax0.fill_between(timestamp[separated[0]:separated[1]], -shade_height, shade_height, color='gray', alpha=0.3)
        ax0.text(separated_center_x, -shade_height + 0.7, textwrap.fill("Parted", width=10), horizontalalignment='center', verticalalignment='bottom')

        # Text box identifying tactile arrays
        ax0.text(5.25, shade_height + 0.70, 'Array 1', horizontalalignment='center', verticalalignment='top', fontsize=12)
        ax0.text(5.25, -shade_height - 0.75, 'Array 0', horizontalalignment='center', verticalalignment='bottom', fontsize=12)
        # ax0.text()

        # Define the coordinate location for the legend
        x_coord = 0.19  # x-coordinate
        y_coord = 1  # y-coordinate

        # Place the legend at the specified coordinate location
        ax0.legend(loc='upper center', fancybox=True, ncol=1, bbox_to_anchor=(x_coord, y_coord))
        # ax0.legend(loc='upper left')

        # Plot titles
        ax0.set_ylabel('Global Y Force (N)')
        ax0.set_xlabel('Time (s)')
        ax1.set_xlabel('Time (s)')
        ax2.set_xlabel('Time (s)')
        ax1.title.set_text("Fail")
        ax2.title.set_text("Pass")

        plt.show()

    def lstm_learned_sum_error(self):
        data = self.load_data('/home/marcus/IMML/ros2_ws/data/gloved/trials/3/stats_0.csv')
        average_distance_error = self.extract_columns(data, header_starts_with=('average_distance_error'))

        # Set the figure size
        plt.figure(figsize=(5, 4))

        plt.plot(average_distance_error, label="Average Distance Error")
        plt.xlabel("Epochs", fontsize=12)
        plt.ylabel("Distance Error (mm)", fontsize=12)
        plt.show()

if __name__ == '__main__':
    name = 'small_fail_2'

    filename = f'/home/marcus/IMML/ros2_ws/src/IMML_cable_follow/trial_control/trial_control/resource/{name}.csv'    
    pltforce = PlotPapillarrayForce(filename)

    # pltforce.plot_pass_fail_trials()
    # pltforce.plot_single_cable_pull('grip')
    # pltforce.imece_plot()
    pltforce.lstm_learned_sum_error()