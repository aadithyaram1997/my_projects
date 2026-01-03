import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt
import scipy.integrate as it
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# Conversion factor and constants
FEET_TO_METERS = 0.3048
Acc_g = -9.78408  # Gravity
Pilot_mass = 102.20
epsilon = 0.1  # Small value to avoid division by zero

# Butterworth filter
def butter_filter(data, cutoff, fs, order=2, filter_type='low'):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype=filter_type, analog=False)
    return filtfilt(b, a, data)

# Washout algorithm class
class ClassicalWashout:
    def __init__(self, fs, hp_acc_x_1, hp_acc_y_1, hp_acc_z_1, hp_acc_x_2, hp_acc_y_2, hp_acc_z_2,
                 lp_acc_x, lp_acc_y, lp_acc_z, scale_x_hp, scale_y_hp, scale_z_hp, scale_x_lp, scale_y_lp,
                 washout_damping_x, washout_damping_y, washout_damping_z, order=2):
        self.fs = fs
        self.hp_acc_x_1 = hp_acc_x_1
        self.hp_acc_y_1 = hp_acc_y_1
        self.hp_acc_z_1 = hp_acc_z_1
        self.hp_acc_x_2 = hp_acc_x_2
        self.hp_acc_y_2 = hp_acc_y_2
        self.hp_acc_z_2 = hp_acc_z_2
        self.lp_acc_x = lp_acc_x
        self.lp_acc_y = lp_acc_y
        self.lp_acc_z = lp_acc_z
        self.scale_x_hp = scale_x_hp
        self.scale_y_hp = scale_y_hp
        self.scale_z_hp = scale_z_hp
        self.scale_x_lp = scale_x_lp
        self.scale_y_lp = scale_y_lp
        self.washout_damping_x = washout_damping_x
        self.washout_damping_y = washout_damping_y
        self.washout_damping_z = washout_damping_z
        self.order = order

    def process(self, accel_data):
        # High-pass scaling
        scaled_ax_hp = accel_data['x'] * self.scale_x_hp
        scaled_ay_hp = accel_data['y'] * self.scale_y_hp
        scaled_az_hp = accel_data['z'] * self.scale_z_hp

        # First and second-order high-pass filters
        highpass_accel_x = butter_filter(scaled_ax_hp, self.hp_acc_x_1, self.fs, order=1, filter_type='high')
        highpass_accel_y = butter_filter(scaled_ay_hp, self.hp_acc_y_1, self.fs, order=1, filter_type='high')
        highpass_accel_z = butter_filter(scaled_az_hp, self.hp_acc_z_1, self.fs, order=1, filter_type='high')

        highpass_accel_x_2 = butter_filter(highpass_accel_x, self.hp_acc_x_2, self.fs, order=2, filter_type='high')
        highpass_accel_y_2 = butter_filter(highpass_accel_y, self.hp_acc_y_2, self.fs, order=2, filter_type='high')
        highpass_accel_z_2 = butter_filter(highpass_accel_z, self.hp_acc_z_2, self.fs, order=2, filter_type='high')

        # Integration for velocities and positions
        vel_x = it.cumulative_trapezoid(highpass_accel_x_2, dx=1 / self.fs, initial=0)
        vel_y = it.cumulative_trapezoid(highpass_accel_y_2, dx=1 / self.fs, initial=0)
        vel_z = it.cumulative_trapezoid(highpass_accel_z_2, dx=1 / self.fs, initial=0)

        pos_x = it.cumulative_trapezoid(vel_x, dx=1 / self.fs, initial=0) * np.exp(-self.washout_damping_x * np.arange(len(vel_x)) / self.fs)
        pos_y = it.cumulative_trapezoid(vel_y, dx=1 / self.fs, initial=0) * np.exp(-self.washout_damping_y * np.arange(len(vel_y)) / self.fs)
        pos_z = it.cumulative_trapezoid(vel_z, dx=1 / self.fs, initial=0) * np.exp(-self.washout_damping_z * np.arange(len(vel_z)) / self.fs)

        return {
            'x': pos_x,
            'y': pos_y,
            'z': pos_z
        }

# Cost function for optimization
def cost_x(damping_x, washout_algorithm, accel_data, time):
    washout_algorithm.washout_damping_x = damping_x[0]
    processed_movement = washout_algorithm.process(accel_data)
    force_x = accel_data['x'] * Pilot_mass
    force_x_post = np.gradient(np.gradient(processed_movement['x'], time), time) * Pilot_mass
    force_x_factor = np.abs(force_x_post - force_x) / (np.abs(force_x) + epsilon)
    return np.mean(force_x_factor)

def optimize_damping(washout_algorithm, accel_data, time):
    result_x = minimize(cost_x, [washout_algorithm.washout_damping_x], args=(washout_algorithm, accel_data, time), bounds=[(0.0001, 0.01)])
    optimized_damping_x = result_x.x[0]
    print(f"Optimized washout_damping_x: {optimized_damping_x}")

if __name__ == "__main__":
    csv_file = "path_to_your_input_csv_file.csv"  # Replace with your file path
    data = pd.read_csv(csv_file)

    accel_data = {
        'x': data['Acceleration_X'].values * FEET_TO_METERS,
        'y': data['Acceleration_Y'].values * FEET_TO_METERS,
        'z': data['Acceleration_Z'].values * FEET_TO_METERS - Acc_g
    }

    time = data['Time'].values

    # Initialize washout algorithm
    washout_algorithm = ClassicalWashout(
        fs=10,
        hp_acc_x_1=0.1, hp_acc_y_1=0.1, hp_acc_z_1=0.1,
        hp_acc_x_2=0.1, hp_acc_y_2=0.1, hp_acc_z_2=0.1,
        lp_acc_x=0.1, lp_acc_y=0.1, lp_acc_z=0.1,
        scale_x_hp=0.8, scale_y_hp=0.8, scale_z_hp=1,
        scale_x_lp=0.8, scale_y_lp=0.8,
        washout_damping_x=0.001, washout_damping_y=0.001, washout_damping_z=0.001
    )

    # Optimize damping
    optimize_damping(washout_algorithm, accel_data, time)
