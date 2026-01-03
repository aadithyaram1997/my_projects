import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt
import scipy.integrate as it
from scipy.optimize import minimize
import csv
import WiPy3

# Conversion factor from feet to meters per second squared
FEET_TO_METERS = 0.3048
Acc_g = -9.78408

def butter_filter(data, cutoff, fs, order=2, filter_type='low'):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype=filter_type, analog=False)
    filtered_data = filtfilt(b, a, data)
    return filtered_data

class ClassicalWashout:
    def __init__(self, fs, hp_acc_x_1, lp_acc_x, hp_roll_1, scale_x_hp, scale_a_hp, washout_damping_x, washout_damping_a, order=2):
        self.fs = fs
        self.hp_acc_x_1 = hp_acc_x_1
        self.lp_acc_x = lp_acc_x
        self.hp_roll_1 = hp_roll_1
        self.scale_x_hp = scale_x_hp
        self.scale_a_hp = scale_a_hp
        self.washout_damping_x = washout_damping_x
        self.washout_damping_a = washout_damping_a
        self.order = order

    def process(self, accel_data, rotational_data):
        scaled_ax_hp = accel_data['x'] * self.scale_x_hp
        scaled_ay_hp = accel_data['y'] * self.scale_x_hp
        scaled_az_hp = accel_data['z'] * self.scale_x_hp
        scaled_ax_lp = accel_data['x'] * self.scale_x_hp

        highpass_accel_x = butter_filter(scaled_ax_hp, self.hp_acc_x_1, self.fs, order=1, filter_type='high')
        highpass_accel_y = butter_filter(scaled_ay_hp, self.hp_acc_x_1, self.fs, order=1, filter_type='high')
        highpass_accel_z = butter_filter(scaled_az_hp, self.hp_acc_x_1, self.fs, order=1, filter_type='high')

        vel_x = it.cumulative_trapezoid(highpass_accel_x, dx=1 / self.fs, initial=0)
        vel_y = it.cumulative_trapezoid(highpass_accel_y, dx=1 / self.fs, initial=0)
        vel_z = it.cumulative_trapezoid(highpass_accel_z, dx=1 / self.fs, initial=0)

        washout_x = it.cumulative_trapezoid(vel_x, dx=1 / self.fs, initial=0)
        washout_y = it.cumulative_trapezoid(vel_y, dx=1 / self.fs, initial=0)
        washout_z = it.cumulative_trapezoid(vel_z, dx=1 / self.fs, initial=0)

        pos_x = washout_x * np.exp(-self.washout_damping_x * np.arange(len(washout_x)) / self.fs)
        pos_y = washout_y * np.exp(-self.washout_damping_x * np.arange(len(washout_y)) / self.fs)
        pos_z = washout_z * np.exp(-self.washout_damping_x * np.arange(len(washout_z)) / self.fs)

        tilt_x = (scaled_ax_lp / -Acc_g)
        lowpass_accel_x = butter_filter(tilt_x, self.lp_acc_x, self.fs, self.order, 'low')
        tilt_x_angle = np.arcsin(np.clip(lowpass_accel_x, -1, 1))
        tilt_x_angle_deg = np.degrees(tilt_x_angle)

        scaled_roll = rotational_data['roll'] * self.scale_a_hp
        highpass_roll = butter_filter(scaled_roll, self.hp_roll_1, self.fs, order=1, filter_type='high')
        washout_roll = it.cumulative_trapezoid(highpass_roll, dx=1 / self.fs, initial=0)
        roll_angle = washout_roll * np.exp(-self.washout_damping_a * np.arange(len(washout_roll)) / self.fs)

        total_roll = roll_angle + tilt_x_angle_deg
        total_pitch = np.zeros_like(total_roll)  # Simplified for example
        total_yaw = np.zeros_like(total_roll)    # Simplified for example

        return {'x': pos_x, 'y': pos_y, 'z': pos_z, 'roll': total_roll, 'pitch': total_pitch, 'yaw': total_yaw}

def cost_function(params, accel_data, rotational_data, Irobot):
    hp_acc_x_1, lp_acc_x, hp_roll_1, scale_x_hp, scale_a_hp, washout_damping_x, washout_damping_a = params
    
    washout_alg = ClassicalWashout(fs=10, hp_acc_x_1=hp_acc_x_1, lp_acc_x=lp_acc_x, hp_roll_1=hp_roll_1,
                                   scale_x_hp=scale_x_hp, scale_a_hp=scale_a_hp,
                                   washout_damping_x=washout_damping_x, washout_damping_a=washout_damping_a)
    
    processed_pose = washout_alg.process(accel_data, rotational_data)
    forces = ForceDis(Irobot, processed_pose['x'], processed_pose['y'], processed_pose['z'],
                      processed_pose['roll'], processed_pose['pitch'], processed_pose['yaw'])
    
    penalty = sum([max(0, 100 - f) + max(0, f - 3000) for f in forces])
    return penalty

def main():
    from WiPy3 import Irobot, Ikin, Iws

    csv_file = "C:\\Users\\mrf-ar\\motioncueing\\Flight gear paramenters\\log_parameters_mrf_non_crash.csv"
    data = pd.read_csv(csv_file)

    accel_data = {
        'x': data['Acceleration_X'].values * FEET_TO_METERS,
        'y': data['Acceleration_Y'].values * FEET_TO_METERS,
        'z': data['Acceleration_Z'].values * FEET_TO_METERS - Acc_g
    }
    rotational_data = {
        'roll': data['Roll_rate'].values,
        'pitch': data['Pitch_rate'].values,
        'yaw': data['Yaw_rate'].values
    }

    setGeometry(Irobot, Ikin, Iws)
    calcWorkspaceforce(Iws)

    initial_params = [0.1, 0.1, 0.000001, 0.8, 0.8, 0.001, 0.005]
    bounds = [(0.01, 1.0), (0.01, 1.0), (0.000001, 0.01), (0.5, 1.5), (0.5, 1.5), (0.0001, 0.01), (0.0001, 0.01)]

    result = minimize(cost_function, initial_params, args=(accel_data, rotational_data, Irobot), bounds=bounds)

    print("Optimized parameters:", result.x)

    # Process and print the optimized poses
    optimized_params = result.x
    washout_alg = ClassicalWashout(fs=10, hp_acc_x_1=optimized_params[0], lp_acc_x=optimized_params[1], hp_roll_1=optimized_params[2],
                                   scale_x_hp=optimized_params[3], scale_a_hp=optimized_params[4],
                                   washout_damping_x=optimized_params[5], washout_damping_a=optimized_params[6])
    optimized_poses = washout_alg.process(accel_data, rotational_data)
    
    print("Optimized poses:")
    for i in range(len(optimized_poses['x'])):
        print(f"x: {optimized_poses['x'][i]}, y: {optimized_poses['y'][i]}, z: {optimized_poses['z'][i]}, roll: {optimized_poses['roll'][i]}, pitch: {optimized_poses['pitch'][i]}, yaw: {optimized_poses['yaw'][i]}")

if __name__ == "__main__":
    main()