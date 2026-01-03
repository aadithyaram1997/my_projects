import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt
import scipy.integrate as it
import matplotlib.pyplot as plt
import csv
import WiPy3

# Conversion factor from feet to meters per second squared
FEET_TO_METERS = 0.3048  # flightgear logs acceleration data in feet per second^2
Acc_g = -9.81  # Gravity in flightgear
Pilot_mass = 101.93
epsilon = 2 # Small value to prevent division by zero

def butter_filter(data, cutoff, fs, order=2, filter_type='low'):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype=filter_type, analog=False)
    filtered_data = filtfilt(b, a, data)
    return filtered_data

class ClassicalWashout:
    def __init__(self, fs, hp_acc_x_1, hp_acc_y_1, hp_acc_z_1, hp_acc_x_2, hp_acc_y_2, hp_acc_z_2, 
                 lp_acc_x, lp_acc_y, lp_acc_z, hp_roll_1, hp_pitch_1, hp_yaw_1, hp_roll_2, hp_pitch_2, hp_yaw_2,
                 scale_x_hp, scale_y_hp, scale_z_hp, scale_x_lp, scale_y_lp, scale_a_hp, scale_b_hp, scale_c_hp,
                 washout_damping_x, washout_damping_y, washout_damping_z, washout_damping_a, washout_damping_b, washout_damping_c,
                 order=2):
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
        self.hp_roll_1  = hp_roll_1
        self.hp_pitch_1 = hp_pitch_1
        self.hp_yaw_1   = hp_yaw_1
        self.hp_roll_2  = hp_roll_2
        self.hp_pitch_2 = hp_pitch_2
        self.hp_yaw_2   = hp_yaw_2
        self.scale_x_hp = scale_x_hp
        self.scale_y_hp = scale_y_hp
        self.scale_z_hp = scale_z_hp
        self.scale_x_lp = scale_x_lp
        self.scale_y_lp = scale_y_lp
        self.scale_a_hp = scale_a_hp
        self.scale_b_hp = scale_b_hp
        self.scale_c_hp = scale_c_hp
        self.washout_damping_x = washout_damping_x
        self.washout_damping_y = washout_damping_y
        self.washout_damping_z = washout_damping_z
        self.washout_damping_a = washout_damping_a
        self.washout_damping_b = washout_damping_b
        self.washout_damping_c = washout_damping_c
        self.order = order

    def process(self, accel_data, rotational_data):
        scaled_ax_hp = accel_data['x'] * self.scale_x_hp
        scaled_ay_hp = accel_data['y'] * self.scale_y_hp
        scaled_az_hp = accel_data['z'] * self.scale_z_hp
        scaled_ax_lp = accel_data['x'] * self.scale_x_lp
        scaled_ay_lp = accel_data['y'] * self.scale_y_lp

        highpass_accel_x = butter_filter(scaled_ax_hp, self.hp_acc_x_1, self.fs, order=1, filter_type='high')
        highpass_accel_y = butter_filter(scaled_ay_hp, self.hp_acc_y_1, self.fs, order=1, filter_type='high')
        highpass_accel_z = butter_filter(scaled_az_hp, self.hp_acc_z_1, self.fs, order=1, filter_type='high')

        highpass_accel_x_2 = butter_filter(highpass_accel_x, self.hp_acc_x_2, self.fs, order=2, filter_type='high')
        highpass_accel_y_2 = butter_filter(highpass_accel_y, self.hp_acc_y_2, self.fs, order=2, filter_type='high')
        highpass_accel_z_2 = butter_filter(highpass_accel_z, self.hp_acc_z_2, self.fs, order=2, filter_type='high')

        vel_x = it.cumulative_trapezoid(highpass_accel_x_2, dx=1 / self.fs, initial=0)
        vel_y = it.cumulative_trapezoid(highpass_accel_y_2, dx=1 / self.fs, initial=0)
        vel_z = it.cumulative_trapezoid(highpass_accel_z_2, dx=1 / self.fs, initial=0)

        washout_x = it.cumulative_trapezoid(vel_x, dx=1 / self.fs, initial=0)
        washout_y = it.cumulative_trapezoid(vel_y, dx=1 / self.fs, initial=0)
        washout_z = it.cumulative_trapezoid(vel_z, dx=1 / self.fs, initial=0)

        pos_x = washout_x * np.exp(-self.washout_damping_x * np.arange(len(washout_x)) / self.fs)
        pos_y = washout_y * np.exp(-self.washout_damping_y * np.arange(len(washout_y)) / self.fs)
        pos_z = washout_z * np.exp(-self.washout_damping_z * np.arange(len(washout_z)) / self.fs)

        tilt_x = (scaled_ax_lp / -Acc_g)
        tilt_y = (scaled_ay_lp / -Acc_g)

        lowpass_accel_x = butter_filter(tilt_x, self.lp_acc_x, self.fs, self.order, 'low')
        lowpass_accel_y = butter_filter(tilt_y, self.lp_acc_y, self.fs, self.order, 'low')

        tilt_x_angle = np.arcsin(np.clip(lowpass_accel_x, -1, 1))
        tilt_y_angle = np.arcsin(np.clip(lowpass_accel_y, -1, 1))

        tilt_x_angle_deg = np.degrees(tilt_x_angle)
        tilt_y_angle_deg = np.degrees(tilt_y_angle)

        scaled_roll = rotational_data['roll'] * self.scale_a_hp
        scaled_pitch = rotational_data['pitch'] * self.scale_b_hp
        scaled_yaw = rotational_data['yaw'] * self.scale_c_hp

        highpass_roll = butter_filter(scaled_roll, self.hp_roll_1, self.fs, order=1, filter_type='high')
        highpass_pitch = butter_filter(scaled_pitch, self.hp_pitch_1, self.fs, order=1, filter_type='high')
        highpass_yaw = butter_filter(scaled_yaw, self.hp_yaw_1, self.fs, order=1, filter_type='high')

        highpass_roll_2 = butter_filter(highpass_roll, self.hp_roll_2, self.fs, order=2, filter_type='high')
        highpass_pitch_2 = butter_filter(highpass_pitch, self.hp_pitch_2, self.fs, order=2, filter_type='high')
        highpass_yaw_2 = butter_filter(highpass_yaw, self.hp_yaw_2, self.fs, order=2, filter_type='high')

        washout_roll = it.cumulative_trapezoid(highpass_roll_2, dx=1 / self.fs, initial=0)
        washout_pitch = it.cumulative_trapezoid(highpass_pitch_2, dx=1 / self.fs, initial=0)
        washout_yaw = it.cumulative_trapezoid(highpass_yaw_2, dx=1 / self.fs, initial=0)

        roll_angle = washout_roll * np.exp(-self.washout_damping_a * np.arange(len(washout_roll)) / self.fs)
        pitch_angle = washout_pitch * np.exp(-self.washout_damping_b * np.arange(len(washout_pitch)) / self.fs)
        yaw_angle = washout_yaw * np.exp(-self.washout_damping_c * np.arange(len(washout_yaw)) / self.fs)

        total_roll = roll_angle + tilt_y_angle_deg
        total_pitch = pitch_angle + tilt_x_angle_deg
        total_yaw = yaw_angle

        return {
            'x': pos_x,
            'y': pos_y,
            'z': pos_z,
            'roll': total_roll,
            'pitch': total_pitch,
            'yaw': total_yaw
        }

def calculate_force_factors(data, processed_data):
    force_x_raw = (data['Acceleration_X'].values * FEET_TO_METERS * Pilot_mass)
    force_y_raw = (data['Acceleration_Y'].values * FEET_TO_METERS * Pilot_mass)
    force_z_raw = ((data['Acceleration_Z'].values * FEET_TO_METERS - Acc_g) * Pilot_mass)

    v_x = np.gradient(processed_data['x'], data['Time'])
    v_y = np.gradient(processed_data['y'], data['Time'])
    v_z = np.gradient(processed_data['z'], data['Time'])

    accel_x_post = np.gradient(v_x, data['Time'])
    accel_y_post = np.gradient(v_y, data['Time'])
    accel_z_post = np.gradient(v_z, data['Time'])

    force_x_post = accel_x_post * Pilot_mass
    force_y_post = accel_y_post * Pilot_mass
    force_z_post = accel_z_post * Pilot_mass

    force_x_factor = np.abs(force_x_post - force_x_raw) / (np.abs(force_x_raw) + epsilon)
    force_y_factor = np.abs(force_y_post - force_y_raw) / (np.abs(force_y_raw) + epsilon)
    force_z_factor = np.abs(force_z_post - force_z_raw) / (np.abs(force_z_raw) + epsilon)

    force_x_factor = np.nan_to_num(np.where(np.isfinite(force_x_factor), force_x_factor, 0))
    force_y_factor = np.nan_to_num(np.where(np.isfinite(force_y_factor), force_y_factor, 0))
    force_z_factor = np.nan_to_num(np.where(np.isfinite(force_z_factor), force_z_factor, 0))

    force_factors = pd.DataFrame({
        'Time': data['Time'],
        'Force_X_Factor': force_x_factor,
        'Force_Y_Factor': force_y_factor,
        'Force_Z_Factor': force_z_factor
    })

    force_factors.to_csv("force_factors.csv", index=False)
    num_elements = len(force_x_factor)
    print(f"Number of elements in force_x_factor: {num_elements}")

    average_force_x_factor = force_x_factor.mean()
    average_force_y_factor = force_y_factor.mean()
    average_force_z_factor = force_z_factor.mean()


# Print the average ratio
    print(f"Average Force_X_Factor: {average_force_x_factor}")
    print(f"Average Force_Y_Factor: {average_force_y_factor}")
    print(f"Average Force_Z_Factor: {average_force_z_factor}")


    force_factor_csv_file = "force_factor.csv"  # Replace with your CSV file path
    force_factor_data = pd.read_csv(force_factor_csv_file)

# Ensure the 'Force_X_Factor' column is numeric
    force_factor_data['Force_X_Factor'] = pd.to_numeric(force_factor_data['Force_X_Factor'], errors='coerce')

# Calculate the average
    average_force_x_factor_csv = force_factor_data['Force_X_Factor'].mean()

# Print the average
    print(f"Average Force_X_Factor from CSV: {average_force_x_factor_csv}")

# Ensure the 'Force_X_Factor' column is numeric
    force_factor_data['Force_Y_Factor'] = pd.to_numeric(force_factor_data['Force_Y_Factor'], errors='coerce')

# Calculate the average
    average_force_y_factor_csv = force_factor_data['Force_Y_Factor'].mean()

# Print the average
    print(f"Average Force_Y_Factor from CSV: {average_force_y_factor_csv}")
 # Ensure the 'Force_X_Factor' column is numeric
    force_factor_data['Force_Z_Factor'] = pd.to_numeric(force_factor_data['Force_Z_Factor'], errors='coerce')

# Calculate the average
    average_force_z_factor_csv = force_factor_data['Force_Z_Factor'].mean()

# Print the average
    print(f"Average Force_Z_Factor from CSV: {average_force_z_factor_csv}")       




    return force_factors

if __name__ == "__main__":
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

    fs = 10  # Sampling frequency (Hz)
    hp_acc_x_1 = 0.05 # High-pass filter cutoff frequency (Hz)
    hp_acc_y_1 = 0.05   
    hp_acc_z_1 = 0.05
        
    hp_acc_x_2 = 0.05  # High-pass filter cutoff frequency (Hz)
    hp_acc_y_2 = 0.05
    hp_acc_z_2 = 0.05

    
    lp_acc_x = 0.05  # Low-pass filter cutoff frequency for acceleration used for tilt coordination (Hz)
    lp_acc_y = 0.05
    lp_acc_z = 0.05

    hp_roll_1 = 0.00001 #High-pass filter cutoff frequency for rotations 1st order(Hz)
    hp_pitch_1 = 0.00001
    hp_yaw_1 = 0.00001

    hp_roll_2 = 0.00001  #High-pass filter cutoff frequency for rotations 2nd order (Hz)
    hp_pitch_2 = 0.00001
    hp_yaw_2 = 0.00001

    scale_x_hp = 0.8
    scale_y_hp = 0.8
    scale_z_hp = 0.8
    scale_x_lp = 0.8
    scale_y_lp = 0.8
    
    scale_a_hp = 0.8 
    scale_b_hp = 0.8
    scale_c_hp = 0.65

    washout_damping_x = 0.001
    washout_damping_y = 0.005
    washout_damping_z = 0.001
    washout_damping_a = 0.005
    washout_damping_b = 0.007
    washout_damping_c = 0.01


    washout_algorithm = ClassicalWashout(fs, hp_acc_x_1, hp_acc_y_1, hp_acc_z_1, hp_acc_x_2, hp_acc_y_2, hp_acc_z_2, lp_acc_x, lp_acc_y, lp_acc_z,
                                         hp_roll_1, hp_pitch_1, hp_yaw_1, hp_roll_2, hp_pitch_2, hp_yaw_2,
                                         scale_x_hp, scale_y_hp, scale_z_hp, scale_x_lp, scale_y_lp, scale_a_hp, scale_b_hp, scale_c_hp,
                                         washout_damping_x, washout_damping_y, washout_damping_z, washout_damping_a, washout_damping_b, washout_damping_c)

    processed_movement = washout_algorithm.process(accel_data, rotational_data)

    output_df = pd.DataFrame({
        'time': data['Time'],
        'x_position': processed_movement['x'],
        'y_position': processed_movement['y'],
        'z_position': processed_movement['z'],
        'roll_angle': processed_movement['roll'],
        'pitch_angle': processed_movement['pitch'],
        'yaw_angle': processed_movement['yaw']
    })

    output_csv_file = "C:\\Users\\mrf-ar\\motioncueing\\processed_motion_data.csv"
    output_df.to_csv(output_csv_file, index=False)

    force_factors = calculate_force_factors(data, processed_movement)

    plt.figure(figsize=(12, 6))
    #plt.plot(force_factors['Time'], force_factors['Force_X_Factor'], label="Force_X_Factor")
    plt.plot(force_factors['Time'], force_factors['Force_Y_Factor'], label="Force_Y_Factor")
   # plt.plot(force_factors['Time'], force_factors['Force_Z_Factor'], label="Force_Z_Factor")
    plt.xlabel("Time (s)")
    plt.ylabel("Normalized Force Factor")
    plt.title("Normalized Force Factors Over Time")
    plt.legend()
    plt.show()

# if __name__ == "__main__":
#     main()
