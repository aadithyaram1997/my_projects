import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt
import scipy.integrate as it
import matplotlib.pyplot as plt
import csv
import WiPy3

# Conversion factor from feet to meters per second squared
FEET_TO_METERS = 0.3048 # flightgear logs acceleration data in feet per second^2

# Gravity (optional. Could be subtracted with Acceleration_Z)
Acc_g = -9.78408 # g in flightgear

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
            'highpass_accel_x': highpass_accel_x,
            'highpass_accel_y': highpass_accel_y,
            'highpass_accel_z': highpass_accel_z,
            'highpass_accel_x_2': highpass_accel_x_2,
            'highpass_accel_y_2': highpass_accel_y_2,
            'highpass_accel_z_2': highpass_accel_z_2,
            'tilt_x' : tilt_x,
            'tilt_y' : tilt_y,

            'lowpass_accel_x' : lowpass_accel_x,
            'lowpass_accel_y' : lowpass_accel_y,

            'tilt_x_angle' : tilt_x_angle,
            'tilt_y_angle' : tilt_y_angle,
            'tilt_x_angle_deg' :tilt_x_angle_deg,
            'tilt_y_angle_deg' :tilt_y_angle_deg,
            
            'highpass_roll'  : highpass_roll,
            'highpass_pitch' : highpass_pitch,
            'highpass_yaw'   : highpass_yaw,
            'highpass_roll_2'  : highpass_roll_2, 
            'highpass_pitch_2' : highpass_pitch_2,
            'highpass_yaw_2'   : highpass_yaw_2,

            'washout_x' : washout_x,
            'washout_y' : washout_y,
            'washout_z' : washout_z,

        
            'x': pos_x,
            'y': pos_y,
            'z': pos_z,
            'washout_roll': washout_roll,
            'washout_pitch': washout_pitch,
            'washout_yaw' : washout_yaw,
            'yaw': total_yaw,
            'roll' : total_roll,
            'pitch' : total_pitch,
            'vel_x' : vel_x,
            'vel_y' : vel_y,
            'vel_z' : vel_z,
            'yaw_angle' : yaw_angle,
            'roll_angle' : roll_angle,
            'pitch_angle' : pitch_angle,
            # 'forcex' : forcex,
            # 'forcey' : forcey,
            # 'forcez' : forcez

        }

def setGeometry(Irobot, Ikin, Iws):
    WiPy3.createRobot()
    Irobot.setMotionPattern(8, 5)

    Irobot.setBase(0, 7.6029, 5.5952, 3.5665)
    Irobot.setBase(1, 7.5238, -5.5789, 3.5745)
    Irobot.setBase(2, -7.4614, -5.5621, 3.7536)
    Irobot.setBase(3, -7.3114, 5.6113, 3.7408)
    Irobot.setBase(4, 7.6126, 5.5812, -0.7817)
    Irobot.setBase(5, 7.5427, -5.5851, -0.8525)
    Irobot.setBase(6, -7.7641, -5.5694, -0.9384)
    Irobot.setBase(7, -7.6959, 5.6008, -0.9036)

    Irobot.setPlatform(0, 0.5488, 0.46, -0.48)
    Irobot.setPlatform(1, 0.5488, -0.46, -0.48)
    Irobot.setPlatform(2, -0.5488, -0.46, -0.48)
    Irobot.setPlatform(3, -0.5488, 0.46, -0.48)
    Irobot.setPlatform(4, 0.36, 0.7488, 0.48)
    Irobot.setPlatform(5, 0.36, -0.7488, 0.48)
    Irobot.setPlatform(6, -0.36, -0.7488, 0.48)
    Irobot.setPlatform(7, -0.36, 0.7488, 0.48)

    Irobot.setPlatformCenterOfGravity(0, 0, 0)
    Irobot.setPlatformInertiaMatrix(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    Ikin.setKinematicsModel(0)
    Ikin.setElasticityModel(1)
    Iws.setForceLimits(100, 3000)
    Iws.setWrench(0, 0, -1000, 0 , 0, 0)

def calcWorkspaceforce(Iws):
    Iws.setMethod(4)
    Iws.setWorkspaceCriterion(0)
    Iws.setIterations(6)
    Iws.calculateWorkspace()

def ForceDis(Irobot, x, y, z, roll, pitch, yaw):
    force_distribution = Irobot.getForceDistribution(x, y, z, roll, pitch, yaw)
    return force_distribution

def plot_force_distribution(data):
    rows = list(range(len(data)))
    columns = list(zip(*data))

    fig, axes = plt.subplots(2, 4, figsize=(20, 10), sharex=True, sharey=True)
    fig.suptitle('Force Distribution Across Rows')

    axes = axes.flatten()

    for i, (ax, column) in enumerate(zip(axes, columns)):
        ax.plot(rows, column)
        ax.set_title(f'Force Distribution {i + 1}')
        ax.set_xlabel('Row Index')
        ax.set_ylabel('Force Value')
        ax.grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

if __name__ == "__main__":
    from WiPy3 import Irobot, Ikin, Iws

    # Part 1: Process the motion data and save to CSV
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

    fs = 10
    hp_acc_x_1 = 0.1
    hp_acc_y_1 = 0.11
    hp_acc_z_1 = 0.1

    hp_acc_x_2 = 0.1
    hp_acc_y_2 = 0.11
    hp_acc_z_2 = 0.1

    lp_acc_x = 0.1
    lp_acc_y = 0.1
    lp_acc_z = 0.1

    hp_roll_1 = 0.000001
    hp_pitch_1 = 0.000001
    hp_yaw_1 = 0.000001

    hp_roll_2 = 0.000001
    hp_pitch_2 = 0.000001
    hp_yaw_2 = 0.000001

    scale_x_hp = 1
    scale_y_hp = 1
    scale_z_hp = 1

    scale_x_lp = 0.8
    scale_y_lp = 0.8

    scale_a_hp = 0.8
    scale_b_hp = 1
    scale_c_hp = 0.8
    
    washout_damping_x = 0.001
    washout_damping_y = 0.001
    washout_damping_z = 0.001
    washout_damping_a = 0.005
    washout_damping_b = 0.007
    washout_damping_c = 0.06

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

    output_csv_file = "C:\\Users\\mrf-ar\\motioncueing\\processed_motion_data_mrf_swiss.csv"
    output_df.to_csv(output_csv_file, index=False)

    # Part 2: Read the CSV file and calculate force distribution
    input_file = output_csv_file
    output_file = 'C:\\Users\\mrf-ar\\motioncueing\\Force distribution\\force_distribution_mrf_swiss.csv'

    setGeometry(Irobot, Ikin, Iws)
    calcWorkspaceforce(Iws)

    with open(input_file, mode='r') as infile, open(output_file, mode='w', newline='') as outfile:
        reader = csv.DictReader(infile)
        writer = csv.writer(outfile)

        for row in reader:
            x = float(row['x_position'])
            y = float(row['y_position'])
            z = float(row['z_position'])
            roll = float(row['roll_angle'])
            pitch = float(row['pitch_angle'])
            yaw = float(row['yaw_angle'])

            force_distribution = ForceDis(Irobot, x, y, z, roll, pitch, yaw)
            if force_distribution:
                writer.writerow(force_distribution)
            else:
                writer.writerow([0] * 8)
                print(f"Row with x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw} returned all zeros.")

    data = []
    with open(output_file, mode='r') as infile:
        reader = csv.reader(infile)
        for row in reader:
            data.append(list(map(float, row)))

    plot_force_distribution(data)

# if __name__ == "__main__":
#     main()

