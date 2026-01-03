import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt
import scipy.integrate as it
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import csv
import WiPy3

# Conversion factor from feet to meters per second squared
FEET_TO_METERS = 0.3048 # flightgear logs acceleration data in feet per second^2

# Gravity (optional. Could be subtracted with Acceleration_Z)
Acc_g = -9.78408 # g in flightgear
Pilot_mass = 102.20


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

force_x_storage = {}
force_y_storage = {}
force_z_storage = {}

def is_force_feasible(processed_movement, Irobot, time):
    for i in range(len(time)):
        x = processed_movement['x'][i]
        y = processed_movement['y'][i]
        z = processed_movement['z'][i]
        roll = processed_movement['roll'][i]
        pitch = processed_movement['pitch'][i]
        yaw = processed_movement['yaw'][i]

        force_distribution = ForceDis(Irobot, x, y, z, roll, pitch, yaw)
        
        if not force_distribution or all(f == 0 for f in force_distribution):
            return False  # infeasible if empty or all 0s

    return True  # only if all are feasible


def compute_force_error(processed, accel_data, axis, time, mass):
    if axis == 'x':
        raw_force = accel_data['x'] * mass
        vel = np.gradient(processed['x'], time)
        acc = np.gradient(vel, time)
        final_acc = acc + (-processed['lowpass_accel_x'] * Acc_g)
    elif axis == 'y':
        raw_force = accel_data['y'] * mass
        vel = np.gradient(processed['y'], time)
        acc = np.gradient(vel, time)
        final_acc = acc + (-processed['lowpass_accel_y'] * Acc_g)
    elif axis == 'z':
        raw_force = (accel_data['z'] + Acc_g) * mass
        vel = np.gradient(processed['z'], time)
        acc = np.gradient(vel, time)
        final_acc = acc + Acc_g

    post_force = final_acc * mass
    force_factor = np.abs(post_force - raw_force) / (np.abs(raw_force) + 3)
    force_factor = np.nan_to_num(force_factor)
    return np.mean(force_factor)

def evaluate_objective(damping_value, axis, washout_algorithm, accel_data, rotational_data, time, mass, Irobot):
    # 1. Set new damping
    setattr(washout_algorithm, f"washout_damping_{axis}", damping_value[0])

    # 2. Process the movement
    processed = washout_algorithm.process(accel_data, rotational_data)

    # 3. Force check before calculating cost
    if not is_force_feasible(processed, Irobot, time):
        return 1e6  # Penalize infeasible damping

    # 4. Compute cost only if feasible
    return compute_force_error(processed, accel_data, axis, time, mass)


def optimize_damping(washout_algorithm, accel_data, rotational_data, time, Irobot, mass):
    print("\n--- Damping Optimization Started ---")

    bounds = {
        'x': (0.0005, 0.02),
        'y': (0.0005, 0.02),
        'z': (0.0005, 0.02)
    }

    for axis in ['x', 'y', 'z']:
        init_val = getattr(washout_algorithm, f"washout_damping_{axis}")
        result = minimize(
            evaluate_objective,
            [init_val],
            args=(axis, washout_algorithm, accel_data, rotational_data, time, mass, Irobot),
            bounds=[bounds[axis]],
            method='L-BFGS-B',
            options={'maxiter': 100}
        )

        print(f"\n{axis.upper()} → Initial: {init_val:.5f}, Final: {result.x[0]:.5f}, Cost: {result.fun:.5f}")

        if result.success and result.fun < 1e5:
            setattr(washout_algorithm, f"washout_damping_{axis}", result.x[0])
            print(f"✔ Accepted washout_damping_{axis}: {result.x[0]}")
        else:
            print(f"✖ Optimization failed for {axis}. Keeping original value.")

    print("--- Optimization Finished ---")

def plot_force_x():
    if 'force_x' in force_x_storage:
        plt.figure(figsize=(10, 5))
        plt.plot(force_x_storage['time'], force_x_storage['force_x'], label="Raw Force X", linestyle="dashed")
        plt.plot(force_x_storage['time'], force_x_storage['force_x_post'], label="Processed Force X", linestyle="solid")
        plt.xlabel("Time (s)")
        plt.ylabel("Force (N)")
        plt.title("Comparison of Raw vs. Processed Force X")
        plt.legend()
        plt.grid(True)
        plt.show()
    else:
        print("No force data found! Ensure cost_x() was executed.")

def plot_force_x_factor():
    if 'force_x_factor' in force_x_storage:
        plt.figure(figsize=(10, 5))
        plt.plot(force_x_storage['time'], force_x_storage['force_x_factor'], label="Force X Factor", color='red')
        plt.xlabel("Time (s)")
        plt.ylabel("Force Factor")
        plt.title("Force X Factor Over Time")
        plt.legend()
        plt.grid(True)
        plt.show()
    else:
        print("No force factor data found! Ensure cost_x() was executed.")

def plot_force_y():
    if 'force_y' in force_y_storage:
        plt.figure(figsize=(10, 5))
        plt.plot(force_y_storage['time'], force_y_storage['force_y'], label="Raw Force Y", linestyle="dashed")
        plt.plot(force_y_storage['time'], force_y_storage['force_y_post'], label="Processed Force Y", linestyle="solid")
        plt.xlabel("Time (s)")
        plt.ylabel("Force (N)")
        plt.title("Comparison of Raw vs. Processed Force Y")
        plt.legend()
        plt.grid(True)
        plt.show()
    else:
        print("No force data found! Ensure cost_y() was executed.")

def plot_force_y_factor():
    if 'force_y_factor' in force_y_storage:
        plt.figure(figsize=(10, 5))
        plt.plot(force_y_storage['time'], force_y_storage['force_y_factor'], label="Force Y Factor", color='green')
        plt.xlabel("Time (s)")
        plt.ylabel("Force Factor")
        plt.title("Force Y Factor Over Time")
        plt.legend()
        plt.grid(True)
        plt.show()
    else:
        print("No force factor data found! Ensure cost_y() was executed.")

def plot_force_z():
    if 'force_z' in force_z_storage:
        plt.figure(figsize=(10, 5))
        plt.plot(force_z_storage['time'], force_z_storage['force_z'], label="Raw Force Z", linestyle="dashed")
        plt.plot(force_z_storage['time'], force_z_storage['force_z_post'], label="Processed Force Z", linestyle="solid")
        plt.xlabel("Time (s)")
        plt.ylabel("Force (N)")
        plt.title("Comparison of Raw vs. Processed Force Z")
        plt.legend()
        plt.grid(True)
        plt.show()
    else:
        print("No force data found! Ensure cost_z() was executed.")

def plot_force_z_factor():
    if 'force_z_factor' in force_z_storage:
        plt.figure(figsize=(10, 5))
        plt.plot(force_z_storage['time'], force_z_storage['force_z_factor'], label="Force Z Factor", color='blue')
        plt.xlabel("Time (s)")
        plt.ylabel("Force Factor")
        plt.title("Force Z Factor Over Time")
        plt.legend()
        plt.grid(True)
        plt.show()
    else:
        print("No force factor data found! Ensure cost_z() was executed.")



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

    time = data['Time'].values


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
    optimize_damping(washout_algorithm, accel_data, rotational_data, time, Irobot, Pilot_mass)

    #optimize_damping(washout_algorithm, accel_data, time)
    plot_force_x()  # Plot Force X data after optimization
    plot_force_x_factor()
    plot_force_y()
    plot_force_y_factor()
    plot_force_z()
    plot_force_z_factor()


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
    force_x = (accel_data['x'] * Pilot_mass)
    force_y = (accel_data['y'] * Pilot_mass)
    force_z = (accel_data['z'] + Acc_g) * Pilot_mass

    force_data = pd.DataFrame({
        'Time': data['Time'],
         'Force_X': force_x,
         'Force_Y': force_y,
         'Force_Z': force_z
    })
    force_output_csv_file = "pilot_forces.csv"
    force_data.to_csv(force_output_csv_file, index=False)



    forces_after_motion_cueing = pd.DataFrame({
        'Time': data['Time'],
        # 'Force_X_post': force_x_post,
        # 'Force_Y_post': force_y_post,
        # 'Force_Z_post': force_z_post
    })
    forces_after_motion_cueing.to_csv("forces_after_motion_cueing.csv", index=False)

   # Small epsilon to prevent division by zero
    epsilon = 0.1




    force_factor = pd.DataFrame({
        # 'Force_X_Factor': force_x_factor,
        # 'Force_Y_Factor': force_y_factor,
        # 'Force_Z_Factor': force_z_factor
    })
    force_factor.to_csv("force_factor.csv", index=False)

     

    # Part 2: Read the CSV file and calculate force distribution
    input_file = output_csv_file
    output_file = 'C:\\Users\\mrf-ar\\motioncueing\\Force distribution\\force_distribution.csv'

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

    force_dis_data = []
    with open(output_file, mode='r') as infile:
        reader = csv.reader(infile)
        for row in reader:
            force_dis_data.append(list(map(float, row)))

    plot_force_distribution(force_dis_data)

    plt.figure(figsize=(18, 10))

    # Acceleration data
    plt.subplot(421)
    plt.plot(data['Time'], accel_data['x'], label="X")
    plt.plot(data['Time'], accel_data['y'], label="Y")
    plt.plot(data['Time'], accel_data['z'], label="Z")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.legend()
    plt.title("Raw Acceleration Data")

    raw_vx = it.cumulative_trapezoid(accel_data['x'], dx=1 / fs, initial=0)
    raw_vy = it.cumulative_trapezoid(accel_data['y'], dx=1 / fs, initial=0)
    raw_vz = it.cumulative_trapezoid(accel_data['z'], dx=1 / fs, initial=0)

    raw_x = it.cumulative_trapezoid(raw_vx, dx=1 / fs, initial=0)
    raw_y = it.cumulative_trapezoid(raw_vy, dx=1 / fs, initial=0)
    raw_z = it.cumulative_trapezoid(raw_vz, dx=1 / fs, initial=0)

    plt.subplot(422)
    plt.plot(data['Time'], raw_x, label="x")
    plt.plot(data['Time'], raw_y, label="y")
    plt.plot(data['Time'], raw_z, label="z")
    plt.xlabel("Time (s)")
    plt.ylabel("m")
    plt.legend()
    plt.title("raw_pos")

    plt.subplot(423)
    plt.plot(data['Time'], processed_movement['highpass_accel_x'], label="X")
    plt.plot(data['Time'], processed_movement['highpass_accel_y'], label="Y")
    #plt.plot(data['Time'], processed_movement['highpass_accel_z'], label="Z")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.legend()
    plt.title("1st HP Accelerations")

    plt.subplot(424)
    plt.plot(data['Time'], processed_movement['highpass_accel_x_2'], label="X")
    plt.plot(data['Time'], processed_movement['highpass_accel_y_2'], label="Y")
    plt.plot(data['Time'], processed_movement['highpass_accel_z_2'], label="Z")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.legend()
    plt.title("2nd HP Accelerations")

    plt.subplot(425)
    plt.plot(data['Time'], processed_movement['washout_x'], label="X")
    plt.plot(data['Time'], processed_movement['washout_y'], label="Y")
    plt.plot(data['Time'], processed_movement['washout_z'], label="Z")
    plt.xlabel("Time (s)")
    plt.ylabel("acc (m/s²)")
    plt.legend()
    plt.title(" position before washout")

    plt.subplot(426)
    plt.plot(data['Time'], processed_movement['lowpass_accel_x'], label="X")
    plt.plot(data['Time'], processed_movement['lowpass_accel_y'], label="Y")
   # plt.plot(data['Time'], processed_movement['lowpass_accel_z'], label="Z")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.legend()
    plt.title("1st HP Accelerations")

    


    # plt.title("pos before washout")


    plt.subplot(427)
    plt.plot(data['Time'], processed_movement['x'], label="X Position")
    plt.plot(data['Time'], processed_movement['y'], label="Y Position")
    plt.plot(data['Time'], processed_movement['z'], label="Z Position")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.legend()
    plt.title("Processed Position Data")

    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(18, 10))





    # Rotational rates
    plt.subplot(521)
    plt.plot(data['Time'], rotational_data['roll'], label="Roll")
    plt.plot(data['Time'], rotational_data['pitch'], label="Pitch")
    plt.plot(data['Time'], rotational_data['yaw'], label="Yaw")
    plt.xlabel("Time (s)")
    plt.ylabel("Rate (deg/s)")
    plt.legend()
    plt.title("Raw Rotational Rates")

        # Angles (integrated from rates)
    roll_angle = it.cumulative_trapezoid(rotational_data['roll'], dx=1 / fs, initial=0)
    pitch_angle = it.cumulative_trapezoid(rotational_data['pitch'], dx=1 / fs, initial=0)
    yaw_angle = it.cumulative_trapezoid(rotational_data['yaw'], dx=1 / fs, initial=0)

    plt.subplot(522)
    plt.plot(data['Time'], roll_angle, label="Roll")
    plt.plot(data['Time'], pitch_angle, label="Pitch")
    plt.plot(data['Time'], yaw_angle, label="Yaw")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.legend()
    plt.title("Integrated Angles (from raw rates)")

    plt.subplot(523)
    plt.plot(data['Time'], processed_movement['highpass_roll'], label="roll")
    plt.plot(data['Time'], processed_movement['highpass_pitch'], label="pitch")
    plt.plot(data['Time'], processed_movement['highpass_yaw'], label="yaw")
    plt.xlabel("Time (s)")
    plt.ylabel("Rate (deg/s)")
    plt.legend()
    plt.title("1st HP Filtered Rotation rate")

    plt.subplot(524)
    plt.plot(data['Time'], processed_movement['highpass_roll_2'], label="roll")
    plt.plot(data['Time'], processed_movement['highpass_pitch_2'], label="pitch")
    plt.plot(data['Time'], processed_movement['highpass_yaw_2'], label="yaw")
    plt.xlabel("Time (s)")
    plt.ylabel("Rate (deg/s)")
    plt.legend()
    plt.title("2nd HP Filtered Rotation rate")

    plt.subplot(525)
    plt.plot(data['Time'], processed_movement['washout_roll'], label="roll")
    plt.plot(data['Time'], processed_movement['washout_pitch'], label="pitch")
    plt.plot(data['Time'], processed_movement['washout_yaw'], label="yaw")
    plt.xlabel("Time (s)")
    plt.ylabel("deg")
    plt.legend()
    plt.title("rotation angle before washout")



    plt.subplot(527)
    plt.plot(data['Time'], processed_movement['roll_angle'], label="roll")
    plt.plot(data['Time'], processed_movement['pitch_angle'], label="pitch")
    plt.plot(data['Time'], processed_movement['yaw_angle'], label="yaw")
    plt.xlabel("Time (s)")
    plt.ylabel("deg")
    plt.legend()
    plt.title("angle without tilt")

    plt.subplot(528)
    plt.plot(data['Time'], processed_movement['roll'], label="roll")
    plt.plot(data['Time'], processed_movement['pitch'], label="pitch")
    plt.plot(data['Time'], processed_movement['yaw'], label="yaw")
    plt.xlabel("Time (s)")
    plt.ylabel("deg")
    plt.legend()
    plt.title("angle with tilt")


   

    plt.tight_layout()
    plt.show()
