import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt
import scipy.integrate as it
import matplotlib.pyplot as plt
#import math

# Conversion factor from feet to meters per second squared
FEET_TO_METERS = 0.3048 # flightgear logs acceleration data in feet per second^2

#Gravity (optional. Could be subtracted with Acceleration_Z)
Acc_g = -9.78408 # g in flightgear


# Manual implementation of cumulative trapezoidal integration but now can be done from import scipy.integrate with cumulative_trapezoid()
#def cumtrapz(y, x=None, dx=1.0, initial=0.0):
#    y = np.asarray(y)
#    if x is None:
#        x = np.arange(len(y)) * dx
#    x = np.asarray(x)

    # Compute the cumulative trapezoidal integration
#    return np.concatenate(([initial], np.cumsum((y[1:] + y[:-1]) * (x[1:] - x[:-1]) / 2.0)))
# Butterworth filter
def butter_filter(data, cutoff, fs, order=2, filter_type='low'):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype=filter_type, analog=False)
    filtered_data = filtfilt(b, a, data)
    return filtered_data


# Washout algorithm class
class ClassicalWashout:
    def __init__(self, fs, hp_acc_x_1, hp_acc_y_1, hp_acc_z_1,hp_acc_x_2, hp_acc_y_2, hp_acc_z_2, lp_acc_x, lp_acc_y, lp_acc_z, hp_roll_1, hp_pitch_1, hp_yaw_1, hp_roll_2, hp_pitch_2, hp_yaw_2,
                 scale_x_hp, scale_y_hp, scale_z_hp,
                 scale_x_lp, scale_y_lp, scale_a_hp, scale_b_hp, scale_c_hp , washout_damping_x, washout_damping_y, washout_damping_z, washout_damping_a, washout_damping_b, washout_damping_c,
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


        #self.lowpass_cutoff_acc = lowpass_cutoff_acc
        #self.highpass_cutoff_rot = highpass_cutoff_rot

        self.order = order

    def process(self, accel_data, rotational_data):

        scaled_ax_hp = accel_data['x'] * self.scale_x_hp
        scaled_ay_hp = accel_data['y'] * self.scale_y_hp
        scaled_az_hp = accel_data['z'] * self.scale_z_hp

        scaled_ax_lp = accel_data['x'] * self.scale_x_lp
        scaled_ay_lp = accel_data['y'] * self.scale_y_lp

        # first order High-pass filter for accelerations
        highpass_accel_x = butter_filter(scaled_ax_hp, self.hp_acc_x_1, self.fs, order = 1, filter_type = 'high')
        highpass_accel_y = butter_filter(scaled_ay_hp, self.hp_acc_y_1, self.fs, order = 1, filter_type = 'high')
        highpass_accel_z = butter_filter(scaled_az_hp, self.hp_acc_z_1, self.fs, order = 1, filter_type = 'high')

        #print(highpass_accel_y)

           # second order High-pass filter for accelerations
        highpass_accel_x_2 = butter_filter(highpass_accel_x, self.hp_acc_x_2, self.fs, order = 2, filter_type = 'high')
        highpass_accel_y_2 = butter_filter(highpass_accel_y, self.hp_acc_y_2, self.fs, order = 2, filter_type = 'high')
        highpass_accel_z_2 = butter_filter(highpass_accel_z, self.hp_acc_z_2, self.fs, order = 2, filter_type = 'high')

        #Damping for washout

        washout_x = highpass_accel_x_2 * np.exp(-self.washout_damping_x * np.arange(len(highpass_accel_x_2)) / self.fs)
        washout_y = highpass_accel_y_2 * np.exp(-self.washout_damping_y * np.arange(len(highpass_accel_y_2)) / self.fs)
        washout_z = highpass_accel_z_2 * np.exp(-self.washout_damping_z * np.arange(len(highpass_accel_z_2)) / self.fs)

                # Integrate accelerations to obtain velocities and then positions
        vel_x = it.cumulative_trapezoid(washout_x, dx=1 / self.fs, initial=0)
        vel_y = it.cumulative_trapezoid(washout_y, dx=1 / self.fs, initial=0)
        vel_z = it.cumulative_trapezoid(washout_z, dx=1 / self.fs, initial=0)

        pos_x = it.cumulative_trapezoid(vel_x, dx=1 / self.fs, initial=0)
        pos_y = it.cumulative_trapezoid(vel_y, dx=1 / self.fs, initial=0)
        pos_z = it.cumulative_trapezoid(vel_z, dx=1 / self.fs, initial=0)


        

        tilt_x = (scaled_ax_lp/-(Acc_g))
        tilt_y = (scaled_ay_lp/-(Acc_g))



        #Low-pass filter for accelerations (for tilt coordination)
        lowpass_accel_x = butter_filter(tilt_x, self.lp_acc_x, self.fs, self.order, 'low')
        lowpass_accel_y = butter_filter(tilt_y, self.lp_acc_y, self.fs, self.order, 'low')

        tilt_x_angle = np.arcsin(np.clip(lowpass_accel_x, -1, 1))
        tilt_y_angle = np.arcsin(np.clip(lowpass_accel_y, -1, 1))

        tilt_x_angle_deg = np.degrees(tilt_x_angle)
        tilt_y_angle_deg = np.degrees(tilt_y_angle)

        #print(tilt_x_angle_deg)

        scaled_roll = rotational_data['roll'] * self.scale_a_hp
        scaled_pitch = rotational_data['pitch'] * self.scale_b_hp
        scaled_yaw = rotational_data['yaw'] * self.scale_c_hp

        #1st high-pass filter for rotational rates 
        highpass_roll  = butter_filter(scaled_roll,  self.hp_roll_1,  self.fs, order =1, filter_type= 'high')
        highpass_pitch = butter_filter(scaled_pitch, self.hp_pitch_1, self.fs, order =1, filter_type= 'high')
        highpass_yaw   = butter_filter(scaled_yaw,   self.hp_yaw_1,   self.fs, order =1, filter_type= 'high')

        #2nd high-pass filter for rotational rates 
        highpass_roll_2  = butter_filter(highpass_roll,  self.hp_roll_2,  self.fs, order =2, filter_type= 'high')
        highpass_pitch_2 = butter_filter(highpass_pitch, self.hp_pitch_2, self.fs, order =2, filter_type= 'high')
        highpass_yaw_2   = butter_filter(highpass_yaw,   self.hp_yaw_2,   self.fs, order =2, filter_type= 'high')

        washout_roll = highpass_roll_2 * np.exp(-self.washout_damping_a * np.arange(len(highpass_roll_2)) / self.fs)
        washout_pitch = highpass_pitch_2 * np.exp(-self.washout_damping_b * np.arange(len(highpass_pitch_2)) / self.fs)
       # washout_yaw = highpass_yaw_2 * np.exp(-self.washout_damping_c * np.arange(len(highpass_yaw_2)) / self.fs)
        #washout_yaw = highpass_yaw_2 * np.exp(-np.linspace(0,6,len(highpass_yaw_2)))
        




        # Integrate rotational rates to obtain angles
        roll_angle = it.cumulative_trapezoid(washout_roll, dx=1 / self.fs, initial=0)
        pitch_angle = it.cumulative_trapezoid(washout_pitch, dx=1 / self.fs, initial=0)
        yaw_angle = it.cumulative_trapezoid(highpass_yaw_2, dx=1 / self.fs, initial=0)

        # Apply washout process to positions and angles
        #Start and stop of washout can be adjusted accordingly
        #washout_x = pos_x * np.exp(-np.linspace(0, 0, len(pos_x)))
        #washout_y = pos_y * np.exp(-np.linspace(0, 0, len(pos_y)))
        #washout_z = pos_z * np.exp(-np.linspace(0, 0, len(pos_z)))

        #washout_roll = roll_angle * np.exp(-np.linspace(0, 0, len(roll_angle)))
       # washout_pitch = pitch_angle * np.exp(-np.linspace(0, 0, len(pitch_angle)))
        washout_yaw = yaw_angle * np.exp(-self.washout_damping_c * np.arange(len(yaw_angle)) / self.fs)

        total_roll =  roll_angle + tilt_y_angle_deg
        total_pitch = pitch_angle + tilt_x_angle_deg
        total_yaw = washout_yaw

       # print(washout_yaw)


        return {
            #'raw_pos_x': pos_x,
            #'raw_pos_y': pos_y,
            #'raw_pos_z': pos_z,
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
            'pitch_angle' : pitch_angle
 
        }


# processes
if __name__ == "__main__":
    # Load the CSV file
    csv_file = "C:\\Users\\mrf-ar\\motioncueing\\Flight gear paramenters\\log_parameters_mrf_non_crash.csv"  # Replace with your file path
    data = pd.read_csv(csv_file)

    # Convert acceleration data from feet per second squared to meters per second squared
    accel_data = {
        'x': data['Acceleration_X'].values * FEET_TO_METERS ,
        'y': data['Acceleration_Y'].values * FEET_TO_METERS,
        'z': data['Acceleration_Z'].values * FEET_TO_METERS - Acc_g
    }
    rotational_data = {
        'roll': data['Roll_rate'].values,
        'pitch': data['Pitch_rate'].values,
        'yaw': data['Yaw_rate'].values
    }

    # Initialize the washout algorithm
    fs = 10  # Sampling frequency (Hz)
    hp_acc_x_1 = 0.1 # High-pass filter cutoff frequency (Hz)
    hp_acc_y_1 = 0.11
    hp_acc_z_1 = 0.1
        
    hp_acc_x_2 = 0.1  # High-pass filter cutoff frequency (Hz)
    hp_acc_y_2 = 0.11
    hp_acc_z_2 = 0.1

    
    lp_acc_x = 0.1  # Low-pass filter cutoff frequency for acceleration used for tilt coordination (Hz)
    lp_acc_y = 0.1
    lp_acc_z = 0.1

    hp_roll_1 = 0.000001  #High-pass filter cutoff frequency for rotations 1st order(Hz)
    hp_pitch_1 = 0.000001
    hp_yaw_1 = 0.000001

    hp_roll_2 = 0.000001  #High-pass filter cutoff frequency for rotations 2nd order (Hz)
    hp_pitch_2 = 0.000001
    hp_yaw_2 = 0.000001

    scale_x_hp = 0.8
    scale_y_hp = 0.8
    scale_z_hp = 1
    scale_x_lp = 0.8
    scale_y_lp = 0.8
    
    scale_a_hp = 0.8 
    scale_b_hp = 1
    scale_c_hp = 0.8

    washout_damping_x = 0.001
    washout_damping_y = 0.001
    washout_damping_z = 0.001
    washout_damping_a = 0.001
    washout_damping_b = 0.001
    washout_damping_c = 0.001

   

    washout_algorithm = ClassicalWashout(fs, hp_acc_x_1, hp_acc_y_1, hp_acc_z_1,hp_acc_x_2, hp_acc_y_2, hp_acc_z_2, lp_acc_x, lp_acc_y, lp_acc_z, hp_roll_1, hp_pitch_1, hp_yaw_1,hp_roll_2, hp_pitch_2, hp_yaw_2, scale_x_hp, scale_y_hp, scale_z_hp,
                 scale_x_lp, scale_y_lp, scale_a_hp, scale_b_hp, scale_c_hp,  washout_damping_x, washout_damping_y, washout_damping_z, washout_damping_a, washout_damping_b, washout_damping_c)

    # Process the data
    processed_movement = washout_algorithm.process(accel_data, rotational_data)

    # Save processed data to CSV
    output_df = pd.DataFrame({
        'time': data['Time'],
        'x_position': processed_movement['x'],
        'y_position': processed_movement['y'],
        'z_position': processed_movement['z'],
        'roll_angle': processed_movement['roll'],
        'pitch_angle': processed_movement['pitch'],
        'yaw_angle': processed_movement['yaw']
    })

    # Replace with your desired output file path
    output_csv_file = "processed_motion_data.csv"
    output_df.to_csv(output_csv_file, index=False)

    #  Plot raw and processed data
    plt.figure(figsize=(18, 10))

    # Acceleration data
    plt.subplot(521)
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

    plt.subplot(522)
    plt.plot(data['Time'], raw_x, label="x")
    plt.plot(data['Time'], raw_y, label="y")
    plt.plot(data['Time'], raw_z, label="z")
    plt.xlabel("Time (s)")
    plt.ylabel("m")
    plt.legend()
    plt.title("raw_pos")

    plt.subplot(523)
    plt.plot(data['Time'], processed_movement['highpass_accel_x'], label="X")
    plt.plot(data['Time'], processed_movement['highpass_accel_y'], label="Y")
    plt.plot(data['Time'], processed_movement['highpass_accel_z'], label="Z")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.legend()
    plt.title("1st HP Accelerations")

    plt.subplot(524)
    plt.plot(data['Time'], processed_movement['highpass_accel_x_2'], label="X")
    plt.plot(data['Time'], processed_movement['highpass_accel_y_2'], label="Y")
    plt.plot(data['Time'], processed_movement['highpass_accel_z_2'], label="Z")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.legend()
    plt.title("2nd HP Accelerations")

    plt.subplot(525)
    plt.plot(data['Time'], processed_movement['washout_x'], label="X")
    plt.plot(data['Time'], processed_movement['washout_y'], label="Y")
    plt.plot(data['Time'], processed_movement['washout_z'], label="Z")
    plt.xlabel("Time (s)")
    plt.ylabel("acc (m/s²)")
    plt.legend()
    plt.title(" Washout acceleration")

    


    before_wo_vx = it.cumulative_trapezoid(processed_movement['highpass_accel_x_2'], dx=1 / fs, initial=0)
    before_wo_vy = it.cumulative_trapezoid(processed_movement['highpass_accel_y_2'], dx=1 / fs, initial=0)
    before_wo_vz = it.cumulative_trapezoid(processed_movement['highpass_accel_z_2'], dx=1 / fs, initial=0)

    before_wo_x = it.cumulative_trapezoid(before_wo_vx, dx=1 / fs, initial=0)
    before_wo_y = it.cumulative_trapezoid(before_wo_vy, dx=1 / fs, initial=0)
    before_wo_z = it.cumulative_trapezoid(before_wo_vz, dx=1 / fs, initial=0)

    plt.subplot(526)
    plt.plot(data['Time'],  before_wo_x, label="x")
    plt.plot(data['Time'],  before_wo_vy, label="y")
    plt.plot(data['Time'],  before_wo_z, label="z")
    plt.xlabel("Time (s)")
    plt.ylabel("m")
    plt.legend()
    plt.title("velocity before washout")

    plt.subplot(527)
    plt.plot(data['Time'],  before_wo_x, label="x")
    plt.plot(data['Time'],  before_wo_y, label="y")
    plt.plot(data['Time'],  before_wo_z, label="z")
    plt.xlabel("Time (s)")
    plt.ylabel("m")
    plt.legend()
    plt.title("pos before washout")


    plt.subplot(528)
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
    plt.ylabel("rate deg/s")
    plt.legend()
    plt.title(" Washout rotational rate")

    before_wo_a = it.cumulative_trapezoid(processed_movement['highpass_roll_2'], dx=1 / fs, initial=0)
    before_wo_b = it.cumulative_trapezoid(processed_movement['highpass_pitch_2'], dx=1 / fs, initial=0)
    before_wo_c = it.cumulative_trapezoid(processed_movement['highpass_yaw_2'], dx=1 / fs, initial=0)

    plt.subplot(526)
    plt.plot(data['Time'],  before_wo_a, label="roll")
    plt.plot(data['Time'],  before_wo_b, label="pitch")
    plt.plot(data['Time'],  before_wo_c, label="yaw")
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


    # plt.subplot(624)
    # #plt.plot(data['Time'], processed_movement['highpass_accel_x_2'], label="X Acceleration (High-Pass)")
    # #plt.plot(data['Time'], processed_movement['highpass_accel_y_2'], label="Y Acceleration (High-Pass)")
    # plt.plot(data['Time'], processed_movement['highpass_accel_z_2'], label="Z Acceleration (High-Pass)")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Acceleration (m/s²)")
    # plt.legend()
    # plt.title("High-Pass Filtered Accelerations")

   
    # plt.subplot(529)
    # plt.plot(data['Time'], processed_movement['vel_x'], label="x")
    # plt.plot(data['Time'], processed_movement['vel_y'], label="Y")
    # plt.plot(data['Time'], processed_movement['vel_z'], label="Z")
    # plt.xlabel("Time (s)")
    # plt.ylabel("vel (m/s)")
    # plt.legend()
    # plt.title("velocity")

    # plt.subplot(628)
    # #plt.plot(data['Time'], processed_movement['yaw_angle'], label="yaw_afterHP")
    # #plt.plot(data['Time'], processed_movement['roll_angle'], label="roll_afterHP")
    # plt.plot(data['Time'], processed_movement['pitch_angle'], label="pitch_afterHP")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Angle (deg)")
    # plt.legend()
    # plt.title(" after high pass(from rates)")

    # plt.subplot(624)
    # plt.plot(data['Time'], processed_movement['highpass_roll'], label="roll Rate (Low-Pass)")
    # #plt.plot(data['Time'], processed_movement['highpass_pitch'], label="pitch Rate (Low-Pass)")
    # #plt.plot(data['Time'], processed_movement['highpass_yaw'], label="yaw Rate (High-Pass)")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Yaw Rate (deg/s)")
    # plt.legend()
    # plt.title("high-Pass Filtered Rotations")





    # plt.subplot(626)
    # plt.plot(data['Time'], processed_movement['washout_x'], label="X")
    # plt.plot(data['Time'], processed_movement['washout_y'], label="Y")
    # plt.plot(data['Time'], processed_movement['washout_z'], label="Z")
    # plt.xlabel("Time (s)")
    # plt.ylabel("acc (m/s²)")
    # plt.legend()
    # plt.title(" Washout acceleration")

    



#    # Plot Generated motion cues for angles
#     plt.subplot(629)
#     #plt.plot(data['Time'], processed_movement['roll'], label="Roll Angle")
#     plt.plot(data['Time'], processed_movement['pitch'], label="Pitch Angle")
#    # plt.plot(data['Time'], processed_movement['yaw'], label="Yaw Angle")
#     plt.xlabel("Time (s)")
#     plt.ylabel("Angle (deg)")
#     plt.legend()
#     plt.title("Processed Angle Data")

    # Plot Generated motion cues of position
    # plt.subplot(627)
    # plt.plot(data['Time'], processed_movement['x'], label="X Position")
    # plt.plot(data['Time'], processed_movement['y'], label="Y Position")
    # plt.plot(data['Time'], processed_movement['z'], label="Z Position")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Position (m)")
    # plt.legend()
    # plt.title("Processed Position Data")

    plt.tight_layout()
    plt.show()
