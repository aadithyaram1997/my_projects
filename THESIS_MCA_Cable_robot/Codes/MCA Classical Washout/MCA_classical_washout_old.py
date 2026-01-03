import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt
import scipy.integrate as it
import matplotlib.pyplot as plt

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
    def __init__(self, fs, hp_acc_x, hp_acc_y, hp_acc_z, lp_acc_x, lp_acc_y, lp_acc_z, hp_roll, hp_pitch, hp_yaw, order=2):
        self.fs = fs
        self.hp_acc_x = hp_acc_x
        self.hp_acc_y = hp_acc_y
        self.hp_acc_z = hp_acc_z
        self.lp_acc_x = lp_acc_x
        self.lp_acc_y = lp_acc_y
        self.lp_acc_z = lp_acc_z
        self.hp_roll  = hp_roll
        self.hp_pitch = hp_pitch
        self.hp_yaw   = hp_yaw

        #self.lowpass_cutoff_acc = lowpass_cutoff_acc
        #self.highpass_cutoff_rot = highpass_cutoff_rot

        self.order = order

    def process(self, accel_data, rotational_data):
        # High-pass filter for accelerations
        highpass_accel_x = butter_filter(accel_data['x'], self.hp_acc_x, self.fs, self.order, 'high')
        highpass_accel_y = butter_filter(accel_data['y'], self.hp_acc_y, self.fs, self.order, 'high')
        highpass_accel_z = butter_filter(accel_data['z'], self.hp_acc_z, self.fs, self.order, 'high')


        #Low-pass filter for accelerations (for tilt coordination)
        lowpass_accel_x = butter_filter(accel_data['x'], self.lp_acc_x, self.fs, self.order, 'low')
        lowpass_accel_y = butter_filter(accel_data['y'], self.lp_acc_y, self.fs, self.order, 'low')
        lowpass_accel_z = butter_filter(accel_data['z'], self.lp_acc_z, self.fs, self.order, 'low')


        #high-pass filter for rotational rates 
        highpass_roll = butter_filter(rotational_data['roll'], self.hp_roll, self.fs, self.order, 'high')
        highpass_pitch = butter_filter(rotational_data['pitch'], self.hp_pitch, self.fs, self.order, 'high')
        highpass_yaw = butter_filter(rotational_data['yaw'], self.hp_yaw, self.fs, self.order, 'high')

        # Integrate accelerations to obtain velocities and then positions
        vel_x = it.cumulative_trapezoid(highpass_accel_x, dx=1 / self.fs, initial=0)
        vel_y = it.cumulative_trapezoid(highpass_accel_y, dx=1 / self.fs, initial=0)
        vel_z = it.cumulative_trapezoid(highpass_accel_z, dx=1 / self.fs, initial=0)

        pos_x = it.cumulative_trapezoid(vel_x, dx=1 / self.fs, initial=0)
        pos_y = it.cumulative_trapezoid(vel_y, dx=1 / self.fs, initial=0)
        pos_z = it.cumulative_trapezoid(vel_z, dx=1 / self.fs, initial=0)

        # Integrate rotational rates to obtain angles
        roll_angle = it.cumulative_trapezoid(highpass_roll, dx=1 / self.fs, initial=0)
        pitch_angle = it.cumulative_trapezoid(highpass_pitch, dx=1 / self.fs, initial=0)
        yaw_angle = it.cumulative_trapezoid(highpass_yaw, dx=1 / self.fs, initial=0)

        # Apply washout process to positions and angles
        #Start and stop of washout can be adjusted accordingly
        washout_x = pos_x * np.exp(-np.linspace(0, 0, len(pos_x)))
        washout_y = pos_y * np.exp(-np.linspace(0, 0, len(pos_y)))
        washout_z = pos_z * np.exp(-np.linspace(0, 0, len(pos_z)))

        washout_roll = roll_angle * np.exp(-np.linspace(0, 0, len(roll_angle)))
        washout_pitch = pitch_angle * np.exp(-np.linspace(0, 0, len(pitch_angle)))
        washout_yaw = yaw_angle * np.exp(-np.linspace(0, 0, len(yaw_angle)))


        return {
            'raw_pos_x': pos_x,
            'raw_pos_y': pos_y,
            'raw_pos_z': pos_z,
            'highpass_accel_x': highpass_accel_x,
            'highpass_accel_y': highpass_accel_y,
            'highpass_accel_z': highpass_accel_z,
            'lowpass_accel_x' : lowpass_accel_x,
            'lowpass_accel_y' : lowpass_accel_y,
            'lowpass_accel_z' : lowpass_accel_z,
            'highpass_roll': highpass_roll,
            'highpass_pitch': highpass_pitch,
            'highpass_yaw' :highpass_yaw,
            'x': washout_x,
            'y': washout_y,
            'z': washout_z,
            'roll': washout_roll,
            'pitch': washout_pitch,
            'yaw': washout_yaw,
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
    csv_file = "C:/Users/mrf-ar/AppData/Roaming/flightgear.org/Export/log_parameters_mrf_non_crash.csv"  # Replace with your file path
    data = pd.read_csv(csv_file)

    # Convert acceleration data from feet per second squared to meters per second squared
    accel_data = {
        'x': data['Acceleration_X'].values * FEET_TO_METERS ,
        'y': data['Acceleration_Y'].values * FEET_TO_METERS,
        'z': data['Acceleration_Z'].values * FEET_TO_METERS 
    }
    rotational_data = {
        'roll': data['Roll_rate'].values,
        'pitch': data['Pitch_rate'].values,
        'yaw': data['Yaw_rate'].values
    }

    # Initialize the washout algorithm
    fs = 10  # Sampling frequency (Hz)
    hp_acc_x = 0.1  # High-pass filter cutoff frequency (Hz)
    hp_acc_y = 0.1
    hp_acc_z = 0.1

    
    lp_acc_x = 0.1  # Low-pass filter cutoff frequency for acceleration used for tilt coordination (Hz)
    lp_acc_y = 0.1
    lp_acc_z = 0.1

    hp_roll = 0.01  #High-pass filter cutoff frequency for rotations (Hz)
    hp_pitch = 0.01
    hp_yaw = 0.1
   

    washout_algorithm = ClassicalWashout(fs, hp_acc_x, hp_acc_y, hp_acc_z, lp_acc_x, lp_acc_y, lp_acc_z, hp_roll, hp_pitch, hp_yaw)

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
    plt.plot(data['Time'], accel_data['x'], label="X Acceleration")
    #plt.plot(data['Time'], accel_data['y'], label="Y Acceleration")
    plt.plot(data['Time'], accel_data['z'], label="Z Acceleration")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.legend()
    plt.title("Raw Acceleration Data")

    # Rotational rates
    plt.subplot(522)
    plt.plot(data['Time'], rotational_data['roll'], label="Roll Rate")
    plt.plot(data['Time'], rotational_data['pitch'], label="Pitch Rate")
    plt.plot(data['Time'], rotational_data['yaw'], label="Yaw Rate")
    plt.xlabel("Time (s)")
    plt.ylabel("Rate (deg/s)")
    plt.legend()
    plt.title("Raw Rotational Rates")

    plt.subplot(523)
    plt.plot(data['Time'], processed_movement['highpass_accel_x'], label="X Acceleration (High-Pass)")
    plt.plot(data['Time'], processed_movement['highpass_accel_y'], label="Y Acceleration (High-Pass)")
    plt.plot(data['Time'], processed_movement['highpass_accel_z'], label="Z Acceleration (High-Pass)")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s²)")
    plt.legend()
    plt.title("High-Pass Filtered Accelerations")

    # plt.subplot(529)
    # plt.plot(data['Time'], processed_movement['vel_x'], label="x")
    # plt.plot(data['Time'], processed_movement['vel_y'], label="Y")
    # plt.plot(data['Time'], processed_movement['vel_z'], label="Z")
    # plt.xlabel("Time (s)")
    # plt.ylabel("vel (m/s)")
    # plt.legend()
    # plt.title("velocity")

    plt.subplot(528)
    #plt.plot(data['Time'], processed_movement['yaw_angle'], label="yaw_afterHP")
    plt.plot(data['Time'], processed_movement['roll_angle'], label="roll_afterHP")
    #plt.plot(data['Time'], processed_movement['pitch_angle'], label="pitch_afterHP")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.legend()
    plt.title("yaw after high pass(from rates)")

    plt.subplot(524)
    plt.plot(data['Time'], processed_movement['highpass_roll'], label="roll Rate (Low-Pass)")
    #plt.plot(data['Time'], processed_movement['highpass_pitch'], label="pitch Rate (Low-Pass)")
    #plt.plot(data['Time'], processed_movement['highpass_yaw'], label="yaw Rate (High-Pass)")
    plt.xlabel("Time (s)")
    plt.ylabel("Yaw Rate (deg/s)")
    plt.legend()
    plt.title("high-Pass Filtered Rotations")



    # Angles (integrated from rates)
    roll_angle = it.cumulative_trapezoid(rotational_data['roll'], dx=1 / fs, initial=0)
    pitch_angle = it.cumulative_trapezoid(rotational_data['pitch'], dx=1 / fs, initial=0)
    yaw_angle = it.cumulative_trapezoid(rotational_data['yaw'], dx=1 / fs, initial=0)

    plt.subplot(526)
    plt.plot(data['Time'], roll_angle, label="Roll Angle")
    #plt.plot(data['Time'], pitch_angle, label="Pitch Angle")
  #  plt.plot(data['Time'], yaw_angle, label="Yaw Angle")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.legend()
    plt.title("Integrated Angles (from rates)")

    plt.subplot(525)
    plt.plot(data['Time'], processed_movement['raw_pos_x'], label="X Position (Raw)")
    plt.plot(data['Time'], processed_movement['raw_pos_y'], label="Y Position (Raw)")
    plt.plot(data['Time'], processed_movement['raw_pos_z'], label="Z Position (Raw)")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.legend()
    plt.title("Raw Positions Before Washout")

    



   # Plot Generated motion cues for angles
    plt.subplot(529)
    plt.plot(data['Time'], processed_movement['roll'], label="Roll Angle")
   # plt.plot(data['Time'], processed_movement['pitch'], label="Pitch Angle")
   # plt.plot(data['Time'], processed_movement['yaw'], label="Yaw Angle")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.legend()
    plt.title("Processed Angle Data")

    # Plot Generated motion cues of position
    plt.subplot(527)
    plt.plot(data['Time'], processed_movement['x'], label="X Position")
    plt.plot(data['Time'], processed_movement['y'], label="Y Position")
    plt.plot(data['Time'], processed_movement['z'], label="Z Position")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.legend()
    plt.title("Processed Position Data")

    plt.tight_layout()
    plt.show()
