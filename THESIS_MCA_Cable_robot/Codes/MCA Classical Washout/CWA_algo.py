# Butterworth filter
def butter_filter(data, cutoff, fs, order=2, filter_type='low'):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype=filter_type, analog=False)
    filtered_data = filtfilt(b, a, data)
    return filtered_data

class ClassicalWashout:
    def __init__(self, fs, hp_acc_x, hp_acc_y, hp_acc_z, lp_acc_x, lp_acc_y, hp_roll, hp_pitch, hp_yaw, washout_damping, scale_acc, scale_rot, order=2):
        self.fs = fs
        self.hp_acc_x = hp_acc_x
        self.hp_acc_y = hp_acc_y
        self.hp_acc_z = hp_acc_z
        self.lp_acc_x = lp_acc_x
        self.lp_acc_y = lp_acc_y
        self.hp_roll  = hp_roll
        self.hp_pitch = hp_pitch
        self.hp_yaw   = hp_yaw
        self.washout_damping = washout_damping
        self.scale_acc = scale_acc  # Scale factor for acceleration
        self.scale_rot = scale_rot  # Scale factor for rotational rates
        self.order = order

    def process(self, accel_data, rotational_data):
        # Step 1: Apply scale factors to accelerations
        scaled_accel_x = accel_data['x'] * self.scale_acc
        scaled_accel_y = accel_data['y'] * self.scale_acc
        scaled_accel_z = accel_data['z'] * self.scale_acc

        # Step 2: First-order high-pass filter for accelerations
        first_order_hp_x = butter_filter(scaled_accel_x, self.hp_acc_x, self.fs, order=1, filter_type='high')
        first_order_hp_y = butter_filter(scaled_accel_y, self.hp_acc_y, self.fs, order=1, filter_type='high')
        first_order_hp_z = butter_filter(scaled_accel_z, self.hp_acc_z, self.fs, order=1, filter_type='high')

        # Step 3: Second-order high-pass filter (washout process) for accelerations
        highpass_accel_x = butter_filter(first_order_hp_x, self.hp_acc_x, self.fs, order=self.order, filter_type='high')
        highpass_accel_y = butter_filter(first_order_hp_y, self.hp_acc_y, self.fs, order=self.order, filter_type='high')
        highpass_accel_z = butter_filter(first_order_hp_z, self.hp_acc_z, self.fs, order=self.order, filter_type='high')

        # Step 4: Apply washout damping factor (after second-order filter for accelerations)
        damped_accel_x = highpass_accel_x * np.exp(-self.washout_damping * np.arange(len(highpass_accel_x)) / self.fs)
        damped_accel_y = highpass_accel_y * np.exp(-self.washout_damping * np.arange(len(highpass_accel_y)) / self.fs)
        damped_accel_z = highpass_accel_z * np.exp(-self.washout_damping * np.arange(len(highpass_accel_z)) / self.fs)

        # Step 5: Low-pass filter for accelerations (for tilt coordination)
        lowpass_accel_x = butter_filter(scaled_accel_x, self.lp_acc_x, self.fs, self.order, 'low')
        lowpass_accel_y = butter_filter(scaled_accel_y, self.lp_acc_y, self.fs, self.order, 'low')

        # Calculate tilt coordination angles
        tilt_coordination_roll = lowpass_accel_y / G  # Roll from y-acceleration
        tilt_coordination_pitch = lowpass_accel_x / G  # Pitch from x-acceleration

        # Step 6: Apply scale factor to rotational rates
        scaled_roll = rotational_data['roll'] * self.scale_rot
        scaled_pitch = rotational_data['pitch'] * self.scale_rot
        scaled_yaw = rotational_data['yaw'] * self.scale_rot

        # Step 7: First-order high-pass filter for rotational rates
        first_order_hp_roll = butter_filter(scaled_roll, self.hp_roll, self.fs, order=1, filter_type='high')
        first_order_hp_pitch = butter_filter(scaled_pitch, self.hp_pitch, self.fs, order=1, filter_type='high')
        first_order_hp_yaw = butter_filter(scaled_yaw, self.hp_yaw, self.fs, order=1, filter_type='high')

        # Step 8: Apply second-order high-pass filter to rotational rates
        highpass_roll = butter_filter(first_order_hp_roll, self.hp_roll, self.fs, order=self.order, filter_type='high')
        highpass_pitch = butter_filter(first_order_hp_pitch, self.hp_pitch, self.fs, order=self.order, filter_type='high')
        highpass_yaw = butter_filter(first_order_hp_yaw, self.hp_yaw, self.fs, order=self.order, filter_type='high')

        # Step 9: Apply washout damping factor (after second-order filter for rotational rates)
        damped_roll = highpass_roll * np.exp(-self.washout_damping * np.arange(len(highpass_roll)) / self.fs)
        damped_pitch = highpass_pitch * np.exp(-self.washout_damping * np.arange(len(highpass_pitch)) / self.fs)
        damped_yaw = highpass_yaw * np.exp(-self.washout_damping * np.arange(len(highpass_yaw)) / self.fs)

        # Step 10: Integrate accelerations to obtain velocities and then positions (after damping)
        vel_x = it.cumulative_trapezoid(damped_accel_x, dx=1 / self.fs, initial=0)
        vel_y = it.cumulative_trapezoid(damped_accel_y, dx=1 / self.fs, initial=0)
        vel_z = it.cumulative_trapezoid(damped_accel_z, dx=1 / self.fs, initial=0)

        pos_x = it.cumulative_trapezoid(vel_x, dx=1 / self.fs, initial=0)
        pos_y = it.cumulative_trapezoid(vel_y, dx=1 / self.fs, initial=0)
        pos_z = it.cumulative_trapezoid(vel_z, dx=1 / self.fs, initial=0)

        # Step 11: Integrate rotational rates to obtain angles (after damping)
        roll_angle = it.cumulative_trapezoid(damped_roll, dx=1 / self.fs, initial=0)
        pitch_angle = it.cumulative_trapezoid(damped_pitch, dx=1 / self.fs, initial=0)
        yaw_angle = it.cumulative_trapezoid(damped_yaw, dx=1 / self.fs, initial=0)

        # Combine with tilt coordination angles
        final_roll_angle = roll_angle + tilt_coordination_roll
        final_pitch_angle = pitch_angle + tilt_coordination_pitch

        return {
            'raw_pos_x': pos_x,
            'raw_pos_y': pos_y,
            'raw_pos_z': pos_z,
            'highpass_accel_x': highpass_accel_x,
            'highpass_accel_y': highpass_accel_y,
            'highpass_accel_z': highpass_accel_z,
            'lowpass_accel_x': lowpass_accel_x,
            'lowpass_accel_y': lowpass_accel_y,
            'highpass_roll': highpass_roll,
            'highpass_pitch': highpass_pitch,
            'highpass_yaw': highpass_yaw,
            'damped_roll': damped_roll,
            'damped_pitch': damped_pitch,
            'damped_yaw': damped_yaw,
            'x': pos_x,
            'y': pos_y,
            'z': pos_z,
            'roll': final_roll_angle,
            'pitch': final_pitch_angle,
            'yaw': yaw_angle
        }

# Main processes
if __name__ == "__main__":
    # Load the CSV file
    csv_file = "C:/Users/mrf-ar/AppData/Roaming/flightgear.org/Export/log_parameters_mrf_non_crash.csv"  # Replace with your file path
    data = pd.read_csv(csv_file)

    # Convert acceleration data from feet per second squared to meters per second squared
    accel_data = {
        'x': data['Acceleration_X'].values * FEET_TO_METERS,
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
    lp_acc_x = 0.1  # Low-pass filter cutoff frequency for tilt coordination (Hz)
    lp_acc_y = 0.1
    hp_roll = 0.01  # High-pass filter cutoff frequency for rotations (Hz)
    hp_pitch = 0.01
    hp_yaw = 0.1
    washout_damping = 0.1  # Damping factor for washout
    scale_acc = 0.5  # Scale factor for accelerations
    scale_rot = 1.0  # Scale factor for rotational rates

    washout_algorithm = ClassicalWashout(fs, hp_acc_x, hp_acc_y, hp_acc_z, lp_acc_x, lp_acc_y, hp_roll, hp_pitch, hp_yaw, washout_damping, scale_acc, scale_rot)

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

    output_df.to_csv('processed_motion.csv', index=False)
