MCA_classical_washout.py reads the data accelerations x,y,z and rotations roll.pitch,yaw from csv file of logged paramters from flightgear and generates motion cues using classical washout algorithm
These motion cues are saved in a csv file and can be seen on a plot when the code is executed.

Paremeters for scaling down the initial value and the speed of the washout can be changed manually in the washout process of the code

Optionally, other parameters such as order of filters, high-pass and low-pass cut off frequencies, sampling rate can changed accordingly 

Example csv file is added (processed_motion_data.csv)