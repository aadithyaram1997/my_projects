import WiPy3
import csv
import matplotlib.pyplot as plt
import numpy as np
##

def setGeometry(Irobot, Ikin, Iws):    

    WiPy3.createRobot()
    Irobot.setMotionPattern(8, 5)
    
    '''For Ipanema 3'''
    Irobot.setBase(0,   7.6029,   5.5952,   3.5665)  
    Irobot.setBase(1,   7.5238,  -5.5789,   3.5745)
    Irobot.setBase(2,  -7.4614,  -5.5621,   3.7536)
    Irobot.setBase(3,  -7.3114,   5.6113,   3.7408)
    Irobot.setBase(4,   7.6126,   5.5812,  -0.7817)
    Irobot.setBase(5,   7.5427,  -5.5851,  -0.8525)
    Irobot.setBase(6,  -7.7641,  -5.5694,  -0.9384)
    Irobot.setBase(7,  -7.6959,   5.6008,  -0.9036)
    '''For CRS'''
    # Irobot.setBase(0,   6.94000,  6.42750,  3.42414)
    # Irobot.setBase(1,   6.94000, -6.72750,  3.42414)
    # Irobot.setBase(2,  -7.50000, -6.72750,  3.42414)
    # Irobot.setBase(3,  -7.50000,  6.42750,  3.42414)
    # Irobot.setBase(4,   6.94000,  6.42750, -3.10714)
    # Irobot.setBase(5,   6.94000, -6.72750, -3.10714)
    # Irobot.setBase(6,  -7.50000, -6.72750, -3.10714)
    # Irobot.setBase(7,  -7.50000,  6.42750, -3.10714)
    # set which part of the Base looks where (four have to look in the opposite direction)
    # for i in range(4):
    #     Irobot.setBaseOrientation(i, 0, 0, 0)
    # for i in range(4, 8):
    #     Irobot.setBaseOrientation(i, 180, 0, 0)

    '''For IPAnema 3'''
    Irobot.setPlatform(0,  0.5488,  0.46, -0.48)
    Irobot.setPlatform(1,  0.5488, -0.46, -0.48)
    Irobot.setPlatform(2, -0.5488, -0.46, -0.48)
    Irobot.setPlatform(3, -0.5488,  0.46, -0.48)
    Irobot.setPlatform(4,  0.36,  0.7488, 0.48)
    Irobot.setPlatform(5,  0.36, -0.7488, 0.48)
    Irobot.setPlatform(6, -0.36, -0.7488, 0.48)
    Irobot.setPlatform(7, -0.36,  0.7488, 0.48)
    
    '''For CRS'''
    # Irobot.setPlatform(0, 1.20845459,  0.87799281, -0.74699887)
    # Irobot.setPlatform(1, 1.20848055, -0.87803318, -0.74634473)
    # Irobot.setPlatform(2,-0.46170469, -1.42031757, -0.74688708)
    # Irobot.setPlatform(3,-0.46166721,  1.42059647, -0.74688049)
    # Irobot.setPlatform(4, 0.46164352,  1.42051451,  0.74678722)
    # Irobot.setPlatform(5, 0.46172807, -1.42079329,  0.74668035)
    # Irobot.setPlatform(6,-1.20841601, -0.87779025,  0.74679891)
    # Irobot.setPlatform(7,-1.20851883,  0.87783049,  0.74684468)

    Irobot.setPlatformCenterOfGravity(0, 0, 0)
    Irobot.setPlatformInertiaMatrix(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) #TODO: no inertia???
    Ikin.setKinematicsModel(0)  # "Set kinematics model [0: FIXED, 1: PULLEY]" set to 0 for testing
    Ikin.setElasticityModel(1)  # NONELASTIC, LIN_ELASTIC, SAGGING #TODO: what is the difference between those?
    Iws.setForceLimits(100, 3000) #set to 100 and 3000 for ipanema3

    #Irobot.setPlatformMass(weight = 300)
    #Iws.setWrench(0, 0, -(10 * weight), 0, 0, 0) #this is essentially the weigth calculation that affects the workspace calculation. 
    Iws.setWrench(0, 0, -1000, 0 , 0, 0) #ForceZ just to test
    Irobot.getForceDistribution(0, 0, 0, 0 , 0 ,0)

    Ikin.getStructureMatrixMatrix()

def calcWorkspaceforce(Iws):
    Iws.setMethod(4) #quadratic programming (4,0)
    Iws.setWorkspaceCriterion(0)
    Iws.setIterations(6)
    Iws.calculateWorkspace()
    #Iws.intersectWorkspace()  # need to use Iws.intersectWorkspace() when combining with other workspace calculations e.g. bounding box otherwise Iws.calculateWorkspace() is enough

def ForceDis(Irobot, x, y, z, roll, pitch, yaw):
    force_distribution = Irobot.getForceDistribution(x, y, z, roll, pitch, yaw)
    return force_distribution
    #print (Irobot.getForceDistribution(0, 0, 0, 0 , 0 ,0))

def calculate_wrench(Iws, Irobot, Ikin, force_distribution):
  #  Iws.setWrench(0, 0, -1000, 0 , 0, 0) #ForceZ just to test
    Irobot.getForceDistribution(0, 0, 0, 0 , 0 ,0)
   # Ikin.getStructureMatrixMatrix()
    # Get the structure matrix from Ikin
    structure_matrix = np.array(Ikin.getStructureMatrixMatrix())
    
    # Convert the force distribution to an array
    force_distribution_array = np.array(force_distribution)
    
    # Calculate wrench values by multiplying the transposed structure matrix with force distribution
    wrench_values = structure_matrix.T @ force_distribution_array
    
    return wrench_values

def plot_force_distribution(data):
    # Prepare the data
    rows = list(range(len(data)))  # X-axis (row indices)
    columns = list(zip(*data))  # Transpose to get columns

    # Create a figure with 8 subplots (2x4 grid)
    fig, axes = plt.subplots(2, 4, figsize=(20, 10), sharex=True, sharey=True)
    fig.suptitle('Force Distribution Across Rows')

    # Flatten the axes array for easy iteration
    axes = axes.flatten()

    # Plot each force distribution column in a separate subplot
    for i, (ax, column) in enumerate(zip(axes, columns)):
        ax.plot(rows, column)
        ax.set_title(f'Force Distribution {i + 1}')
        ax.set_xlabel('Row Index')
        ax.set_ylabel('Force Value')
        ax.grid(True)

    # Adjust layout to prevent overlap
    plt.tight_layout(rect=[0, 0, 1, 0.96])  # Adjust the top margin to make room for the suptitle
   # plt.savefig('force_distribution_subplot.png')  # Save the figure as a PNG file
    plt.show()  # Display the figure


def main():
    from WiPy3 import  Irobot,  Ikin,   Iws
    # This runs on python 3.9.19 64-bit, it might run on different versions of python 3.9 64-bit4
    #manualmain(Irobot, Ikin, Iws)
    input_file = 'C:\\Users\\mrf-ar\\motioncueing\\processed_motion_data.csv'  #csv file containing motion cues
    output_file = 'C:\\Users\\mrf-ar\\motioncueing\\Force distribution\\force_distribution.csv'
    wrench_output_file = 'C:\\Users\\mrf-ar\\motioncueing\\Force distribution\\wrench_values.csv'
 #def manualmain(Irobot, Ikin, Iws):
    setGeometry(Irobot, Ikin, Iws)
    calcWorkspaceforce(Iws)
    row_counter = 0  # Initialize row counter

    with open(input_file, mode='r') as infile, \
         open(output_file, mode='w', newline='') as force_outfile, \
         open(wrench_output_file, mode='w', newline='') as wrench_outfile:
        reader = csv.DictReader(infile)
        force_writer = csv.writer(force_outfile)
        wrench_writer = csv.writer(wrench_outfile)        # Write headers for both output files
        force_writer.writerow([f'Force Distribution {i+1}' for i in range(8)])
        wrench_writer.writerow([f'Wrench Value {i+1}' for i in range(6)])



        #writer.writerow(['Force Distribution Values'])
        #writer.writerow(['Wrench Values'])
        
        for row in reader:
               row_counter += 1
           # try:
               x = float(row['x_position'])
               y = float(row['y_position'])
               z = float(row['z_position'])
               roll = float(row['roll_angle'])
               pitch = float(row['pitch_angle'])
               yaw = float(row['yaw_angle'])


            # Calculate force distribution for the given pose
               force_distribution = ForceDis(Irobot, x, y, z, roll, pitch, yaw)

               wrench_values = calculate_wrench(Iws, Irobot, Ikin, force_distribution)

            #   Ikin.getStructureMatrixMatrix()
            #    structure_matrix = np.array(Ikin.getStructureMatrixMatrix())
            #    force_distribution_array = np.array(Irobot.getForceDistribution(x, y, z, roll, pitch, yaw))
            #    wrench_values = structure_matrix.T @ force_distribution_array


               if force_distribution:
                force_writer.writerow(force_distribution)
                
                # structure_matrix = np.array(Ikin.getStructureMatrixMatrix())
                # force_distribution_array = np.array(force_distribution)
                # wrench_values = structure_matrix.T @ force_distribution_array
                
                wrench_writer.writerow(wrench_values)
                if row_counter == 3000:
                    print("3000th Row - Pose:")
                    print(f"x: {x}, y: {y}, z: {z}, roll: {roll}, pitch: {pitch}, yaw: {yaw}")
                    print("3000th Row - Wrench Values:", wrench_values)
                    print(force_distribution)
            # Write all 8 force distribution values to the output CSV

               else:
                   force_writer.writerow([0] * 8)
                   wrench_writer.writerow([0] * 6) 
                   print(f"Row with x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw} returned all zeros.")  #prints the values of the rows where IRobot.getForceDistribution() returns false

            # except ValueError as e:
            #     # Handle cases where the conversion or calculation failed
            #     print(f"Error processing row: {row}. Error: {e}")
            #     writer.writerow([0] * 8)  # Write zeros if an error occurs
                   
   # ForceDis(Irobot)
    data = []
    with open(output_file, mode='r') as infile:
        reader = csv.reader(infile)
        next(reader)  # Skip the 'Force Distribution Values' header row
        for row in reader:
             data.append(list(map(float, row)))


    plot_force_distribution(data)



if __name__ == "__main__":
    #import os
    #import sys
    #import numpy as np
    main()



