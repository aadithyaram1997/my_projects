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
    structure_matrix = np.array(Ikin.getStructureMatrixMatrix())
    #force_distribution = np.array(Irobot.getForceDistribution(0, 0, 0, 0, 0, 0))


    #Ikin.getStructureMatrixMatrix()
    #print(Ikin.getStructureMatrixMatrix())
    force_distribution = np.array(Irobot.getForceDistribution(0.011470145051149837,-0.23399056654775222, 0.24935519381237592, -0.08896530028038255,  2.009102325938577,-5.667549724976974))

    print(force_distribution)
    #print(force_distribution)
    #print(structure_matrix)
    #print(Iws.getWrench(291.46533815,  326.84778612 , 343.3262147,   292.52226424, -193.50178718,-190.45914693, -193.25479665, -186.98813449))
  
    # Transpose the structure matrix to 6x8 and multiply
    result = structure_matrix.T @ force_distribution
    print("Wrench values", result)

def calcWorkspaceforce(Iws):
    Iws.setMethod(4) #quadratic programming (4,0)
    Iws.setWorkspaceCriterion(0)
    Iws.setIterations(6)
    Iws.calculateWorkspace()
    #Iws.intersectWorkspace()  # need to use Iws.intersectWorkspace() when combining with other workspace calculations e.g. bounding box otherwise Iws.calculateWorkspace() is enough

# def ForceDis(Irobot, x, y, z, roll, pitch, yaw):
#     force_distribution = Irobot.getForceDistribution(x, y, z, roll, pitch, yaw)
#     return force_distribution


def main():
    from WiPy3 import  Irobot,  Ikin,   Iws
    # This runs on python 3.9.19 64-bit, it might run on different versions of python 3.9 64-bit4
    manualmain(Irobot, Ikin, Iws)
    #input_file = 'C:\\Users\\mrf-ar\\motioncueing\\processed_motion_data.csv'  #csv file containing motion cues
    #output_file = 'C:\\Users\\mrf-ar\\motioncueing\\Force distribution\\force_distribution.csv'
def manualmain(Irobot, Ikin, Iws):
    setGeometry(Irobot, Ikin, Iws)
    calcWorkspaceforce(Iws)
    

                   
   # ForceDis(Irobot,x,y,z, roll, pitch, yaw)
#     data = []
#     with open(output_file, mode='r') as infile:
#         reader = csv.reader(infile)
#         for row in reader:
#             data.append(list(map(float, row)))

#     plot_force_distribution(data)



if __name__ == "__main__":
   # import os
   # import sys
  #  import numpy as np
    main()



# [[0.733431490161148, 0.5339189107434722, 0.420724192304771, 0.4498142056170613, -0.5829405520142092, -0.044363787258110576], 
#  [0.7300054609017915, -0.5357455130910653, 0.424345109853235, -0.45235659681619944, -0.5832832175203153, 0.0417853744304475],
#  [-0.7217134906893576, -0.5326873247180759, 0.44201114402431274, -0.4590150421158603, 0.5889981913714345, -0.03964940191182448], 
#  [-0.7125089825935877, 0.5427420699190176, 0.4447043908749616, 0.4650802133636108, 0.586058081357101, 0.029897284021493498], 
#  [0.8236052610772888, 0.5487673473829925, -0.14327865288327157, -0.37069538202283014, 0.4469108403550764, -0.41915937443679663], 
#  [0.819841790101239, -0.552020946088048, -0.15209311057261213, 0.378857375319035, 0.4482775790547351, 0.4151699918361105], 
#  [-0.8274377969058029, -0.5387213359846724, -0.15851187465474412, 0.3772799330141151, -0.45423441739049325, -0.4256457413685832],
#  [-0.8239376843537457, 0.5449563985992685, -0.15540017994681532, -0.37794272607182416, -0.45143415327065145, 0.4207802345483481]]