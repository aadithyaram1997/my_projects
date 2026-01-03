import WiPy3

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
    Iws.setWrench(0, 0, -1000, 0, 0, 0) #ForceZ just to test

def calcWorkspaceforce(Iws):
    Iws.setMethod(4) #quadratic programming (4,0)
    Iws.setWorkspaceCriterion(0)
    Iws.setIterations(6)
    Iws.calculateWorkspace()
    #Iws.intersectWorkspace()  # need to use Iws.intersectWorkspace() when combining with other workspace calculations e.g. bounding box otherwise Iws.calculateWorkspace() is enough

def ForceDis(Irobot):
 
    print (Irobot.getForceDistribution(0, 0, 0, 0, 0, 0,))

def main():
    from WiPy3 import (
        Irobot,
        Ikin,
        Iws,
    )# This runs on python 3.9.19 64-bit, it might run on different versions of python 3.9 64-bit4
    manualmain(Irobot, Ikin, Iws)

def manualmain(Irobot, Ikin, Iws):
    setGeometry(Irobot, Ikin, Iws)
    calcWorkspaceforce(Iws)
    ForceDis(Irobot)



if __name__ == "__main__":
    #import os
    #import sys
    #import numpy as np
    main()



