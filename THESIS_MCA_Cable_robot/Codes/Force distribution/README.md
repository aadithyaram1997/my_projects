The codes use WiPy3 and they run specifically on python 3.9.19

force_dis_MCA.py generates the force distrbution on each of the 8 cables for every row(pose) from the csv file of the generated motion cues.

IRobot.getForceDistribution() is returned false, if the specific pose is not feasible on the cable robot. (Force is outside the limit set)

if False, a row of 0s are added for the specific pose and the poses at which its false is printed when the code is executed.

The force distribution on each cable is also plotted.