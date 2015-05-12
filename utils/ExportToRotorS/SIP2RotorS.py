#	___SIPtoRotorS___
# 	StructuralInspectionPlanner Path Exporter to RotorS missions
#	
#	This script reads the output *.csv file of an inspection path generated by
# 	the StructuralInspectionPlanner Toolbox and exports a *.awm file that can
# 	be loaded at the Groud Control Station of the DJI Drones. Note that all constraints
#	of the platform or the GCS software must be respected by the user.
#   
#   More info on RotorS simulator:
#   https://github.com/ethz-asl/rotors_simulator.git
#
#

import math
import sys, getopt
import numpy as np
from math import pow, cos, sin, sqrt, degrees, radians, atan2, pi, ceil, floor
from scipy import cos, sin, arctan, sqrt, arctan2


# RotorS Mission Parameters
Speed = 0.25						# m/s
RotationalSpeed = 0.5		# rad/s
TimeStep = 0.1					# s, approximately

def main(argv):
	inputfile = ''
	outputfile = ''
	try:
		opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
	except getopt.GetoptError:
		print 'test.py -i <inputfile> -o <outputfile>'
		sys.exit(2)
	for opt, arg in opts:
		if opt == '-h':
			print 'test.py -i <inputfile> -o <outputfile>'
			sys.exit()
		elif opt in ("-i", "--ifile"):
			inputfile = arg
		elif opt in ("-o", "--ofile"):
			outputfile = arg
	
	rotors_mission_file = open(outputfile, "wb")
	PathENU = np.genfromtxt(inputfile, delimiter = ',')

	Nx,Ny = PathENU.shape
	if Nx < 2:
		print "Number of waypoints must at least be 2"
		sys.exit(2)
	time = np.zeros(Nx); time[0] = 0.0;
	distance = np.zeros(Nx-1);
	rotation = np.zeros(Nx-1);
	
	for i in range(0, Nx-1):
		distance[i] = sqrt((PathENU[i,0]-PathENU[i+1,0])*(PathENU[i,0]-PathENU[i+1,0]) + (PathENU[i,1]-PathENU[i+1,1])*(PathENU[i,1]-PathENU[i+1,1]) + (PathENU[i,2]-PathENU[i+1,2])*(PathENU[i,2]-PathENU[i+1,2]));
		rotation[i] = np.abs(PathENU[i,3] - PathENU[i+1,3]);
		if rotation[i] > pi:
			rotation[i] = 2.0*pi-rotation[i];
		time[i+1] = max(distance[i]/Speed, rotation[i]/RotationalSpeed);
	
	if TimeStep < time[1]:
		rotors_mission_file.write(str(TimeStep) + " ");
	else:
		rotors_mission_file.write(str(time[1]) + " ");
	rotors_mission_file.write(str(PathENU[0,0]) + " ");
	rotors_mission_file.write(str(PathENU[0,1]) + " ");
	rotors_mission_file.write(str(PathENU[0,2]) + " ");
	rotors_mission_file.write(str(PathENU[0,3]) + "\n");
	for i in range(0, Nx-1):
		numDisc = math.floor(time[i+1]/TimeStep);
		dt = time[i+1]/numDisc;
		for j in range(1, int(numDisc+1)):
			jfloat = float(j);
			dyaw = PathENU[i,3] - PathENU[i+1,3];
			if dyaw > pi:
				dyaw = dyaw - 2.0*pi;
			if dyaw < -pi:
				dyaw = dyaw + 2.0*pi;
			rotors_mission_file.write(str(dt) + " ");
			rotors_mission_file.write(str(PathENU[i+1,0]*jfloat/numDisc + PathENU[i,0]*(1-jfloat/numDisc)) + " ");
			rotors_mission_file.write(str(PathENU[i+1,1]*jfloat/numDisc + PathENU[i,1]*(1-jfloat/numDisc)) + " ");
			rotors_mission_file.write(str(PathENU[i+1,2]*jfloat/numDisc + PathENU[i,2]*(1-jfloat/numDisc)) + " ");
			rotors_mission_file.write(str(PathENU[i,3] - dyaw*jfloat/numDisc) + "\n");
	
	rotors_mission_file.close()
	
	print "### RotorS Mission file written :: please make sure you have respected all the platform mission constraints!"

if __name__ == "__main__":
    main(sys.argv[1:])