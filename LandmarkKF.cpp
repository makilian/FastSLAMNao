/* LandmarkKF is an implementation of an extended kalman filter,
 which is used to track the location of an individual beacon */

#include <stdlib.h>
#include "LandmarkKF.h"

/* Landmark Constructor */
LandmarkKF::LandmarkKF()
{
	mean = NMatrix(2, 1);
	covariance = NMatrix(2, 2); 	
	measurementPrediction = NMatrix(2, 1);
	measureCovar = NMatrix(2, 2);
	measureNoise = NMatrix(2, 1);
	jacobian = NMatrix(2, 2);
	kGain = NMatrix(2, 2);
}


/* Given the robots pose, and the landmarks mean, predict the measurement the robot will get */
NMatrix LandmarkKF::predictMeasurement(Point2D robotPose, AngRad robotAngle)
{
	NMatrix predictedMeasure = NMatrix(2, 1);
		
	//predicted distance
	predictedMeasure[0][0] = sqrt(pow((mean[0][0] - robotPose.x), 2) + pow((mean[1][0] - robotPose.y), 2));
	
	//predictred angle
	predictedMeasure[1][0] = atan((mean[1][0] - robotPose.y)/(mean[0][0] - robotPose.x)) - robotAngle;	

	return predictedMeasure;
}

/* Given the robots pose, and the robot measurement, predict the coordinates of the landmark */
NMatrix LandmarkKF::predictLandmarkCoords(NMatrix measurement, Point2D robotPose, AngRad robotAngle)
{
	NMatrix tempMean = NMatrix(2, 1);
	//landmark x coord
	tempMean[0][0] = cos(robotAngle + measurement[1][0]) * measurement[0][0] + robotPose.x;

	//landmark y coord
	tempMean[1][0] = sin(robotAngle + measurement[1][0]) * measurement[0][0] + robotPose.y;	

	return tempMean;
}

/* This is the jacobian used in the Extended Kalman Filter update */
NMatrix LandmarkKF::calculateJacobian(Point2D robotPose)
{
	NMatrix tempjacobian = NMatrix(2,2);
	float q = pow((mean[0][0] - robotPose.x), 2) + pow((mean[1][0] - robotPose.y), 2);	

	tempjacobian[0][0] = (mean[0][0] - robotPose.x)/sqrt(q);
	tempjacobian[1][0] = (mean[1][0] - robotPose.y)/q;
	tempjacobian[0][1] = (mean[1][0] - robotPose.y)/sqrt(q);
	tempjacobian[1][1] = (mean[0][0] - robotPose.x)/q;

	return tempjacobian;
}
