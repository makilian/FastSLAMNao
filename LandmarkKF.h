/* LandmarkKF is an implementation of an extended kalman filter,
 which is used to track the location of an individual beacon */
#ifndef _LANDMARKKF_H
#define _LANDMARKKF_H

#include <math/Geometry.h>
#include <common/WorldObject.h>
#include <math/Pose2D.h>
#include <common/NMatrix.h>


class LandmarkKF
{
	public :
	//members
	NMatrix mean; // state (x, y) of landmark (2x1)
	NMatrix covariance; //covariance of landmarks coordinates (2x2)	
	
	NMatrix measurementPrediction;
	NMatrix measureCovar;
	NMatrix measureNoise;

	NMatrix jacobian;
	NMatrix kGain;
	
	//constructors
	LandmarkKF();
	
	//functions
	NMatrix predictMeasurement(Point2D robotPose, AngRad robotAngle);
	NMatrix predictLandmarkCoords(NMatrix measurement, Point2D robotPose, AngRad robotAngle);
	NMatrix calculateJacobian(Point2D robotPose); 

};

#endif







