/* Particle is an implementation of a particle for the particle filter used by our
   FastSLAM localization code */

#ifndef _PARTICLE_H
#define _PARTICLE_H

#include <math/Geometry.h>
#include <common/WorldObject.h>
#include <math/Pose2D.h>

class LandmarkKF;

class Particle 
{
	public : 

	//data members	
	float M_prob;
	Point2D M_pos;
	AngRad M_ang;
	LandmarkKF *landmarks[6];
	bool seen[6];
	
	//for random num generation
	float v;
	int use_last;

	public:
	//constructors
	Particle();

	Particle( float prob,
		Point2D pos,
		AngRad ang,
	    	Rectangle boundary );
  
	//methods
	float getProbability();
	Point2D getPosition();
	AngRad getAngle();

	void setProbability( float prob );
	void degradeProbability( float factor );
	bool setPosition( Point2D pos, Rectangle boundary );
	void setAngle( AngRad ang );

	void setParticle( float prob, 
			Point2D pos, 
			AngRad ang,
		    	Rectangle boundary );


  	void moveRelative( Pose2D odom, Rectangle boundary );

  	void moveGlobal( Vector2D globalDisplacement,
		   	AngRad rotation, 
			Rectangle boundary );

  	void randomWalk( float maxDistance, 
			AngRad maxRotation,
		   	Rectangle boundary );

  	void placeRandomly( Rectangle boundary );
	void placeAtCenter(Rectangle boundary);

  	float getDistanceToPoint( Point2D p );
  	AngRad getBearingToPoint( Point2D p );

	//landmark stuff	
	LandmarkKF *getLandmarkForID(int id);
	bool hasSeen(int id);
	void clearSeenArray();
	void insertNewLandmark(int id, LandmarkKF *newLandmark); 
	float generateGaussianRandom(float stdDev);

  	friend ostream& operator << ( ostream &os, Particle p );
};


#endif
