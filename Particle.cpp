#include<stdlib.h>
#include "Particle.h"
#include "LandmarkKF.h"

Particle::Particle() {

  M_prob = -1;
  for(int x = 0; x < 6; x++)
	seen[x] = false;

}

Particle::Particle( float prob,
		    Point2D pos,
		    AngRad ang,
		    Rectangle boundary ) {

  use_last = 0;
  v = 0;
  setParticle( prob, pos, ang, boundary );
  for(int x = 0; x < 6; x++)
	seen[x] = false;

}

float Particle::getProbability()
{
  return M_prob;
}

Point2D Particle::getPosition()
{
  return M_pos;
}

AngRad Particle::getAngle()
{
  return M_ang;
}


void Particle::setProbability( float prob )
{

  M_prob = prob;

}



void Particle::degradeProbability( float factor )
{
  setProbability( M_prob * factor );
}

bool Particle::setPosition( Point2D pos, Rectangle boundary )
{
  if ( boundary.isInside( pos ) ) {
    M_pos = pos;
    return true;
  }
  return false;
}


void Particle::setAngle( AngRad ang )
{
  M_ang = normalizeAngle( ang );
}


void Particle::setParticle( float prob, Point2D pos, AngRad ang,
			    Rectangle boundary )
{
  setProbability( prob );
  if ( !setPosition( pos, boundary ) ){
    cout << "setParticle position error, tried to set " << pos << endl;
    M_pos = boundary.getPosOutside();
  }
  setAngle( ang );
}


/* This function was changed to make the odometry update probabilistic 
   The values a1, a2, a3, a4 have been tuned to make the update be a gaussian
   with a 'crescent moon shape'.  The idea for this was in chapter 5 of the textbook
  Probabilistic Robotics */
void Particle::moveRelative(Pose2D disp, Rectangle boundary)
{
	float xDisp = disp.translation.x;
	float yDisp = disp.translation.y;
	float rotDisp = disp.rotation;

	//deterministic stuff
	float deltaRot1 = atan2(yDisp, xDisp);
	float deltaRot2 = rotDisp - deltaRot1;
	float deltaTrans = sqrt(pow(xDisp, 2) + pow(yDisp, 2));

	// TUNE THESE 
	float a1 = .05;
	float a2 = .3;
	float a3 = .01;
	float a4 = .0001;

	//standard devs for error generation
	float standDevRot1 = a1*abs(deltaRot1) + a2*deltaTrans;
	float standDevTrans = a3*deltaTrans + a4*(abs(deltaRot1) + abs(deltaRot2));
	float standDevRot2 = a1*abs(deltaRot2) + a2*deltaTrans;

	//error - (the crescent moon shaped gaussian)
	float errorRot1 = generateGaussianRandom(pow(standDevRot1, 2)); 
	float errorRot2 = generateGaussianRandom(pow(standDevRot2, 2));
	float errorTrans = generateGaussianRandom(pow(standDevTrans, 2));

	//added in error
	float deltaRot1Hat = deltaRot1 + errorRot1;
	float deltaRot2Hat = deltaRot2 + errorRot2;
	float deltaTransHat = deltaTrans + errorTrans;

	//the probabilistic x, y, and theta deltas
	float newX = cos(deltaRot1Hat)*deltaTransHat;
	float newY = sin(deltaRot1Hat)*deltaTransHat;
	AngRad angleChange = deltaRot1Hat + deltaRot2Hat;

	//the new coordinates
	Point2D newPos = Point2D(M_pos.x + newX, M_pos.y + newY);
	AngRad newAng = M_ang + angleChange;

	//update
	setPosition(newPos, boundary);
	setAngle(newAng);
} 

//generate a random point for a gaussian zero mean random variable with the given stddev
float Particle::generateGaussianRandom(float stdDev)
{
	float temp = 0;

	for(int i = 0; i < 12; i++){
		temp += ((2.0 * (float(rand()) / RAND_MAX) - 1.0) * sqrt(stdDev));
	}

	return temp / 2.0;
}

void Particle::moveGlobal( Vector2D globalDisplacement,
			   AngRad rotation, Rectangle boundary )
{
  setPosition( M_pos + globalDisplacement, boundary );
  setAngle( M_ang + rotation );
}


void Particle::randomWalk( float maxDistance, AngRad maxRotation,
                           Rectangle boundary )
{
  Vector2D dPos( maxDistance * ( 1 - M_prob ) * (2 * drand48() - 1),
		 maxDistance * ( 1 - M_prob ) * (2 * drand48() - 1) );
  AngRad dAng =  maxRotation * ( 1 - M_prob ) * (2 * drand48() - 1);
  
  moveGlobal( dPos, dAng, boundary );
}

void Particle::placeRandomly( Rectangle boundary )
{
  Vector2D dPos( boundary.getWidth() * drand48(),
		 boundary.getLength() * drand48() );
  setPosition( boundary.getBottomLeft() + dPos,
	       boundary );
  setAngle( TWOPI * drand48() - M_PI );
}

/* Places the particle at the center of the map, useful for FastSLAM initialization */
void Particle::placeAtCenter(Rectangle boundary)
{
	Point2D dPos = Point2D(0, 0);
	setPosition(dPos, boundary);
	setAngle(0);
}


float Particle::getDistanceToPoint( Point2D p )
{
  return M_pos.getDistanceTo( p );
}

AngRad Particle::getBearingToPoint( Point2D p )
{
  return M_pos.getBearingTo(p, M_ang);
}



/*! Overloaded version of the C++ output operator for output. 
  \param os output stream to which information should be written
  \param p a Particle which must be printed
  \return output stream containing (x,y) */
ostream& operator << ( ostream &os, Particle p )
{
  return ( os << "( " << p.M_pos << ", " << Rad2Deg(p.M_ang) << ", " << p.M_prob << " )" );
}


/* Helper function to access landmark array */
LandmarkKF *Particle::getLandmarkForID(int id)
{
	return landmarks[id];
}

/* helper function to clear the beacons you've seen */
void Particle::clearSeenArray()
{
	for(int x = 0; x < 6; x++)
		seen[x] = false;	
}

/* helper function so SLAM can ask if it's seen the beacon before */
bool Particle::hasSeen(int id)
{
	return seen[id];
}

/* when SLAM sees a new beacon, uses this method to insert it into the array */
void Particle::insertNewLandmark(int id, LandmarkKF *newLandmark)
{
	seen[id] = true;
	landmarks[id] = newLandmark;
}
