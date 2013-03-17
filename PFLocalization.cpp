#include "PFLocalization.h"
#include "LandmarkKF.h"
#include <iostream>

PFLocalization::PFLocalization(){
  printf("Creating Particle Filter Localization system\n");

  m_boundary = FIELD;

  measurementNoise = NMatrix(2, 2);
  measurementNoise[0][0] = 25000; //distance std. dev
  measurementNoise[1][1] = 1.85; //bearing std. sev

  partParams.NUM_PARTICLES = 500;

  partParams.RESAMPLE_FREQ = 1; //20; // --5 --15;
  partParams.DELTA_DIST = 150; // --75;
  partParams.DELTA_ANG = DEG_T_RAD * 30; // --15;
  partParams.DEGRADE_FACTOR = 0.99; // --0.99

  partParams.RESET_PROB_EVERY = true; //

  // flags
  partParams.LOC_DEBUG = false;

  partParams.USE_BEACONS = true; //

  // sigma's
  partParams.BEACON_DIST_SIGMA = 12500; //
  partParams.BEACON_BEAR_SIGMA = 50; // // 10; //500;

  partParams.USE_BEARINGS = true;
  partParams.USE_DISTANCES = true;

  partParams.MAX_BEACON_DIST = 8000;

  // num particles cannot be greater than maxparticles
  if (partParams.NUM_PARTICLES > MAX_PARTICLES) {
    cout << "ERROR: NUM_PARTICLES(" << partParams.NUM_PARTICLES
         << ") greater than MAX_PARTICLES(" << MAX_PARTICLES << ")" << endl;
    partParams.NUM_PARTICLES = MAX_PARTICLES;
  }

  // init temp particles
  tmpParticles = new Particle[MAX_PARTICLES];
  //cout << "PFLocalization done" << endl;
}

void PFLocalization::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("team_packets");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("vision_odometry");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("vision_joint_angles");
  requiresMemoryBlock("behavior");
  requiresMemoryBlock("vision_processed_sonar");
}

void PFLocalization::specifyMemoryBlocks() {
  getOrAddMemoryBlock(worldObjects,"world_objects");
  getOrAddMemoryBlock(localizationMem,"localization");
  getOrAddMemoryBlock(teamPacketsMem,"team_packets");
  getOrAddMemoryBlock(frameInfo,"vision_frame_info");
  getOrAddMemoryBlock(odometry,"vision_odometry");
  getOrAddMemoryBlock(robotState,"robot_state");
  getOrAddMemoryBlock(gameState,"game_state");
  getOrAddMemoryBlock(jointAngles,"vision_joint_angles");
  getOrAddMemoryBlock(behaviorMem,"behavior");
  getOrAddMemoryBlock(processedSonar,"vision_processed_sonar");
}


PFLocalization::~PFLocalization() {
  delete [] tmpParticles;
}

void PFLocalization::initSpecificModule() {

  reInit();

}

void PFLocalization::reInit(){

  cout << "Creating Particle Filter Localization system" << endl;
  m_boundary = FIELD;
  
  measurementNoise = NMatrix(2, 2);

  /* TUNING PARAMETERS */
  measurementNoise[0][0] = 25000; 
  measurementNoise[1][1] = 1.85; 
  partParams.NUM_PARTICLES = 500; 
  partParams.RESAMPLE_FREQ = 1; 
  /* ~~~~~~~~~~~~~~~~ */

  partParams.DELTA_DIST = 150; // --75;
  partParams.DELTA_ANG = DEG_T_RAD * 30; // --15;
  partParams.DEGRADE_FACTOR = 0.99; // --0.99

  partParams.RESET_PROB_EVERY = true; //

  // flags
  partParams.LOC_DEBUG = false;

  partParams.USE_BEACONS = true; //

  // sigma's
  partParams.BEACON_DIST_SIGMA = 12500; //
  partParams.BEACON_BEAR_SIGMA = 50; // // 10; //500;

  partParams.USE_BEARINGS = true;
  partParams.USE_DISTANCES = true;

  partParams.MAX_BEACON_DIST = 8000;

  // num particles cannot be greater than maxparticles
  if (partParams.NUM_PARTICLES > MAX_PARTICLES) {
    cout << "ERROR: NUM_PARTICLES(" << partParams.NUM_PARTICLES
         << ") greater than MAX_PARTICLES(" << MAX_PARTICLES << ")" << endl;
    partParams.NUM_PARTICLES = MAX_PARTICLES;
  }

  // init temp particles
  tmpParticles = new Particle[MAX_PARTICLES];

  // init particles
  resetParticles();

}




void PFLocalization::processFrame() {
  //std::cout << "HI" << std::endl;
  //cout << "PFLocalization" << endl;
  if (!isMemorySatisfied()) return;

  timePassed = frameInfo->seconds_since_start - timeLast;
  timeLast = frameInfo->seconds_since_start;

  // update particles from odometry
  updateParticlesFromOdometry();

  // update particles from observations
  updateParticlesFromObservations();

  // possibly resample
  if ((frameInfo->frame_id+1) % partParams.RESAMPLE_FREQ == 0){
    resampleParticles();
  }

  //recalculate map
  estimateBeaconLocations();

  // update pose
  // do this before the random walk adds some noise
  estimateRobotPose();

  // random walk
//  randomWalkParticles();

  // update relative object locations
  estimateRelativeObjectLocations();
  // print particles (for debug)
  //if (frameInfo->frame_id  == 0)

#ifdef ALLOW_LOC_DEBUG
  if (partParams.LOC_DEBUG)
    printParticles();
#endif

}

/** Set particle probabilities to one and place AT CENTER */
void PFLocalization::resetParticles()
{
	for (int i = 0; i < partParams.NUM_PARTICLES; i++)
  	{
  		Particle *part = &(localizationMem->particles[i]);
    		part->setProbability(1);
    		part->placeAtCenter(m_boundary);
		part->clearSeenArray();
  	}
}


void PFLocalization::setParticleProbabilities(float newProb){

  for (int i = 0; i < partParams.NUM_PARTICLES; i++){
    Particle *part = &(localizationMem->particles[i]);
    part->setProbability(newProb);
  }
}


/* PROBABILISTIC odometry update */
void PFLocalization::updateParticlesFromOdometry()
{
	// get odometry update from somewhere in snapshot
  	Pose2D disp = odometry->displacement;

  	if (disp.translation.x != 0 || disp.translation.y != 0 || disp.rotation != 0)
  	{

    		pfLog((40, "odom! = (%5.0f, %5.0f), a: %5.0f", disp.translation.x, disp.translation.y, disp.rotation));
    		for (int i = 0; i < partParams.NUM_PARTICLES; i++)
		{ 	
			Particle *part = &(localizationMem->particles[i]);
			part->moveRelative(disp, m_boundary);
    		}
  	}
}

void PFLocalization::updateParticlesFromObservations(){
  for (int i = 0; i < partParams.NUM_PARTICLES; i++){

    pfLog((40, "Update Particle %d", i));
    pfLog((40, "((%5.0f, %5.0f), %5.0f, %5.5f)",
           localizationMem->particles[i].M_pos.x,
           localizationMem->particles[i].M_pos.y,
           Rad2Deg(localizationMem->particles[i].M_ang),
           localizationMem->particles[i].M_prob));

    updateParticleFromObservations( &(localizationMem->particles[i]));
  }
}

// update particle probabilities from observations
void PFLocalization::updateParticleFromObservations(Particle *part)
{
	float newProb = 1.0;
	WorldObject *objects = &(worldObjects->objects_[0]);

//	cout << "Particle pos: " << part->getPosition() << endl; 
//	cout << "Particle ang: " << part->getAngle() << endl; 

	for(int i = BEACON_OFFSET; i < BEACON_OFFSET + NUM_BEACONS; i++)
	{
		if (objects[i].seen)
		{
			beaconsSeen[i - BEACON_OFFSET] = true;
			localizationMem->beaconsSeen[i - BEACON_OFFSET] = true;
			NMatrix actualMeasurement = NMatrix(2, 1);
			actualMeasurement[0][0] = objects[i].visionDistance;
			actualMeasurement[1][0] = objects[i].visionBearing;
			
	
			if(part->hasSeen(i - BEACON_OFFSET)) //we've seen this beacon before, so we need to update our estimate of its location
			{
				//landmark position updates from new obsevations	
				LandmarkKF *landmark = part->getLandmarkForID(i - BEACON_OFFSET);
	
				landmark->measurementPrediction = landmark->predictMeasurement(part->getPosition(), part->getAngle());  //predict measurement is obscenely large.  it thinks the landmark is really far?
		
				landmark->jacobian = landmark->calculateJacobian(part->getPosition());
		
				landmark->measureCovar = landmark->jacobian * landmark->covariance * (landmark->jacobian).transp() + measurementNoise;		
	
				landmark->kGain = landmark->covariance * (landmark->jacobian).transp() * Invert22(landmark->measureCovar);
				landmark->mean = landmark->mean + landmark->kGain * (actualMeasurement - landmark->measurementPrediction);
			
				NMatrix identityMatrix = NMatrix(2, 2, true);	
		
				landmark->covariance = (identityMatrix - landmark->kGain * landmark->jacobian) * landmark->covariance;
				
				//now that we've updated the landmark position, we can assign the importance weight (this is probably similar to our old way of doing it)
				newProb *= calculateImportance(part, landmark, actualMeasurement);
			}
			else 
			{
				//first time we've seen this beacon, need to add it to the map
				LandmarkKF *newLandmark = new LandmarkKF();
				newLandmark->mean = newLandmark->predictLandmarkCoords(actualMeasurement, part->getPosition(), part->getAngle());
				newLandmark->jacobian = newLandmark->calculateJacobian(part->getPosition());
				newLandmark->covariance = Invert22(newLandmark->jacobian) * measurementNoise * (Invert22(newLandmark->jacobian)).transp();			
	
				//insert the new landmark into the particles map
				part->insertNewLandmark(i - BEACON_OFFSET, newLandmark);
				
				//update its probability  
				newProb*=1;
			}
    		}
  	} 

	part->setProbability(newProb);
}

/* FastSLAM importance weight calculation */
float PFLocalization::calculateImportance(Particle *part, LandmarkKF *landmark, NMatrix measurement)
{
	float determinant = (landmark->measureCovar)[0][0] * (landmark->measureCovar)[1][1] - (landmark->measureCovar)[0][1] * (landmark->measureCovar)[1][0];	

	float coeff = 1.0/sqrt(abs(2.0*M_PI*determinant));

	float expo = exp(-1.0/2.0 * (((measurement - landmark->measurementPrediction).transp()) * Invert22(landmark->measureCovar) * (measurement - landmark->measurementPrediction))[0][0]);

	return coeff*expo;
}


/** Resample particles */
void PFLocalization::resampleParticles()
{
  
  float sum = 0;
  for(int i = 0; i < partParams.NUM_PARTICLES; i++)
  {
  	sum += localizationMem->particles[i].M_prob;
  }
  
  for(int a = 0; a < partParams.NUM_PARTICLES; a ++)
  {
  	float threshold = ((rand() * 1.0) / RAND_MAX) * sum;
  	float cumSum = 0;
  	for(int i = 0; i < partParams.NUM_PARTICLES; i++)
  	{
  		cumSum += localizationMem->particles[i].M_prob;
		if(cumSum >= threshold)
		{
			tmpParticles[a] = localizationMem->particles[i];
			break;						
		}
	}
  }

  memcpy(localizationMem->particles, tmpParticles, partParams.NUM_PARTICLES * sizeof(Particle));
  setParticleProbabilities(1.0);
}


/** Perform a random walk on the particles */
void PFLocalization::randomWalkParticles()
{
  // loop through half, moving each pair of particles opposite directions

  for ( int i = 0; i < partParams.NUM_PARTICLES/2; i++ ) {
    Particle *part1 = &(localizationMem->particles[i]);
    Particle *part2 = &(localizationMem->particles[i+partParams.NUM_PARTICLES/2]);

    Vector2D dPos( partParams.DELTA_DIST * (2.0 * drand48() - 1),
                   partParams.DELTA_DIST * (2.0 * drand48() - 1) );
    AngRad dAng =  partParams.DELTA_ANG * (2.0 * drand48() - 1);

    // move them in opposite directions on this vector, based on their prob
    float p1Ratio = 1.0 - part1->M_prob;
    float p2Ratio = 1.0 - part1->M_prob;

    float p1AngleRatio = p1Ratio;
    float p2AngleRatio = p2Ratio;

    part1->moveGlobal(dPos*p1Ratio, p1AngleRatio*dAng, m_boundary);
    part2->moveGlobal(-dPos*p2Ratio,p2AngleRatio*-dAng, m_boundary);

  }
}


/** Take weighted average of particles (with SD)
    and set as global location of world object WO_SELF */
void PFLocalization::estimateRobotPose(){

  // init vars
  Point2D w_possum;
  w_possum.setPoint(0, 0);
  Vector2D w_ang;
  Point2D sum;
  Point2D sum_sqr;
  sum.setPoint(0, 0);
  sum_sqr.setPoint(0, 0);

  AngRad angsum = 0.0;
  AngRad angdiff_sqr = 0.0;


  // calculate average and sd's
  float probval = 0.0;
  for( int i = 0; i < partParams.NUM_PARTICLES; i++ ) {
    Particle *part = &(localizationMem->particles[i]);
    w_possum += part->getPosition() * part->getProbability();
    probval += part->getProbability();
    w_ang += Vector2D( part->getProbability(),
                       part->getAngle(), POLAR );

    sum += part->getPosition();
    sum_sqr += part->getPosition() * part->getPosition();

    angsum += part->getAngle();
    // dont this this will work right with angles wrapping around
    //angsum_sqr += square(part->getAngle());

  }

  Point2D mean = sum / partParams.NUM_PARTICLES;
  Point2D var  = (sum_sqr - sum*mean) / partParams.NUM_PARTICLES;
  Point2D sd ( sqrtf(var.x), sqrtf(var.y));

  AngRad angmean = angsum / partParams.NUM_PARTICLES;

  // Second loop to get the squared differences of the angles
  for( int i = 0; i < partParams.NUM_PARTICLES; i++ ) {
    Particle *part = &(localizationMem->particles[i]);

    angdiff_sqr += square(normalizeAngle(part->getAngle() - angmean));
  }

  AngRad angvar = angdiff_sqr / partParams.NUM_PARTICLES;
  AngRad angsd = sqrtf(angvar);

  // set WO_SELF location
  worldObjects->objects_[robotState->WO_SELF].loc = ( w_possum / probval );
  worldObjects->objects_[robotState->WO_SELF].orientation = w_ang.getDirection();

  // set WO_SELF sd
  worldObjects->objects_[robotState->WO_SELF].sd = sd;
  worldObjects->objects_[robotState->WO_SELF].sdOrientation = angsd;
}

/* Thid function is used to draw the beacons on the map.  It averages where each particle believes the landmark are */
void PFLocalization::estimateBeaconLocations()
{
	WorldObject *objects = &(worldObjects->objects_[0]);

	for(int i = 0; i < NUM_BEACONS; i++)
	{
		if(beaconsSeen[i] == true)  //if this beacon is on the map
		{	
			float sumX = 0.0;
			float sumY = 0.0;
	
	  		for(int a = 0; a < partParams.NUM_PARTICLES; a++)  //see where each particle thinks it is and get the average
	  		{	
				LandmarkKF *landmark = localizationMem->particles[a].getLandmarkForID(i);
				NMatrix tempo = landmark->mean;
	  			sumX += tempo[0][0];
	  			sumY += tempo[1][0];
			}
			objects[i + BEACON_OFFSET].loc = Point2D(sumX/partParams.NUM_PARTICLES, sumY/partParams.NUM_PARTICLES); 
		}
	}
}

/** For each world object, calculate relative location and
    update its world object. */
void PFLocalization::estimateRelativeObjectLocations(){

  Point2D myloc = worldObjects->objects_[robotState->WO_SELF].loc;
  AngRad myorient = worldObjects->objects_[robotState->WO_SELF].orientation;

  for (int i = 0; i < NUM_WORLD_OBJS; i++){
    // dont update robots
    if (i >= WO_TEAM1 && i <= WO_OPPONENT4)
      continue;

    // calculate distance and bearing to each object
    worldObjects->objects_[i].distance =
      myloc.getDistanceTo(worldObjects->objects_[i].loc);
    worldObjects->objects_[i].bearing =
      myloc.getBearingTo(worldObjects->objects_[i].loc,
                         myorient);

  }

}


float PFLocalization::getProbabilitySum(Particle* part, int numPart){
  float probSum = 0.0;
  for (int i = 0; i < numPart; i++){
    probSum += part[i].getProbability();
  }
  return probSum;
}


void PFLocalization::printParticles(){

  for (int i = 0; i < partParams.NUM_PARTICLES; i++){
    Particle *part = &(localizationMem->particles[i]);
    pfLog((98, " Particle %d: ((%5.0f, %5.0f), %5.0f, %5.5f)", i, part->M_pos.x,
           part->M_pos.y, Rad2Deg(part->M_ang), part->M_prob));
  }
}



/**
 * Comparison function for quick sort
 */
int compareProbabilities( const void *one, const void *two ) {
  Particle *a, *b;
  a = (Particle*) one;
  b = (Particle*) two;
  if( a->getProbability() >= b->getProbability() ) {
    return -1;
  }
  else {
    return 1;
  }
}
