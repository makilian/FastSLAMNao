#ifndef _PF_Localization_H_
#define _PF_Localization_H_

#include "Particle.h"

#include <Module.h>

#include <memory/Memory.h>

#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/TeamPacketsBlock.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/JointBlock.h>
#include <memory/BehaviorBlock.h>
#include <memory/ProcessedSonarBlock.h>

#include <math/Pose2D.h>
#include <math/Geometry.h>
#include <common/NMatrix.h>
#include <common/WorldObject.h>
#include <common/RobotInfo.h>
#include <common/States.h>
#include <common/Field.h>


int compareProbabilities(const void *one, const void *two);

//#define ALLOW_LOC_DEBUG

#ifdef TOOL
#define ALLOW_LOC_DEBUG
#endif

// Parameters
struct ParticleParams{
  int NUM_PARTICLES;

  int RESAMPLE_FREQ;

  float DELTA_DIST;
  AngRad DELTA_ANG;
  float DEGRADE_FACTOR;

  bool RESET_PROB_EVERY;

  bool LOC_DEBUG;
  bool USE_BEACONS;

  bool USE_KNOWN_LINES;



  // Sigma's for different observation types
  float BEACON_BEAR_SIGMA;
  float BEACON_DIST_SIGMA;

  bool USE_BEARINGS;
  bool USE_DISTANCES;


  float MAX_BEACON_DIST;
 
};





class PFLocalization: public Module  {
 public:
  void specifyMemoryDependency();
  void specifyMemoryBlocks();
  void initSpecificModule();

  void reInit();

  PFLocalization();
  ~PFLocalization();

  void processFrame();

  // memory info
  WorldObjectBlock* worldObjects;
  LocalizationBlock* localizationMem;
  TeamPacketsBlock* teamPacketsMem;
  FrameInfoBlock* frameInfo;
  RobotStateBlock* robotState;
  GameStateBlock* gameState;
  OdometryBlock* odometry;
  JointBlock* jointAngles;
  BehaviorBlock* behaviorMem;
  ProcessedSonarBlock* processedSonar;

  // our ballkf
  float timePassed;
  float timeLast;

  bool beaconsSeen[6];

  //measurement noise matrix
  NMatrix measurementNoise;


  // Temp particles for swapping
  // (so we're not allocating memory for this all the time)
  Particle* tmpParticles;

  ParticleParams partParams;

  //void setParams(ParticleParams* params){ partParams = params; };

  // Boundary for particles to stay in
  Rectangle m_boundary;


  // PFLocalization functions on particles

  // Particle updates
  void updateParticlesFromObservations();
  void updateParticleFromObservations(Particle *part);

  void updateParticlesFromOdometry();

  // reseeding and resampling
  void resampleParticles();



  void randomWalkParticles();
  void estimateRobotPose();
  void estimateRelativeObjectLocations();
  void estimateBeaconLocations();

  float getProbabilitySum(Particle *part, int numPart);
  float calculateImportance(Particle *part, LandmarkKF *landmark, NMatrix measurement);

  void setParticleProbabilities(float newProb);
  void resetParticles();
  void printParticles();

};

#endif
