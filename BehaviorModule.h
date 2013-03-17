#ifndef BEHAVIOR_MODULE_H
#define BEHAVIOR_MODULE_H

#include <Module.h>

#include <memory/Memory.h>
#include <memory/RobotStateBlock.h>

class FrameInfoBlock;
class WorldObjectBlock;
class JointCommandBlock;
class WalkRequestBlock;
class BodyModelBlock;
class JointBlock;
class OdometryBlock;
class SensorBlock;
class RobotStateBlock;
class LocalizationBlock;

class BehaviorModule : public Module {

double timePassedSincePanMotion;
double lastPanMotionTime;
int panCount;
double durationOfPanMotion;

double timePassedSinceTiltMotion;
double lastTiltMotionTime;
int tiltCount;
double durationOfTiltMotion;

public:
  void specifyMemoryDependency();
  void specifyMemoryBlocks();

  void processFrame();


private:
  // memory info
  FrameInfoBlock *frameInfo;
  WorldObjectBlock *worldObjects;
  JointCommandBlock *jointCommands;
  WalkRequestBlock *walkRequest;
  BodyModelBlock *bodyModel;
  JointBlock *jointAngles;
  LocalizationBlock *localization;
  OdometryBlock *odometry;
  SensorBlock *sensors;
  RobotStateBlock *robotState;
};

#endif
