#include "BehaviorModule.h"
#include <iostream>

// memory
#include <memory/FrameInfoBlock.h>
#include <memory/WorldObjectBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/WalkRequestBlock.h>
#include <memory/BodyModelBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/JointBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/SensorBlock.h>

static float startTime = -1;
static float currTime;
static Point2D initialHalfway;
enum State {BEGIN, WATCHING, WALKING, CENTERING};
static State state;
static float fiveSecondStart;
bool first = true;

void BehaviorModule::specifyMemoryDependency() {
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("vision_joint_commands");
  requiresMemoryBlock("vision_walk_request");
  requiresMemoryBlock("vision_body_model");
  requiresMemoryBlock("vision_joint_angles");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("vision_odometry");
  requiresMemoryBlock("vision_sensors");
  requiresMemoryBlock("robot_state");
}

void BehaviorModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(frameInfo,"vision_frame_info");
  getOrAddMemoryBlock(worldObjects,"world_objects");
  getOrAddMemoryBlock(jointCommands,"vision_joint_commands");
  getOrAddMemoryBlock(walkRequest,"vision_walk_request");
  getOrAddMemoryBlock(bodyModel,"vision_body_model");
  getOrAddMemoryBlock(jointAngles,"vision_joint_angles");
  getOrAddMemoryBlock(localization, "localization");
  getOrAddMemoryBlock(odometry,"vision_odometry");
  getOrAddMemoryBlock(sensors,"vision_sensors");
  getOrAddMemoryBlock(robotState,"robot_state");
}

void BehaviorModule::processFrame() 
{	
	if(startTime == -1)
	{	
		startTime = frameInfo->seconds_since_start;
		state = BEGIN;
	}

	currTime = frameInfo->seconds_since_start;
	timePassedSincePanMotion = currTime - lastPanMotionTime;
	

	if(timePassedSincePanMotion >= durationOfPanMotion/1000)// && (currTime - startTime < 10))
	{
		panCount++;
		durationOfPanMotion = 2000;
		
		if(panCount % 4 == 0)		
			jointCommands->setHeadPan(DEG_T_RAD * 10, 2000.0, false);
		else if(panCount % 4 == 1)
			jointCommands->setHeadTilt(DEG_T_RAD * -5, 2000.0, false);
		else if(panCount % 4 == 2)
			jointCommands->setHeadPan(DEG_T_RAD * -10, 2000.0, false);
		else
			jointCommands->setHeadTilt(DEG_T_RAD * 5, 2000.0, false);
			
		lastPanMotionTime = currTime;
	}


	if(currTime - startTime <= 25)
		state = BEGIN;
	else if(currTime - startTime <= 35)
		state = WALKING;
	else
	{
		if(first)
		{
			fiveSecondStart = currTime;
			first = false;
			state = WATCHING;
		}
		if(currTime - fiveSecondStart >= 5)
		{
			fiveSecondStart = currTime; 
			if(state == WATCHING)
				state = CENTERING;
			else if(state == CENTERING)
				state = WATCHING;
		}
	}

	if(state == BEGIN)
	{
		//do nothing
		cout << "BEGIN" << endl;
	}
	else if(state == CENTERING) //if 15 seconds has passed, let's hope we've built the map correctly!
	{
		Point2D myloc = worldObjects->objects_[robotState->WO_SELF].loc;
		AngRad myorient = worldObjects->objects_[robotState->WO_SELF].orientation;
		int count = 0;
		int xCoord = 0;
		int yCoord = 0;
			
		for(int x = 0; x < 6; x++)
		{
			if(localization->beaconsSeen[x] == true)
			{
				cout << "Beacon " << x + 56 << " (" << worldObjects->objects_[x+56].loc.x << ", " << worldObjects->objects_[x+56].loc.y << ") " << endl;
				xCoord += worldObjects->objects_[x + 56].loc.x;
				yCoord += worldObjects->objects_[x + 56].loc.y;
				count++;
			}
		}

		xCoord = xCoord/count;
		yCoord = yCoord/count;	

		Point2D target = Point2D(xCoord, yCoord);
		Point2D relativeTarget = target.globalToRelative(myloc, myorient);	
		AngRad relativeAngleToTarget = myloc.getBearingTo(target, myorient);

		cout << "MY LOC: " << myloc << endl;
		cout << "MY ANGLE: " << myorient << endl;
		cout << "TARGET: " << target << endl;
		cout << "RELATIVE TARGET: " << relativeTarget << endl;
		cout << "RELATIVE ANGLE TO TARGET: " << relativeAngleToTarget << endl;

		walkRequest->setWalkTarget(relativeTarget.x - 200, relativeTarget.y, relativeAngleToTarget/2);
	}
	else if (state == WALKING)
	{
		cout << "WALKING" << endl;
		walkRequest->setWalk(.8, 0, .0);
	}
	else if(state == WATCHING)
	{
		cout << "WATCHING" << endl;
		walkRequest->setWalk(.0, 0, .0);
	}
	else
	{
		cout << "WTF??" << endl;

	}
}
