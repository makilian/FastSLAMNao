#ifndef LOCALIZATIONBLOCK_
#define LOCALIZATIONBLOCK_

#include "MemoryBlock.h"
#include <localization/Particle.h>


#define MAX_PARTICLES 1000

struct LocalizationBlock : public MemoryBlock {
public:
  LocalizationBlock()  {
    header.version = 5;
    header.size = sizeof(LocalizationBlock);
    NUM_PARTICLES = MAX_PARTICLES;
  }

  bool beaconsSeen[6];
  int NUM_PARTICLES;
  Particle particles[MAX_PARTICLES];

};

#endif 
