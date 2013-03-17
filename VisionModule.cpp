#include "VisionModule.h"
#include <memory/FrameInfoBlock.h>
#include <memory/JointBlock.h>
#include <memory/RobotVisionBlock.h>
#include <memory/SensorBlock.h>
#include <memory/ImageBlock.h>
#include <memory/BodyModelBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/CameraBlock.h>
#include <memory/WorldObjectBlock.h>
#include <memory/RobotInfoBlock.h>

#include <boost/lexical_cast.hpp>

const uint16_t VisionModule::IMAGE_WIDTH_DIV2 = IMAGE_WIDTH / 2;
const uint16_t VisionModule::IMAGE_WIDTH_DIV4 = IMAGE_WIDTH / 4;
const uint16_t VisionModule::IMAGE_HEIGHT_DIV2 = IMAGE_HEIGHT / 2;
const uint16_t VisionModule::IMAGE_HEIGHT_DIV4 = IMAGE_HEIGHT / 4;

std::ostream& operator<<(std::ostream& out, const SegCoord &p) {
  out << "(" << p.r << ", " << p.c << ")";

  return out;
}

std::ostream& operator<<(std::ostream& out, const RLE &r) {
  out << r.id << " ";

  if (r.color == sc_UNDEFINED) {
    out << "UNDEFINED";
  }
  else if (r.color == sc_FIELD_GREEN) {
    out << "FIELD GREEN";
  }
  else if (r.color == sc_WHITE) {
    out << "WHITE";
  }
  else if (r.color == sc_ORANGE) {
    out << "ORANGE";
  }
  else if (r.color == sc_PINK) {
    out << "PINK";
  }
  else if (r.color == sc_BLUE) {
    out << "BLUE";
  }
  else if (r.color == sc_YELLOW) {
    out << "YELLOW";
  }

  out << " starts at " << r.start << " ends at " << r.end;
  out << " parent ID " << r.parentID;

  return out;
}

std::ostream& operator<<(std::ostream& out, const Blob &b) {
  out << b.id << " ";

  if (b.color == sc_UNDEFINED) {
    out << "UNDEFINED";
  }
  else if (b.color == sc_FIELD_GREEN) {
    out << "FIELD GREEN";
  }
  else if (b.color == sc_WHITE) {
    out << "WHITE";
  }
  else if (b.color == sc_ORANGE) {
    out << "ORANGE";
  }
  else if (b.color == sc_PINK) {
    out << "PINK";
  }
  else if (b.color == sc_BLUE) {
    out << "BLUE";
  }
  else if (b.color == sc_YELLOW) {
    out << "YELLOW";
  }

  out << " Center: " << b.center << " Area: " << b.area
      << " Num pixels: " << b.numPixels << std::endl;

  out << "TL: " << b.tl << " TR: " << b.tr << " BL: " << b.bl;
  out << " BR: " << b.br << " H: " << b.height << " W: " << b.width
      << std::endl;

  return out;
}

void VisionModule::specifyMemoryDependency() {
  //last_frame_processed_ = 0;

  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("vision_joint_angles");
  requiresMemoryBlock("robot_vision");
  requiresMemoryBlock("vision_sensors");
  requiresMemoryBlock("raw_image");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("vision_body_model");
  requiresMemoryBlock("camera_info");
  requiresMemoryBlock("robot_info");

  providesMemoryBlock("world_objects");
}

void VisionModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(vision_frame_info_,"vision_frame_info");
  getOrAddMemoryBlock(joint_angles_,"vision_joint_angles");
  getOrAddMemoryBlock(robot_vision_,"robot_vision");
  getOrAddMemoryBlock(sensors_,"vision_sensors");
  getOrAddMemoryBlock(image_,"raw_image");
  getOrAddMemoryBlock(world_objects_,"world_objects");
  getOrAddMemoryBlock(robot_state_,"robot_state");
  getOrAddMemoryBlock(body_model_,"vision_body_model");
  getOrAddMemoryBlock(camera_info_,"camera_info");

  getOrAddMemoryBlock(robot_info_,"robot_info");
}

void VisionModule::processFrame() {

  if (getCameraType() == Camera::UNKNOWN) {
    return;
  }

  world_objects_->reset();

  // check new transform
  float cameraTilt;
  float cameraXOffset;
  float cameraZOffset;

  switch(getCameraType()) {
    case Camera::TOP:
      cameraTilt = 2.0 * DEG_T_RAD;
      cameraXOffset = 53.9;
      cameraZOffset = 67.9;
      break;
    case Camera::BOTTOM:
      cameraTilt = 48.0 * DEG_T_RAD;
      cameraXOffset = 48.80;
      cameraZOffset = 23.81;
      break;
    default:
      cameraTilt = cameraXOffset = cameraZOffset = 0;
  }

  transform_.calculateTransformation(getCameraMatrix(), getTrueFrontHeight());

  segmentImage();
  detectBeacons();

}

/* Placeholder functions */

Camera::Type VisionModule::getCameraType() {
  return camera_info_->current_camera_;
}

bool VisionModule::isBottomCamera() {
  return (camera_info_->current_camera_ == Camera::BOTTOM);
}

bool VisionModule::useSimColorTable() {
  return (vision_frame_info_->source == MEMORY_SIM);
}

string VisionModule::getDataBase() {
  return memory_->data_path_;
}

float VisionModule::getTrueFrontHeight() {
  return body_model_->abs_parts_[BodyPart::torso].translation.z;
}

int VisionModule::getRobotId() {
  return robot_state_->robot_id_;
}

Pose3D VisionModule::getCameraMatrix() {
  if (getCameraType() == Camera::TOP)
    return body_model_->rel_parts_[BodyPart::top_camera];
  return body_model_->rel_parts_[BodyPart::bottom_camera];
}

int VisionModule::getTeamColor() {
  return robot_state_->team_;
}

void VisionModule::initSpecificModule() {
  loadColorTables();
}

VisionModule::VisionModule() {

  printf("Creating vision system");
  fflush(stdout);
  
  bottomColorTable = new unsigned char [LUT_SIZE];
  topColorTable = new unsigned char [LUT_SIZE];
  bottomColorTableName = "none";
  topColorTableName = "none";
  memset(bottomColorTable, c_UNDEFINED, LUT_SIZE);
  memset(topColorTable, c_UNDEFINED, LUT_SIZE);
  printf(".");
  fflush(stdout);

  printTable = true;
}

VisionModule::~VisionModule() {
  if (topColorTable != bottomColorTable)
    delete [] topColorTable;
  delete [] bottomColorTable;
}

bool VisionModule::loadColorTables() {
  bool uniqueColorTablesAvailable = true;
  if (useSimColorTable()) {
    loadColorTable(Camera::BOTTOM, "sim.col");
    // just set top to match bottom (esp so both pointers are the same, and editing either edits both in tool)
    topColorTable = bottomColorTable;
    topColorTableName = bottomColorTableName;
  } else {

    // Attempt to load robot-specific color tables if available
    std::string bottomColorTableFile = 
      boost::lexical_cast<std::string>(getRobotId()) +
      "bottom.col";
    std::string topColorTableFile = 
      boost::lexical_cast<std::string>(getRobotId()) +
      "top.col";
    bool bottomOk = loadColorTable(Camera::BOTTOM, bottomColorTableFile.c_str(), false);
    bool topOk = loadColorTable(Camera::TOP, topColorTableFile.c_str(), false);

    // If not available, then load the default top and bottom color tables 
    uniqueColorTablesAvailable = bottomOk && topOk;
    if (!bottomOk) {
      bottomOk = loadColorTable(Camera::BOTTOM, "defaultbottom.col");
    }
    if (!topOk) {
      topOk = loadColorTable(Camera::TOP, "defaulttop.col");
    }

    // If still not available, then load the single default color table
    if (!bottomOk) {
      bottomOk = loadColorTable(Camera::BOTTOM, "default.col");
    }
    if (!topOk) {
      topOk = loadColorTable(Camera::TOP, "default.col");
    }
   

  }
  return uniqueColorTablesAvailable;
}

bool VisionModule::loadColorTable(Camera::Type camera, const char* fileName, bool fullPath) {
  unsigned char* colorTable = bottomColorTable;
  if (camera == Camera::TOP) {
    colorTable=topColorTable;
  }
  string colorTableName;
  if (!fullPath) {
    colorTableName = getDataBase() + string(fileName);
  } else {
    colorTableName = std::string(fileName);
  }

  FILE* f=fopen(colorTableName.c_str(), "rb");
  if (f==NULL) {
    //std::cout << "Vision: *** ERROR can't load " << colorTableName << " *** for camera " << camera << std::endl << std::flush;
    colorTableName = "none";
    return false;
  }
  //std::cout << "Vision: Loaded " << colorTableName << " ! for camera " << camera << std::endl << std::flush;
  bool ok = fread(colorTable,LUT_SIZE,1,f);
  fclose(f);

  if (camera==Camera::TOP) {
    topColorTableName = colorTableName;
  } else {
    bottomColorTableName = colorTableName;
  }

  return ok;
}

// Simple segmentation
void VisionModule::segmentImage() {

  unsigned char* colorTable = bottomColorTable;
  if (!isBottomCamera())
    colorTable = topColorTable;

  static const uint32_t
    IMAGE_WIDTH_SKIP0 = 0,
    IMAGE_WIDTH_SKIP1 = IMAGE_WIDTH_SKIP0 + IMAGE_WIDTH_DIV2;
  uint32_t x, y;

  // srcPtr is a uint32_t pointer, so it gets 4 bytes of information at a given time
  uint32_t *srcPtr = (uint32_t *) image_->img_.get();
  for (y = 0; y < IMAGE_HEIGHT_DIV4; y += 1, srcPtr += 3 * IMAGE_WIDTH_SKIP1) {
    for (x = 0; x < IMAGE_WIDTH_DIV4; srcPtr += 2, x++) {
  
      #ifdef TOOL
      uint8_t c;
      if (!image_->img_) // if no raw img, grab segImg
        c = robot_vision_->segImg[SEG_IMAGE_WIDTH * y + x];
      else {
        c = colorTable[
          ((*srcPtr & 0x000000feul) << 13) | // y1
          ((*srcPtr & 0x0000fe00ul) >>  2) | // u
          ((*srcPtr & 0xfe000000ul) >> 25)]; // v
        robot_vision_->segImg[SEG_IMAGE_WIDTH * y + x] = c;
      }
      #else
      uint8_t c = colorTable[
        ((*srcPtr & 0x000000feul) << 13) | // y1
        ((*srcPtr & 0x0000fe00ul) >>  2) | // u
        ((*srcPtr & 0xfe000000ul) >> 25)]; // v
      robot_vision_->segImg[SEG_IMAGE_WIDTH * y + x] = c;
      #endif
    }
  }
}

void VisionModule::detectBeacons() {
  // TODO: Use the seg image here, and implement your blob formation code to detect beacons

  std::vector< std::list<RLE> > rleList(SEG_IMAGE_HEIGHT);
  RawBlobType blobs;
  std::vector<Blob> blobList;

  createRLEs(rleList);
  //printRLEs(rleList);

  mergeRLEs(rleList, blobs);
  //printBlobs(blobs);

  getBlobStats(blobs, blobList);
  //printBlobs(blobList);

  //printBlobs(blobList);
  findBeacons(blobList);

  if (OUTPUT) {
    //std::cout << "-----" << std::endl << std::endl;
  }

  // (piyushk): Testing
  //insertFakeBeacon(WO_BEACON_BLUE_OVER_YELLOW, 80,60);
  //insertFakeBeacon(WO_BEACON_YELLOW_OVER_PINK, 40,20);
}

void VisionModule::createRLEs(std::vector< std::list<RLE> > &rleList) {
  SegColor color;
  int cnt = 0;

  for (int r = 0; r < SEG_IMAGE_HEIGHT; ++r) {
    rleList[r] = std::list<RLE>();
    color = sc_UNDEFINED;

    for (int c = 0; c < SEG_IMAGE_WIDTH; ++c) {
      // Check if this pixel is a new color
      if (color != segAt(c, r)) {
        // If there was a previous RLE, set its end point
        if (!rleList[r].empty() && (rleList[r].back().end.r < 0)) {
          rleList[r].back().end = SegCoord(r, c - 1);
        }

        color = (SegColor)segAt(c, r);

        // If this is a defined color, make an RLE
        if (color != sc_UNDEFINED && color != sc_WHITE && color != sc_ORANGE) {
          RLE rle;
          rle.id = cnt;
          rle.parentID = rle.id;
          rle.color = color;
          rle.start = SegCoord(r, c);

          rleList[r].push_back(rle);

          ++cnt;
        }
      }
    } // for (int c)

    // If there was a previous RLE, set its end point
    if (!rleList[r].empty() && (rleList[r].back().end.r < 0)) {
      rleList[r].back().end = SegCoord(r, SEG_IMAGE_WIDTH - 1);
    }
  } // for (size_t r)
}

void VisionModule::mergeRLEs(std::vector< std::list<RLE> > &rleList,
                             RawBlobType &blobs) {
  std::list<RLE>::iterator cIt;
  std::list<RLE>::iterator pIt;
  bool isRoot;

  for (cIt = rleList[0].begin(); cIt != rleList[0].end(); ++cIt) {
    std::map<int, RLE> m;
    m[(*cIt).id] = *cIt;
    blobs.push_back(std::make_pair((*cIt).id, m));
  }

  // Merge adjacent rows by setting parent pointer to parent at highest row
  for (size_t r = 1; r < rleList.size(); ++r) {
    pIt = rleList[r - 1].begin();

    for (cIt = rleList[r].begin(); cIt != rleList[r].end(); ++cIt) {
      if ((*cIt).color != sc_UNDEFINED) {
        isRoot = true;

        for (pIt = rleList[r - 1].begin(); pIt != rleList[r - 1].end(); ++pIt) {
          if ((*pIt).start.c > (*cIt).end.c) {
            break;
          }

          if ((*pIt).color == (*cIt).color) {
            // Check for overlap with previous row
            if (((*pIt).start.c >= (*cIt).start.c &&
                 (*pIt).start.c <= (*cIt).end.c)
                 ||
                ((*cIt).start.c >= (*pIt).start.c &&
                 (*cIt).start.c <= (*pIt).end.c)) {
              if ((*cIt).parentID == (*cIt).id) {
                // This is not a root RLE, add it to the root RLE's list
                isRoot = false;
                (*cIt).parentID = (*pIt).parentID;
                setAdd(blobs, *cIt);
              }
              else {
                // Check if previous row RLE is in the same list as current RLE
                if (!setContains(blobs, (*cIt).parentID, (*pIt).id)) {
                  // Merge all RLEs in previous's parent with current's parent
                  setMerge(blobs, (*cIt).parentID, (*pIt).parentID);
                }
              }
            }
          } // if (pIt.color == cIt.color)
        } // for (pIt)

        // If no overlap with previous row, this is a root RLE
        if (isRoot) {
          std::map<int, RLE> m;
          m[(*cIt).id] = *cIt;
          blobs.push_back(std::make_pair((*cIt).id, m));
        }
      } // if (cIt.color != UNDEF)
    } // for (cIt)
  } // for (size_t r)
}

void VisionModule::getBlobStats(RawBlobType &raw, std::vector<Blob> &blobs) {
  int cnt = 0;
  std::map<int, RLE>::iterator it;

  for (size_t i = 0; i < raw.size(); ++i) {
    if (!(raw[i].second).empty() && (raw[i].second).size() > 1) {
      // Initialize blob
      Blob b;
      b.id = cnt;
      b.numPixels = 0;

      it = (raw[i].second).begin();

      b.color = it->second.color;

      float sr, sc, ec;
      float top = SEG_IMAGE_HEIGHT + 1;
      float bot = -1.0;
      float lft = SEG_IMAGE_WIDTH + 1;
      float rgt = -1.0;

      for ( ; it != (raw[i].second).end(); ++it) {
        sr = it->second.start.r;
        sc = it->second.start.c;
        ec = it->second.end.c;

        // Find the bounding box
        if (top > sr) {
          top = sr;
        }
        if (bot < sr) {
          bot = sr;
        }
        if (lft > sc) {
          lft = sc;
        }
        if (rgt < ec) {
          rgt = ec;
        }

        b.numPixels += ec - sc;
      }

      // Set the 4 corners of the bounding box; if the rectangle blob is
      // not orthogonal to the screen, the box will be bigger than the blob
      b.tl = SegCoord(top, lft);
      b.tr = SegCoord(top, rgt);
      b.bl = SegCoord(bot, lft);
      b.br = SegCoord(bot, rgt);

      // Find the center coordinate of each side of the bounding box
      SegCoord avgTop = b.tl ^ b.tr;
      SegCoord avgBot = b.bl ^ b.br;
      SegCoord avgLft = b.tl ^ b.bl;
      SegCoord avgRgt = b.tr ^ b.br;

      // Set the dimensions of the box
      b.height = avgBot.dist(avgTop);
      b.width = avgRgt.dist(avgLft);
      b.area = b.height * b.width;

      // Average of the midpoints of the top and bottom sides of the box is
      // the center of the entire bounding box
      b.center = avgTop ^ avgBot;

      // Blob must have min size to filter out noise
      if (b.numPixels > MIN_BLOB_PIXELS) {
        blobs.push_back(b);
        ++cnt;
      }
    }
  } // for (raw size)
}

void VisionModule::findBeacons(const std::vector<Blob> &blobs) {
  bool known[NUM_BEACONS];
  for (int i = 0; i < NUM_BEACONS; ++i) {
    known[i] = false;
  }

  // Compare every pair of blobs to determine if they form a beacon
  if (blobs.size() > 1) {
    for (size_t i = 0; i < blobs.size() - 1; ++i) {
      for (size_t j = i + 1; j < blobs.size(); ++j) {
        classifyBeacon(blobs, blobs[i], blobs[j], known);
        classifyBeacon(blobs, blobs[j], blobs[i], known);
      }
    }
  }
}

void VisionModule::classifyBeacon(const std::vector<Blob> &blobs,
                                  const Blob &x, const Blob &y, bool known[]) {
  /*
  BEACON_OFFSET = WO_BEACON_BLUE_OVER_YELLOW;
  WO_BEACON_BLUE_OVER_YELLOW = 56
  WO_BEACON_YELLOW_OVER_BLUE = 57
  WO_BEACON_BLUE_OVER_PINK =   58
  WO_BEACON_PINK_OVER_BLUE =   59
  WO_BEACON_PINK_OVER_YELLOW = 60
  WO_BEACON_YELLOW_OVER_PINK = 61
  */

  if (x.color == sc_BLUE && y.color == sc_YELLOW) {
    if (x.center.r < y.center.r) {
      // blue over yellow
      if (!known[WO_BEACON_BLUE_OVER_YELLOW - BEACON_OFFSET] &&
          isValidBeacon(blobs, WO_BEACON_BLUE_OVER_YELLOW, x, y)) {
        known[WO_BEACON_BLUE_OVER_YELLOW - BEACON_OFFSET] = true;
      }
    }
    else {
      // yellow over blue
      if (!known[WO_BEACON_YELLOW_OVER_BLUE - BEACON_OFFSET] &&
          isValidBeacon(blobs, WO_BEACON_YELLOW_OVER_BLUE, y, x)) {
        known[WO_BEACON_YELLOW_OVER_BLUE - BEACON_OFFSET] = true;
      }
    }
  }
  else if (x.color == sc_BLUE && y.color == sc_PINK) {
    if (x.center.r < y.center.r) {
      // blue over pink
      if (!known[WO_BEACON_BLUE_OVER_PINK - BEACON_OFFSET] &&
          isValidBeacon(blobs, WO_BEACON_BLUE_OVER_PINK, x, y)) {
        known[WO_BEACON_BLUE_OVER_PINK - BEACON_OFFSET] = true;
      }
    }
    else {
      // pink over blue
      if (!known[WO_BEACON_PINK_OVER_BLUE - BEACON_OFFSET] &&
          isValidBeacon(blobs, WO_BEACON_PINK_OVER_BLUE, y, x)) {
        known[WO_BEACON_PINK_OVER_BLUE - BEACON_OFFSET] = true;
      }
    }
  }
  else if (x.color == sc_PINK && y.color == sc_YELLOW) {
    if (x.center.r < y.center.r) {
      // pink over yellow
      if (!known[WO_BEACON_PINK_OVER_YELLOW - BEACON_OFFSET] &&
          isValidBeacon(blobs, WO_BEACON_PINK_OVER_YELLOW, x, y)) {
        known[WO_BEACON_PINK_OVER_YELLOW - BEACON_OFFSET] = true;
      }
    }
    else {
      // yellow over pink
      if (!known[WO_BEACON_YELLOW_OVER_PINK - BEACON_OFFSET] &&
          isValidBeacon(blobs, WO_BEACON_YELLOW_OVER_PINK, y, x)) {
        known[WO_BEACON_YELLOW_OVER_PINK - BEACON_OFFSET] = true;
      }
    }
  }
}

bool VisionModule::isValidBeacon(const std::vector<Blob> &blobs, int beaconID,
                                 const Blob &top, const Blob &bot) {
  // Determine if this is a partial beacon
  bool partialTop = false;
  bool partialBot = false;
  bool partialLft = false;
  bool partialRgt = false;

  if (top.tl.r <= 3 || top.tr.r <= 3) {
    partialTop = true;
  }
  if (bot.bl.r >= SEG_IMAGE_HEIGHT - 3 || bot.br.r >= SEG_IMAGE_HEIGHT - 3) {
    partialBot = true;
  }

  if (top.tl.c <=3 || bot.bl.c <= 3) {
    partialLft = true;
  }
  if (top.tr.c >= SEG_IMAGE_WIDTH - 3 || bot.br.c >= SEG_IMAGE_WIDTH - 3) {
    partialRgt = true;
  }

  bool partial = partialTop || partialBot || partialLft || partialRgt;

  // Check if adjacent corners are too far away
  if (top.bl.dist(bot.tl) >= DIST_THRESHOLD ||
      top.br.dist(bot.tr) >= DIST_THRESHOLD) {
    if (OUTPUT) {
      //std::cout << "REJECT (corners) beacon " << beaconID << std::endl;
      //std::cout << "top = " << top << "bot = " << bot << std::endl;
    }
    return false;
  }

  // Check ratios
  float topRatio = top.height / top.width;
  float botRatio = bot.height / bot.width;
  if (!partialLft && !partialRgt) {

    if (!partialTop && !partialBot) {
      if (topRatio < 0.7 || topRatio > 1.3 ||
          botRatio < 0.7 || botRatio > 1.3) {
        if (OUTPUT) {
          //std::cout << "REJECT (ratio) beacon " << beaconID << std::endl;
          //std::cout << "top = " << top << "bot = " << bot << std::endl;
        }
        return false;
      }
    }
    if (partialTop && !partialBot) {
      if (botRatio < 0.7 || botRatio > 1.3) {
        if (OUTPUT) {
          //std::cout << "REJECT (ratio) beacon " << beaconID << std::endl;
          //std::cout << "top = " << top << "bot = " << bot << std::endl;
        }
        return false;
      }
    }
    if (!partialTop && partialBot) {
      if (topRatio < 0.7 || topRatio > 1.3) {
        if (OUTPUT) {
          //std::cout << "REJECT (ratio) beacon " << beaconID << std::endl;
          //std::cout << "top = " << top << "bot = " << bot << std::endl;
        }
        return false;
      }
    }
  }

  if (!partialTop && !partialBot) {
    if (topRatio < 0.70 * botRatio || topRatio > 1.30 * botRatio ||
        botRatio < 0.70 * topRatio || botRatio > 1.30 * topRatio) {
      if (OUTPUT) {
        //std::cout << "REJECT (ratio) beacon " << beaconID << std::endl;
        //std::cout << "top = " << top << "bot = " << bot << std::endl;
      }
      return false;
    }
  }

  // Check if height and width are similar
  if (partial) {
    if (partialTop || partialBot) {
      if (bot.height > 2 * top.height || top.height > 2 * bot.height) {
        if (OUTPUT) {
          //std::cout << "REJECT (part height) beacon " << beaconID << std::endl;
          //std::cout << "top = " << top << "bot = " << bot << std::endl;
        }
        return false;
      }
    }
    if (partialLft || partialRgt) {
      if (top.width > 2 * bot.width || bot.width > 2 * top.width) {
        if (OUTPUT) {
          //std::cout << "REJECT (part width) beacon " << beaconID << std::endl;
          //std::cout << "top = " << top << "bot = " << bot << std::endl;
        }
        return false;
      }
    }
  }
  else {
    if (top.numPixels < bot.numPixels * 0.70 ||
        top.numPixels > bot.numPixels * 1.30 ||
        bot.numPixels < top.numPixels * 0.70 ||
        bot.numPixels > top.numPixels * 1.30) {
      if (OUTPUT) {
        //std::cout << "REJECT (num pixels) beacon " << beaconID << std::endl;
        //std::cout << "top = " << top << "bot = " << bot << std::endl;
      }
      return false;
    }

    if (top.height < bot.height * 0.70 || top.height > bot.height * 1.30) {
      if (OUTPUT) {
        //std::cout << "REJECT (height) beacon " << beaconID << std::endl;
        //std::cout << "top = " << top << "bot = " << bot << std::endl;
      }
      return false;
    }
    if (top.width < bot.width * 0.70 || top.width > bot.width * 1.30) {
      if (OUTPUT) {
        //std::cout << "REJECT (width) beacon " << beaconID << std::endl;
        //std::cout << "top = " << top << "bot = " << bot << std::endl;
      }
      return false;
    }
  }

  // Search through all other blobs that are not top or bot
  for (size_t i = 0; i < blobs.size(); ++i) {
    if (blobs[i].id != top.id && blobs[i].id != bot.id) {
      if (blobs[i].color != sc_WHITE) {
        // Check for colored blobs above top or below bot
        if ((top.tl.dist(blobs[i].bl) < DIST_THRESHOLD &&
             top.tr.dist(blobs[i].br) < DIST_THRESHOLD)
            ||
            (bot.bl.dist(blobs[i].tl) < DIST_THRESHOLD &&
             bot.br.dist(blobs[i].tr) < DIST_THRESHOLD)) {
          if (OUTPUT) {
            //std::cout << "REJECT (" << i <<") beacon " << beaconID << std::endl;
            //std::cout << "top = " << top << "bot = " << bot << std::endl;
          }
          return false;
        }
      }
    }
  }

  // If the side of the beacon is at a screen edge use the average height
  // of the blobs as the width of the beacon.
  float beaconWidth = (top.width + bot.width) / 2;

  if (partialLft || partialRgt) {
    beaconWidth = (top.height + bot.height) / 2;
  }

  float beaconX = (top.center.c + bot.center.c) / 2;
  float beaconY = (top.center.r + bot.center.r) / 2;

  // If the side of the beacon is at the screen edge, shift the center of the
  // beacon towards that side.
  if (partialRgt) {
    beaconX += (top.height / 2) - (top.width / 2); 
  }
  if (partialLft) {
    beaconX += (top.width / 2) - (top.height / 2);
  }

  float distance = getDistanceToBeacon(beaconWidth);

  setBeaconObject(beaconID,
                  beaconX,
                  beaconY,
                  distance,
                  beaconWidth,
                  partial);

  if (OUTPUT) {
    //std::cout << "*** BEACON " << beaconID
    //          << " at (" << beaconX << ", " << beaconY << ")"
    //          << "  dist = " << distance << " mm"
     //         << "  width = " << beaconWidth
    //          << "  partial = ";

    if (partial) {
      //std::cout << "true" << std::endl;  
    }
    else {
      //std::cout << "false" << std::endl;  
    }

    //std::cout << "top = " << top << "bot = " << bot << std::endl;
  }

  return true;
}

void VisionModule::setAdd(RawBlobType &blobs, const RLE &elem) {
  for (size_t i = 0; i < blobs.size(); ++i) {
    if ((blobs[i].second).find(elem.parentID) != (blobs[i].second).end()) {
      (blobs[i].second)[elem.id] = elem;
      return;
    }
  }

  ////std::cout << "*** ERROR: Parent not in blobs list for " << elem
  //          << std::endl;
}

bool VisionModule::setContains(RawBlobType &blobs, int setID, int elemID) {
  for (size_t i = 0; i < blobs.size(); ++i) {
    if (blobs[i].first == setID) {
      if ((blobs[i].second).find(elemID) != (blobs[i].second).end()) {
        return true;
      }
      else {
        return false;
      }
    }
  }

  ////std::cout << "*** ERROR: Set ID " << setID << " not in blobs list"
  //          << std::endl;

  return false;
}

void VisionModule::setMerge(RawBlobType &blobs, int dstID, int srcID) {
  for (size_t d = 0; d < blobs.size(); ++d) {
    if (blobs[d].first == dstID) {
      for (size_t s = 0; s < blobs.size(); ++s) {
        if (blobs[s].first == srcID) {
          std::map<int, RLE>::iterator it;

          for (it = (blobs[s].second).begin();
               it != (blobs[s].second).end();
               ++it) {
            it->second.parentID = dstID;
            (blobs[d].second)[it->second.id] = it->second;
          }

          (blobs[s].second).clear();

          return;
        }
      }
    }
  }
}

void VisionModule::printRLEs(std::vector< std::map<int, RLE> > &rleList) {
  if (rleList.empty()) {
    //std::cout << "RLE list was empty" << std::endl;
  }
  else {
    //std::cout << "RLEs:" << std::endl;

    std::map<int, RLE>::iterator it;

    for (size_t i = 0; i < rleList.size(); ++i) {
      for (it = rleList[i].begin(); it != rleList[i].end(); ++it) {
        if (it->second.id >= 0) {
          //std::cout << it->second << std::endl;
        }
      }
    }
  }

  //std::cout << std::endl << "-------" << std::endl;
}

void VisionModule::printBlobs(RawBlobType &blobs) {
  if (blobs.empty()) {
    //std::cout << "Blobs list was empty" << std::endl;
  }
  else {
    //std::cout << "Blobs:" << std::endl;

    std::map<int, RLE>::iterator it;

    for (size_t i = 0; i < blobs.size(); ++i) {
      if ((blobs[i].second).size() > 10) {
        //std::cout << "Blob root: " << blobs[i].first << std::endl;
        for (it = (blobs[i].second).begin();
             it != (blobs[i].second).end();
             ++it) {
          //std::cout << "ID: " << it->first
                    //<<" RLE: " << it->second << std::endl;
        }
        //std::cout << std::endl;
      }
    }
  }

  //std::cout << std::endl << "-------" << std::endl;
}

void VisionModule::printBlobs(std::vector<Blob> &blobs) {
  if (blobs.empty()) {
    //std::cout << "Blobs list was empty" << std::endl;
  }
  else {
    //std::cout << "Blobs:" << std::endl;

    for (size_t i = 0; i < blobs.size(); ++i) {
      //std::cout << blobs[i] << std::endl;
    }
  }

  //std::cout << std::endl << "-------" << std::endl;
}

float VisionModule::getDistanceToBeacon(float width) {
  // TODO: Fit a curve between distance and width to get the distance to the beacon here
  //return (-0.162 * width + 2.8499) * 1000; // Distance in mm
  return (19.5421 * pow(width, (float)-1.0424)) * 1000; // Distance in mm
}

// Call this function for every detected beacon
// beaconId is the id from $NAO_HOME/core/common/WorldObject.h
// x, y is the location of the beacon in the segmented image
// distance is the distance to the beacon
// pixelWidth is the width of the beacon in pixels
// partiallyVisible is set to true if the beacon is only partially visible, false otherwise
void VisionModule::setBeaconObject(int beaconId, int x, int y, float distance, float pixelWidth, bool partiallyVisible) {
  if (beaconId >= WO_BEACON_BLUE_OVER_YELLOW && beaconId <= WO_BEACON_YELLOW_OVER_PINK) {
    world_objects_->objects_[beaconId].seen = true;
    world_objects_->objects_[beaconId].visionBearing = transform_.calculateBearing(x) + joint_angles_->values_[HeadPan]; 
    world_objects_->objects_[beaconId].visionElevation = transform_.calculateElevation(y);
    world_objects_->objects_[beaconId].visionDistance = distance;
    world_objects_->objects_[beaconId].imageCenterX = x;
    world_objects_->objects_[beaconId].imageCenterY = y;
    world_objects_->objects_[beaconId].frameLastSeen = vision_frame_info_->frame_id;
    world_objects_->objects_[beaconId].pixelWidth = pixelWidth;
    world_objects_->objects_[beaconId].partiallyVisible = partiallyVisible;
  }
}

void VisionModule::insertFakeBeacon(int beaconId, int x, int y) {
  setBeaconObject(beaconId, x, y, 1000, 15, 0);
}

