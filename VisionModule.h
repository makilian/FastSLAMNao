#ifndef VISION_99KDYIX5
#define VISION_99KDYIX5

#include <Module.h>
#include <common/RobotInfo.h>

#include <common/WorldObject.h>
#include <common/ColorSpaces.h>

#include <vision2/TransformationProvider.h>

#include <list>

#define IMAGE_HEIGHT 480
#define IMAGE_WIDTH 640
#define SEG_IMAGE_HEIGHT 120
#define SEG_IMAGE_WIDTH 160
#define MIN_BLOB_PIXELS 20 
#define DIST_THRESHOLD 7
#define OUTPUT 1

#define segAt(x, y) *(robot_vision_->segImg + y * SEG_IMAGE_WIDTH + x)

enum {
  c_UNDEFINED     = 0,
  c_FIELD_GREEN   = 1,
  c_WHITE         = 2,
  c_ORANGE        = 3,
  c_PINK          = 4,
  c_BLUE          = 5,
  c_YELLOW        = 6,
  NUM_COLORS      = 7
};

struct YUV {
  uint8_t y;
  uint8_t u;
  uint8_t y2;
  uint8_t v;
};

enum SegColor {
  sc_UNDEFINED     = 0,
  sc_FIELD_GREEN   = 1,
  sc_WHITE         = 2,
  sc_ORANGE        = 3,
  sc_PINK          = 4,
  sc_BLUE          = 5,
  sc_YELLOW        = 6,
  sc_NUM_COLORS    = 7
};

struct SegCoord {
  SegCoord() {
    r = -1;
    c = -1;
  }

  SegCoord(int r_, int c_) {
    r = r_;
    c = c_;
  }

  SegCoord(const SegCoord& rhs) {
    r = rhs.r;
    c = rhs.c;
  }

  SegCoord& operator=(const SegCoord &rhs) {
    if (this != &rhs) {
      this->r = rhs.r;
      this->c = rhs.c;
    }

    return *this;
  }

  const SegCoord operator^(const SegCoord &rhs) const {
    return SegCoord((this->r + rhs.r) / 2.0, (this->c + rhs.c) / 2.0);
  }

  float dist(const SegCoord &rhs) const {
    return sqrt((this->c - rhs.c) * (this->c - rhs.c) +
                (this->r - rhs.r) * (this->r - rhs.r));
  }

  float r;
  float c;
};

struct RLE {
  RLE() {
    id = -1;
    parentID = -1;
    color = sc_UNDEFINED;
  }

  RLE(const RLE& rhs) {
    this->id = rhs.id;
    this->parentID = rhs.id;
    this->color = rhs.color;
    this->start = rhs.start;
    this->end = rhs.end;
  }

  RLE& operator=(const RLE &rhs) {
    if (this != &rhs) {
      this->id = rhs.id;
      this->parentID = rhs.id;
      this->color = rhs.color;
      this->start = rhs.start;
      this->end = rhs.end;
    }

    return *this;
  }

  int id;
  int parentID;
  SegColor color;
  SegCoord start;
  SegCoord end;
};

struct Blob {
  int id;
  SegColor color;
  SegCoord center;

  SegCoord tl;
  SegCoord tr;
  SegCoord bl;
  SegCoord br;

  float height;
  float width;

  float area;
  int numPixels;
};

typedef std::vector< std::pair<int, std::map<int, RLE> > > RawBlobType;

class FrameInfoBlock;
class JointBlock;
class RobotVisionBlock;
class SensorBlock;
class ImageBlock;
class WorldObjectBlock;
class RobotStateBlock;
class BodyModelBlock;
class CameraBlock;
class RobotInfoBlock;

class VisionModule: public Module {

public:

  // Module specific stuff
  
  void specifyMemoryBlocks();
  void specifyMemoryDependency();
  void initSpecificModule();
  void processFrame();

  FrameInfoBlock *vision_frame_info_;
  JointBlock *joint_angles_;
  RobotVisionBlock *robot_vision_;
  SensorBlock *sensors_;
  ImageBlock *image_;
  WorldObjectBlock *world_objects_;
  RobotStateBlock *robot_state_;
  BodyModelBlock *body_model_;
  CameraBlock *camera_info_;
  RobotInfoBlock *robot_info_;

  bool printTable;
  
  // Some convenience functions to get stuff from memory
  Camera::Type getCameraType();
  bool isBottomCamera();
  float getTrueFrontHeight();
  bool useSimColorTable();
  std::string getDataBase(); // location of data folder
  int getRobotId();
  int getTeamColor();
  Pose3D getCameraMatrix();

  // Vision begins

  VisionModule();
  ~VisionModule();

  static const uint16_t IMAGE_WIDTH_DIV2;
  static const uint16_t IMAGE_WIDTH_DIV4;
  static const uint16_t IMAGE_HEIGHT_DIV2;
  static const uint16_t IMAGE_HEIGHT_DIV4;
  
  // Initialization Functions
  bool loadColorTables();
  bool loadColorTable(Camera::Type camera, const char* fileName, bool fullpath=false);

  // Color Table Stuff
  unsigned char* bottomColorTable;
  unsigned char* topColorTable;
  string bottomColorTableName;
  string topColorTableName;

  void segmentImage();
  Vision::TransformationProvider transform_;

  // Beacons
  void detectBeacons();
  void createRLEs(std::vector< std::list<RLE> > &rleList);
  void mergeRLEs(std::vector< std::list<RLE> > &rleList,
    RawBlobType &blobs);
  void getBlobStats(RawBlobType &raw, std::vector<Blob> &blobs);
  void findBeacons(const std::vector<Blob> &blobs);
  void classifyBeacon(const std::vector<Blob> &blobs, const Blob &x,
    const Blob &y, bool known[]);
  bool isValidBeacon(const std::vector<Blob> &blobs, int beaconID,
    const Blob &top, const Blob &bot);
  void printRLEs(std::vector< std::map<int, RLE> > &rleList);
  void printBlobs(RawBlobType &blobs);
  void printBlobs(std::vector<Blob> &blobs);
  float getDistanceToBeacon(float width);
  void setBeaconObject(int beacondId, int x, int y, float distance, float pixelWidth, bool isCutOff);
  void insertFakeBeacon(int beaconId, int x, int y);

  // Sets
  void setAdd(RawBlobType &blobs, const RLE &elem);
  bool setContains(RawBlobType &blobs, int setID, int elemID);
  void setMerge(RawBlobType &blobs, int dstID, int srcID);
};

#endif /* end of include guard: VISION_99KDYIX5 */
