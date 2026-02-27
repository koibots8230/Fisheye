#include "Utils.h"
#include <fstream>
#include <vector>
#include <opencv2/core/types.hpp>
#include <json.hpp>

using namespace std;
using namespace cv;

Apriltag::Apriltag(vector<Point2f> corners, int id) {
    this->corners = corners;
    this->id = id;
}

Pose::Pose(Mat tvec, Mat rmat) {
    this->tvec = tvec;
    this->rmat = rmat;
}

CameraThreadset::CameraThreadset(int totalThreads, int maxTagSightings) {
    this->totalThreads = totalThreads;
    this->activeThreads = 0;
    this->tagSightings = 0;
    this->maxTagSightings = maxTagSightings;
    this->lastThreadActivateTime = 0;
}

Mat objPointsOffset(float xOffset, float yOffset) {

    Mat newObjPoints(8, 1, CV_32FC3);
    newObjPoints.ptr<Vec3f>(0)[0] = Vec3f(-tagSizeMeters/2.f, tagSizeMeters/2.f, 0);
    newObjPoints.ptr<Vec3f>(0)[1] = Vec3f(tagSizeMeters/2.f, tagSizeMeters/2.f, 0);
    newObjPoints.ptr<Vec3f>(0)[2] = Vec3f(tagSizeMeters/2.f, -tagSizeMeters/2.f, 0);
    newObjPoints.ptr<Vec3f>(0)[3] = Vec3f(-tagSizeMeters/2.f, -tagSizeMeters/2.f, 0);
    newObjPoints.ptr<Vec3f>(0)[4] = Vec3f((-tagSizeMeters/2.f)+xOffset, (tagSizeMeters/2.f)+yOffset, 0);
    newObjPoints.ptr<Vec3f>(0)[5] = Vec3f((tagSizeMeters/2.f)+xOffset, (tagSizeMeters/2.f)+yOffset, 0);
    newObjPoints.ptr<Vec3f>(0)[6] = Vec3f((tagSizeMeters/2.f)+xOffset, (-tagSizeMeters/2.f)+yOffset, 0);
    newObjPoints.ptr<Vec3f>(0)[7] = Vec3f((-tagSizeMeters/2.f)+xOffset, (-tagSizeMeters/2.f)+yOffset, 0);
    return newObjPoints;
}

vector<int> findPairs(vector<int> ids){
    ifstream aprilTagPairListJsonFile("root/Fisheye/config/apriltagPairs.json");
    nlohmann::json aprilTagPairFullJson = nlohmann::json::parse(aprilTagPairListJsonFile);
    int xOffset2 = aprilTagPairFullJson["AprilTagPairs"][2]["xOffset"];//  create a padded array with the amount of april tags pairs there are total
//  for pairs in json list{
//      if an id given in args is present in the "Tag 1" slot{
//          if an id given in args is present in the "Tag 2" slot{
//              replace the padding at the pair number - 1 slot with a 1
//          }
//      }
//  }
//  for pairs that exist{
//      if the padded array indexed at that pair number is 1{
//          append their pair number (i) to the return vector
//      }
//  }
//  return that return vector
}