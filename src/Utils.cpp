#include "Utils.h"
#include <fstream>
#include <vector>
#include <opencv2/core/types.hpp>
#include "../include/json.hpp"

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
    float tagSizeMeters = 0.5;
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

int findPairs(vector<int> ids){
    ifstream aprilTagPairListJsonFile("root/Fisheye/config/apriltagPairs.json");
    nlohmann::json aprilTagPairFullJson = nlohmann::json::parse(aprilTagPairListJsonFile);
//  int xOffset2 = aprilTagPairFullJson["AprilTagPairs"][2]["xOffset"];
	int numOfPairs = sizeof(aprilTagPairFullJson["AprilTagPairs"]);
	// vector<bool> paddedVector;
	// for (int i = 0; i < numOfPairs; i++){
	// 	paddedVector.emplace_back(0);
	// }
    vector<int> pairsPresent;
//  create a padded array with the amount of april tags pairs there are total
	for (int f = 0; f < numOfPairs; f++){//  for pairs in json list{
        for (int u = 0; u < ids.size(); u++){
            if (aprilTagPairFullJson["AprilTagPairs"][i]["Tag 1"] == ids[u]){
                for (int c = 0; c < ids.size(); c++){ 
                    if (aprilTagPairFullJson["AprilTagPairs"][f]["Tag 2"] == ids[c]){
                        pairsPresent.emplace_back(f+1);
                    }
                }
            }
	    }
    }
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
    if(&pairsPresent[0]){
        return pairsPresent[0];
    } else{
        return 0;
    }
}
vector<float> xyOffsetsForGivenPair(int pairNumber){
    ifstream aprilTagPairListJsonFile("root/Fisheye/config/apriltagPairs.json");
    nlohmann::json aprilTagPairFullJson = nlohmann::json::parse(aprilTagPairListJsonFile);
    vector<float> returnVectorII = {aprilTagPairFullJson["AprilTagPairs"][pairNumber]["xOffset"], aprilTagPairFullJson["AprilTagPairs"][pairNumber]["yOffset"]}; //hence starts the shitpost variables :D
    return returnVectorII;
}

Mat putItAllTogetherNow(vector<int> idsII){
    int pairOperating = findPairs(idsII);
    
    if (pairOperating != 0){
        vector<float> Offsets = xyOffsetsForGivenPair(pairOperating);
        return objPointsOffset(Offsets[0], Offsets[1]);
    }
    Mat newObjPoints(8, 1, CV_32FC3);
    float tagSizeMeters = 0.5;

    newObjPoints.ptr<Vec3f>(0)[0] = Vec3f(-tagSizeMeters/2.f, tagSizeMeters/2.f, 0);
    newObjPoints.ptr<Vec3f>(0)[1] = Vec3f(tagSizeMeters/2.f, tagSizeMeters/2.f, 0);
    newObjPoints.ptr<Vec3f>(0)[2] = Vec3f(tagSizeMeters/2.f, -tagSizeMeters/2.f, 0);
    newObjPoints.ptr<Vec3f>(0)[3] = Vec3f(-tagSizeMeters/2.f, -tagSizeMeters/2.f, 0);
    return newObjPoints;
}