#include <algorithm>
#include <fstream>
#include <functional>
#include <future>

#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include "../include/json.hpp"

#include "../include/BS_thread_pool.hpp"

#include "Camera.h"

using namespace cv;
using namespace std;
using namespace nt;

Mat objPointsOffset(float tagSizeMeters, float xOffset, float yOffset) {
    Mat newObjPoints(8, 1, CV_32FC3);
    newObjPoints.ptr<Vec3f>(0)[0] = Vec3f(-tagSizeMeters/2.f, tagSizeMeters/2.f, 0);
    newObjPoints.ptr<Vec3f>(0)[1] = Vec3f(tagSizeMeters/2.f, tagSizeMeters/2.f, 0);
    newObjPoints.ptr<Vec3f>(0)[2] = Vec3f(tagSizeMeters/2.f, -tagSizeMeters/2.f, 0);
    newObjPoints.ptr<Vec3f>(0)[3] = Vec3f(-tagSizeMeters/2.f, -tagSizeMeters/2.f, 0);
    newObjPoints.ptr<Vec3f>(0)[0] = Vec3f((-tagSizeMeters/2.f)+xOffset, (tagSizeMeters/2.f)+yOffset, 0);
    newObjPoints.ptr<Vec3f>(0)[1] = Vec3f((tagSizeMeters/2.f)+xOffset, (tagSizeMeters/2.f)+yOffset, 0);
    newObjPoints.ptr<Vec3f>(0)[2] = Vec3f((tagSizeMeters/2.f)+xOffset, (-tagSizeMeters/2.f)+yOffset, 0);
    newObjPoints.ptr<Vec3f>(0)[3] = Vec3f((-tagSizeMeters/2.f)+xOffset, (-tagSizeMeters/2.f)+yOffset, 0);
    return newObjPoints;
}
int main() {
    ifstream aprilTagJSON("/root/Fisheye/config/apriltagPairs.json");
    nlohmann::json aprilTagPairs = nlohmann::json::parse(aprilTagJSON);

}