#include "Utils.h"

#include <vector>
#include <opencv2/core/types.hpp>

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

