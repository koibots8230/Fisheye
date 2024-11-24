#include "Camera.h"

#include <string>

#include <opencv2/core/matx.hpp>
#include <opencv2/opencv.hpp>
#include <utility>

#include <ntcore/networktables/NetworkTableInstance.h>
#include <ntcore/networktables/DoubleArrayTopic.h>
#include <ntcore/networktables/IntegerTopic.h>

#include "Utils.h"

using namespace std;
using namespace cv;
using namespace nt;

Camera::Camera(const string& id, Mat matrix, Mat distortionCoefficents, DoubleArrayPublisher tvecOut,
    DoubleArrayPublisher rmatOut, IntegerPublisher idOut, Mat objectPoints, aruco::DetectorParameters detectParams,
    aruco::Dictionary dict, int totalThreads, int maxTagSightings): threadset(totalThreads, maxTagSightings) {
    camera.open(id);

    this->matrix = move(matrix);
    this->distortionCoefficients = move(distortionCoefficients);

    this->objectPoints = move(objectPoints);

    detector = aruco::ArucoDetector(dict, detectParams);

    this->tvecOut = move(tvecOut);
    this->rmatOut = move(rmatOut);
    this->idOut = move(idOut);

    camMutex = new mutex();
}

vector<Apriltag> Camera::findTags(const Mat& image) const {
    vector<vector<Point2f>> corners;
    vector<int> ids;
    vector<vector<Point2f>> rejectedCorners;

    detector.detectMarkers(image, corners, ids, rejectedCorners);

    vector<Apriltag> apriltags;

    for (int a = 0; a < ids.size(); a++) {
        apriltags.emplace_back(corners[a], ids[a]);
    }

    return apriltags;
}

Pose Camera::findRelativePose(const Apriltag& apriltag) const {
    Mat rvec(3,1,DataType<double>::type), tvec(3,1,DataType<double>::type);

    solvePnP(objectPoints, apriltag.corners, matrix, distortionCoefficients,
        rvec, tvec, false, SOLVEPNP_IPPE_SQUARE);

    Mat rmat(3,3,DataType<double>::type);

    Rodrigues(rvec, rmat);

    transpose(rmat, rmat);
    tvec = -rmat * tvec;

    return Pose(tvec, rmat);
}

void Camera::runIteration() {
    Mat image;
    camera.read(image);
    int64_t timestamp = nt::Now();

    vector<Apriltag> apriltags = findTags(image);

    for (const Apriltag& apriltag : apriltags) {
        Pose pose = findRelativePose(apriltag);

        vector<double> tvec;
        vector<double> rmat;

        for(int a = 0; a < 3; a++) {
            tvec.push_back(pose.tvec.at<double>(a));
            for(int b = 0; b < 3; b++) {
                rmat.push_back(pose.rmat.at<double>(a, b));
            }
        }

        tvecOut.Set(tvec, timestamp);
        rmatOut.Set(rmat, timestamp);
        idOut.Set(apriltag.id, timestamp);
    }

    lock_guard<mutex> lock(*camMutex);

    if (apriltags.size() > 0 && threadset.tagSightings < threadset.maxTagSightings) {
        threadset.tagSightings += 1;
    } else if (apriltags.empty()) {
        threadset.tagSightings -= 1;
    }

    threadset.activeThreads -= 1;
}
