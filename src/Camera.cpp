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

Camera::Camera(string& id, vector<vector<double>> matrix, vector<double> distortionCoefficents, DoubleArrayPublisher tvecOut,
    DoubleArrayPublisher rmatOut, IntegerPublisher idOut, Mat objectPoints, aruco::DetectorParameters detectParams,
    aruco::Dictionary dict, int totalThreads, int maxTagSightings): threadset(totalThreads, maxTagSightings) {
    camera.open(id);

    this->matrix = Mat::zeros(3, 3, DataType<double>::type);

    for(int a = 0; a < 3; a++) {
        for(int b = 0; b < 3; b++) {
            this->matrix.at<double>(a, b) = matrix[a][b];
        }
    }

    this->distortionCoefficients = Mat::zeros(5, 1, DataType<double>::type);

    for(int a = 0; a < 5; a++) {
        this->distortionCoefficients.at<double>(a) = distortionCoefficents[a];
    }

    this->objectPoints = move(objectPoints);

    this->tvecOut = move(tvecOut);
    this->rmatOut = move(rmatOut);
    this->idOut = move(idOut);

    camMutex = new mutex();
    comMutex = new mutex();
}

vector<Apriltag> Camera::findTags(Mat& image, aruco::ArucoDetector& detector) {
    vector<vector<Point2f>> corners;
    vector<int> ids;
    vector<vector<Point2f>> rejectedCorners;

    detector.detectMarkers(image, corners, ids, rejectedCorners);

    vector<Apriltag> apriltags;
    cout << ids.size() << endl;
    for (int a = 0; a < ids.size(); a++) {
        apriltags.emplace_back(corners[a], ids[a]);
    }

    return apriltags;
}

Pose Camera::findRelativePose(const Apriltag& apriltag) {
    Mat rvec(3,1,DataType<double>::type), tvec(3,1,DataType<double>::type);

    solvePnP(objectPoints, apriltag.corners, matrix, distortionCoefficients,
        rvec, tvec, false, SOLVEPNP_IPPE_SQUARE);

    Mat rmat(3,3,DataType<double>::type);

    Rodrigues(rvec, rmat);

    transpose(rmat, rmat);
    tvec = -rmat * tvec;

    return Pose(tvec, rmat);
}

aruco::ArucoDetector Camera::runIteration(aruco::ArucoDetector detector) {
    cout << "Run iteration called" << endl;
    Mat image;

    unique_lock<mutex> imageLock(*camMutex);
    camera.read(image);
    imageLock.unlock();

    if(image.empty()) {
        cout << "Bad" << endl;
        return detector;
    }

    int64_t timestamp = nt::Now();

    vector<Apriltag> apriltags = findTags(image, detector);

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

    unique_lock<mutex> lock(*comMutex);

    if (!apriltags.empty() && threadset.tagSightings < threadset.maxTagSightings) {
        threadset.tagSightings += 1;
    } else if (apriltags.empty() && threadset.tagSightings != 0) {
        threadset.tagSightings -= 1;
    }

    threadset.activeThreads -= 1;

    lock.unlock();

    return detector;
}
