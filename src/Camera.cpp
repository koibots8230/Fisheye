#include "Camera.h"

#include <string>
#include <fstream>

#include <opencv2/core/matx.hpp>
#include <opencv2/opencv.hpp>
#include <utility>

#include <ntcore/networktables/NetworkTableInstance.h>
#include <ntcore/networktables/DoubleArrayTopic.h>
#include <ntcore/networktables/IntegerTopic.h>
#include "../include/json.hpp"
#include "Utils.h"

using namespace std;
using namespace cv;
using namespace nt;

Camera::Camera(string& id, vector<vector<double>> matrix, vector<double> distortionCoefficents, vector<int> resolution,
    int fps, DoubleArrayPublisher tvecOut, DoubleArrayPublisher rmatOut, IntegerPublisher idOut, Mat objectPoints,
    aruco::DetectorParameters detectParams, aruco::Dictionary dict, int totalThreads, int maxTagSightings):
threadset(totalThreads, maxTagSightings) {
    camera.open(id);

    camera.set(CAP_PROP_FRAME_WIDTH, resolution[0]);
    camera.set(CAP_PROP_FRAME_HEIGHT, resolution[1]);
    camera.set(CAP_PROP_FPS, fps);

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

Pose Camera::findRelativePose(const vector<Point2f> aprilTagCorners, Mat objPoints) {
    Mat rvec(3,1,DataType<double>::type), tvec(3,1,DataType<double>::type);
	if(aprilTagCorners.size() == 4){
    solvePnP(objPoints, aprilTagCorners, matrix, distortionCoefficients,
        rvec, tvec, false, SOLVEPNP_IPPE_SQUARE);
	}else {
		solvePnP(objPoints, aprilTagCorners, matrix, distortionCoefficients,
    	rvec, tvec, false, SOLVEPNP_ITERATIVE);
	}
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
    ifstream pairjson("config/apriltagPairs.json");
    nlohmann::json pairjson_array = nlohmann::json::parse(pairjson);
	vector<int> ids;
	for(int a =0; a < apriltags.size(); a++){
		ids.emplace_back(apriltags[a].id);
	}
	Mat defaultObjPoints(4, 1, CV_32FC3);
    float tagSizeMeters = 0.1651;
	defaultObjPoints.ptr<Vec3f>(0)[0] = Vec3f(-tagSizeMeters/2.f, tagSizeMeters/2.f, 0);
   	defaultObjPoints.ptr<Vec3f>(0)[1] = Vec3f(tagSizeMeters/2.f, tagSizeMeters/2.f, 0);
   	defaultObjPoints.ptr<Vec3f>(0)[2] = Vec3f(tagSizeMeters/2.f, -tagSizeMeters/2.f, 0);
   	defaultObjPoints.ptr<Vec3f>(0)[3] = Vec3f(-tagSizeMeters/2.f, -tagSizeMeters/2.f, 0);
    int pairPresent = findPairs(ids);
    for (const Apriltag& apriltag : apriltags) {
		Pose pose;
        bool pairOrNone = false;
		if (pairPresent != 0){
		if (apriltag.id == pairjson_array["AprilTagPairs"][pairPresent]["Tag 1"]){
			for (int a = 0; a < apriltags.size(); a++){
				if(apriltags[a].id == pairjson_array["AprilTagPairs"][pairPresent]["Tag 2"]){
					vector<Point2f> combined = apriltag.corners;
combined.insert(combined.end(),
                apriltags[a].corners.begin(),
                apriltags[a].corners.end());
					pose = findRelativePose(combined, putItAllTogetherNow(ids));
					pairOrNone = true;
				}
			}}
		}

        if(pairOrNone == false) {
			pose = findRelativePose(apriltag.corners, defaultObjPoints);
		}
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
