#include <algorithm>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <thread>
#include <iostream>
#include <chrono>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include "json.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

using namespace std;
using namespace cv;

void doVision(Mat frame, Mat cameraMatrix, Mat distCoeffs, Mat objPoints) {

    aruco::DetectorParameters detectorParams = aruco::DetectorParameters();

    detectorParams.cornerRefinementMethod = aruco::CORNER_REFINE_APRILTAG;
    detectorParams.cornerRefinementMaxIterations = 5;
    detectorParams.useAruco3Detection = true;

    aruco::Dictionary dict = aruco::getPredefinedDictionary(aruco::DICT_APRILTAG_36h11);
    aruco::ArucoDetector detector(dict, detectorParams);

    vector<int> ids;
    vector<vector<Point2f>> corners;

    Mat rvec(3,1,DataType<double>::type);
    Mat tvec(3,1,DataType<double>::type);

    auto start = chrono::high_resolution_clock::now();
    
    if (!frame.empty()) {
        detector.detectMarkers(frame, corners, ids);

    } else {
        cerr << "WARN: Empty Frame" << endl;
    }

    if(ids.size() > 0) {
        solvePnP(objPoints, corners.at(0), cameraMatrix, distCoeffs, rvec, tvec, false, SOLVEPNP_IPPE_SQUARE);

        cout << "tvec:" << endl <<  tvec << endl;
        cout << "rvec:" << endl <<  rvec << endl;
    }

    auto stop = chrono::high_resolution_clock::now();

    double timeTaken = chrono::duration_cast<chrono::milliseconds>(stop-start).count();

    cout << "Time Taken: " << timeTaken << "ms" << endl;
}

void readJSON(vector<Mat> &cameraMatricies, vector<Mat> &cameraDistCoeffs, vector<String> &camIDs) {
    ifstream camJSON("../src/cameras.json");
    nlohmann::json camData = nlohmann::json::parse(camJSON);
    for (auto camera : camData) {
        if (camera["id"] != "None") {
            camIDs.push_back(camera["id"]);

            Mat cameraMatrix(3,3,DataType<double>::type);
            setIdentity(cameraMatrix);
        
            cameraMatrix.at<double>(0, 0) = camera["matrix"]["fx"];
            cameraMatrix.at<double>(0, 2) = camera["matrix"]["cx"];
            cameraMatrix.at<double>(1, 1) = camera["matrix"]["fy"];
            cameraMatrix.at<double>(1, 2) = camera["matrix"]["cy"];

            cameraMatricies.push_back(cameraMatrix);

            Mat distCoeffs(5,1,DataType<double>::type);
            distCoeffs.at<double>(0) = camera["distCoeffs"]["k1"];
            distCoeffs.at<double>(1) = camera["distCoeffs"]["k2"];
            distCoeffs.at<double>(2) = camera["distCoeffs"]["p1"];
            distCoeffs.at<double>(3) = camera["distCoeffs"]["p2"];
            distCoeffs.at<double>(4) = camera["distCoeffs"]["k3"];

            cameraDistCoeffs.push_back(distCoeffs);
        }
    }
}

int main() {

    Mat image1, image2;

    vector<Mat> cameraMatricies;
    vector<Mat> cameraDistCoeffs;
    vector<String> cameraIDs;

    readJSON(cameraMatricies, cameraDistCoeffs, cameraIDs);

    double tagSizeMeters = 0.17272;

    Mat objPoints(4, 3, DataType<double>::type);

    objPoints.row(0).col(0) = -tagSizeMeters/2;
    objPoints.row(0).col(1) = tagSizeMeters/2;
    objPoints.row(0).col(2) = 0;
    objPoints.row(1).col(0) = tagSizeMeters/2;
    objPoints.row(1).col(1) = tagSizeMeters/2;
    objPoints.row(1).col(2) = 0;
    objPoints.row(2).col(0) = tagSizeMeters/2;
    objPoints.row(2).col(1) = -tagSizeMeters/2;
    objPoints.row(2).col(2) = 0;
    objPoints.row(3).col(0) = -tagSizeMeters/2;
    objPoints.row(3).col(1) = -tagSizeMeters/2;
    objPoints.row(3).col(2) = 0;

    vector<VideoCapture> cameras;

    // TODO: Run w/ 4 cameras
    for (int i = 0; i < cameraIDs.size(); i++) {
        VideoCapture cap(cameraIDs[i]);
        if (cap.isOpened()) {
            cameras.push_back(cap);
        } else {
            cerr << "ERROR: Camera " << i << " could not be opened!" << endl;
            return -1;
        }
    }

    while(true) {
        vector<int> cameraSet = {0, 1, 2, 3};

        cameras[1].read(image2);
        cameras[0].read(image1);

        thread thread1(doVision, image1, cameraMatricies[cameraSet[0]], cameraDistCoeffs[cameraSet[0]], objPoints);

        thread thread2(doVision, image2, cameraMatricies[cameraSet[1]], cameraDistCoeffs[cameraSet[1]], objPoints);

        thread1.join();
        thread2.join();

        // iter_swap(cameraSet.begin(), cameraSet.begin() + 2);
        // iter_swap(cameraSet.begin() + 1, cameraSet.end());
    }

    return 1;
}  