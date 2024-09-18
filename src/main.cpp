#include <algorithm>
#include <opencv2/core/persistence.hpp>
#include <opencv2/videoio.hpp>
#include <thread>
#include <iostream>
#include <fstream>
#include <functional>
#include <array>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include "json.hpp"

#include <ntcore/networktables/NetworkTableInstance.h>
#include <ntcore/networktables/NetworkTable.h>
#include <ntcore/networktables/DoubleArrayTopic.h>
#include <ntcore/networktables/IntegerTopic.h>

#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

using namespace std;
using namespace cv;
using namespace nt;

void findCamPose(Mat frame, Mat cameraMatrix, Mat distCoeffs, Mat objPoints, aruco::DetectorParameters params, aruco::Dictionary dict, vector<vector<vector<double>>>& out, vector<int>& idsOut) {
    aruco::ArucoDetector detector(dict, params);

    vector<int> ids;
    vector<vector<Point2f>> corners;
    vector<vector<Point2f>> rejectedCorners;
    
    Mat rvec(3,1,DataType<double>::type);
    Mat rmat(3,3,DataType<double>::type);
    Mat tvec(3,1,DataType<double>::type);
    
    if (!frame.empty()) {
        detector.detectMarkers(frame, corners, ids, rejectedCorners);
    } else {
        cerr << "WARN: Empty Frame" << endl;
    }
    
    if (ids.size() > 0) {
        out.clear();
    }

    for(int i = 0; i < ids.size(); i++) {
        out.resize(i + 1);
        out[i].resize(2); 

        solvePnPRansac(objPoints, corners.at(i), cameraMatrix, distCoeffs, rvec, tvec, false, 100, 8, 0.99, noArray(), SOLVEPNP_IPPE_SQUARE);

        Rodrigues(rvec, rmat);

        transpose(rmat, rmat);
	    tvec = -rmat * tvec;
 
        for(int a = 0; a < 3; a++) {
            out[i][0].push_back(tvec.at<double>(a));
            for(int b = 0; b < 3; b++) {
                out[i][1].push_back(rmat.at<double>(a, b));
            }
        }
    }
    idsOut = ids;
}

void runThread(VideoCapture camera, Mat cameraMatrix, Mat distCoeffs, Mat objPoints, aruco::DetectorParameters params, aruco::Dictionary dict, vector<DoubleArrayPublisher> vectorPublishers, IntegerPublisher idPublisher) {
    Mat image;

    vector<vector<vector<double>>> vectorOutputs;
    vector<int> idOutputs;

    int64_t timestamp;

    vectorOutputs = {{{0}, {0}}};
    idOutputs = {0};

    while(true) {
        camera.read(image);
        timestamp = nt::Now();

        findCamPose(image, cameraMatrix, distCoeffs, objPoints, ref(params), ref(dict), ref(vectorOutpus), ref(idOutputs));

        if(nt::IsConnected(nt::GetDefaultInstance())) {
            for(int a = 0; a < vectorOutputs.size(); a++) {
                vectorPublishers[0].Set(vectorOutputs[a][0], timestamp);
                vectorPublishers[1].Set(vectorOutputs[a][1], timestamp);
                idPublisher.Set(idOutputs[a], timestamp);
            }
        }
    }
}

void readCameraJSON(vector<Mat> &cameraMatricies, vector<Mat> &cameraDistCoeffs, vector<String> &camIDs, vector<int> &resolution) {
    ifstream camJSON("/root/Fisheye/config/cameras.json");
    nlohmann::json camConfig = nlohmann::json::parse(camJSON);
    for (auto camera : camConfig["Cameras"]) {
        camIDs.push_back(camera["id"]);

        Mat cameraMatrix;
        cameraMatrix = Mat::zeros(3, 3, DataType<double>::type);
    
        cameraMatrix.at<double>(0, 0) = camera["matrix"]["fx"];
        cameraMatrix.at<double>(0, 2) = camera["matrix"]["cx"];
        cameraMatrix.at<double>(1, 1) = camera["matrix"]["fy"];
        cameraMatrix.at<double>(1, 2) = camera["matrix"]["cy"];
        cameraMatrix.at<double>(2, 2) = 1;
 
        cameraMatricies.push_back(cameraMatrix);

        Mat distCoeffs(5,1,DataType<double>::type);
        distCoeffs.at<double>(0) = camera["distCoeffs"]["k1"];
        distCoeffs.at<double>(1) = camera["distCoeffs"]["k2"];
        distCoeffs.at<double>(2) = camera["distCoeffs"]["p1"];
        distCoeffs.at<double>(3) = camera["distCoeffs"]["p2"];
        distCoeffs.at<double>(4) = camera["distCoeffs"]["k3"];

        cameraDistCoeffs.push_back(distCoeffs);
    }
    resolution[0] = camConfig["frameWidth"];
    resolution[1] = camConfig["frameHeight"];
}

int main() {
    //==================Camera Setup==================

    vector<Mat> cameraMatricies;
    vector<Mat> cameraDistCoeffs;
    vector<String> cameraIDs;
    vector<int> resolution(2);

    readCameraJSON(cameraMatricies, cameraDistCoeffs, cameraIDs, resolution);

    cout << resolution[0] << " x " << resolution[1] << endl;

    const int numCameras = cameraIDs.size();

    vector<VideoCapture> cameras;

    for (int i = 0; i < numCameras; i++) {
        VideoCapture cap;
        cap.open(cameraIDs[i]);
        if (cap.isOpened()) {
            cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));

            cap.set(CAP_PROP_FRAME_WIDTH, resolution[0]);
            cap.set(CAP_PROP_FRAME_HEIGHT, resolution[1]);

            cameras.push_back(cap);
        } else {
            cerr << "ERROR: Camera " << i << " could not be opened!" << endl;
            return -1;
        }
    }

    //==================Detector Setup==================
    
    ifstream detectorJSON("/root/Fisheye/config/detector.json");
    nlohmann::json detectorConfig = nlohmann::json::parse(detectorJSON);

    double tagSizeMeters = detectorConfig["tagSizeMeters"];
    
    Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<Vec3f>(0)[0] = Vec3f(-tagSizeMeters/2.f, tagSizeMeters/2.f, 0);
    objPoints.ptr<Vec3f>(0)[1] = Vec3f(tagSizeMeters/2.f, tagSizeMeters/2.f, 0);
    objPoints.ptr<Vec3f>(0)[2] = Vec3f(tagSizeMeters/2.f, -tagSizeMeters/2.f, 0);
    objPoints.ptr<Vec3f>(0)[3] = Vec3f(-tagSizeMeters/2.f, -tagSizeMeters/2.f, 0);

    aruco::DetectorParameters detectParams = aruco::DetectorParameters();

    detectParams.adaptiveThreshWinSizeMin = detectorConfig["adaptiveThreshWinMin"];
    detectParams.adaptiveThreshWinSizeMax = detectorConfig["adaptiveThreshWinMax"];
    detectParams.adaptiveThreshWinSizeStep = detectorConfig["adaptiveThreshWinStep"];

    detectParams.minMarkerPerimeterRate = detectorConfig["minMarkerPerimiterRate"];
    detectParams.maxMarkerPerimeterRate = detectorConfig["maxMarkerPerimiterRate"];

    detectParams.minMarkerDistanceRate = detectorConfig["minMarkerDistanceRate"];

    detectParams.minDistanceToBorder = detectorConfig["minDistanceToBorder"];

    detectParams.perspectiveRemovePixelPerCell = detectorConfig["perspectiveRemovePixelPerCell"];
    detectParams.perspectiveRemoveIgnoredMarginPerCell = detectorConfig["perspectiveRemoveIgnoredMarginPerCell"];

    detectParams.maxErroneousBitsInBorderRate = detectorConfig["maxErroneousBitsInBorderRate"];
    detectParams.errorCorrectionRate = detectorConfig["errorCorrectionRate"];

    detectParams.cornerRefinementMethod = aruco::CORNER_REFINE_APRILTAG;
    detectParams.relativeCornerRefinmentWinSize = detectorConfig["relativeCornerRefinmentWinSize"];
    detectParams.cornerRefinementMaxIterations = detectorConfig["cornerRefinementMaxIterations"];
    detectParams.cornerRefinementMinAccuracy = detectorConfig["cornerRefinementMinAccuracy"];

    detectParams.useAruco3Detection = true;
    
    aruco::Dictionary dict = aruco::getPredefinedDictionary(aruco::DICT_APRILTAG_36h11);

    //==================NetworkTables Setup==================

    ifstream ntJSON("/root/Fisheye/config/networkTables.json");
    nlohmann::json ntConfig = nlohmann::json::parse(ntJSON);

    auto ntInst = NetworkTableInstance::GetDefault();
    auto ntTable = ntInst.GetTable("fisheye");

    vector<vector<DoubleArrayPublisher>> publishers(numCameras);
    vector<IntegerPublisher> idPublishers(numCameras);
    
    auto options = new nt::PubSubOptions();
    options->sendAll = true;
    options->keepDuplicates = true;

    for(int i = 0; i < numCameras; i++) {
        for(int j = 0; j < 2; j++) {
            string temp = string("Cam") + to_string(i + 1) + ((j == 0) ? string("Tvec") : string("Rvec"));
            cout << temp << endl;
            string_view name = temp;
            DoubleArrayTopic vecTopic = ntTable->GetDoubleArrayTopic(name);
            cout << "pub " << i << j << endl; 
            vecTopic.SetPersistent(false);
            vecTopic.SetCached(false);
                
            publishers[i].push_back(vecTopic.Publish(*options));
            cout << publishers[i].size() << endl;
        }
        string temp = string("Cam") + to_string(i + 1) + string("Ids");
        string_view name = temp;
        IntegerTopic idTopic = ntTable->GetIntegerTopic(name);

        idTopic.SetPersistent(false);
        idTopic.SetCached(false);

        idPublishers[i] = idTopic.Publish(*options);
    }
    
    ntInst.StartClient4("fisheye");
    ntInst.SetServerTeam(ntConfig["teamNumber"]);
    
    ntInst.AddConnectionListener(true, [] (const nt::Event& event) {
      if (event.Is(nt::EventFlags::kDisconnected)) {
          NetworkTableInstance::GetDefault().StopClient();
          NetworkTableInstance::GetDefault().StartClient4("fisheye");
      }
    });

    //==================Main Loop Setup==================

    vector<thread> threads(numCameras);

    for (int a = 0; a < numCameras; a++) {
        thread temp(runThread, cameras[a], cameraMatricies[a], cameraDistCoeffs[a], objPoints, detectParams, dict, publishers[a], idPublishers[a]);
        threads.push_back();
    }

    while(true) {}

    return 1;
}  