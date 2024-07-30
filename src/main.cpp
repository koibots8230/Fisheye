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

void doVision(Mat frame, Mat cameraMatrix, Mat distCoeffs, Mat objPoints, aruco::DetectorParameters params, aruco::Dictionary dict, vector<vector<vector<double>>>& out, vector<int>& idsOut) {
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
        //cout << tvec.at<double>(0) << ", " << tvec.at<double>(1) << ", " << tvec.at<double>(2) << endl;
        Rodrigues(rvec, rmat);
        //cout << atan2(rmat.at<double>(2, 1), rmat.at<double>(2, 2)) << endl; 
        transpose(rmat, rmat);
	tvec = -rmat * tvec;
        //cout << atan2(rmat.at<double>(2, 1), rmat.at<double>(2, 2)) << endl;

        //cout << "after: " << tvec.at<double>(0) << ", " << tvec.at<double>(1) << ", " << tvec.at<double>(2) << endl; 
        for(int a = 0; a < 3; a++) {
            out[i][0].push_back(tvec.at<double>(a));
            for(int b = 0; b < 3; b++) {
                out[i][1].push_back(rmat.at<double>(a, b));
            }
        }
    }
    idsOut = ids;
}

void readJSON(vector<Mat> &cameraMatricies, vector<Mat> &cameraDistCoeffs, vector<String> &camIDs) {
    ifstream camJSON("/root/Fisheye/src/cameras.json");
    nlohmann::json camData = nlohmann::json::parse(camJSON);
    for (auto camera : camData) {
        if (camera["id"] != "None") {
            camIDs.push_back(camera["id"]);

            Mat cameraMatrix;
            cameraMatrix = Mat::zeros(3, 3, DataType<double>::type);
        
            cameraMatrix.at<double>(0, 0) = camera["matrix"]["fx"];
            cameraMatrix.at<double>(0, 2) = camera["matrix"]["cx"];
            cameraMatrix.at<double>(1, 1) = camera["matrix"]["fy"];
            cameraMatrix.at<double>(1, 2) = camera["matrix"]["cy"];
            cameraMatrix.at<double>(2, 2) = 1;
            for (int a = 0; a < 3; a++) {
            for (int b = 0; b < 3; b++) {
            cout << cameraMatrix.at<double>(a, b) << " ";
            }
            cout << endl;}cout << endl; 
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
    Mat image1, image2, image3;

    vector<Mat> cameraMatricies;
    vector<Mat> cameraDistCoeffs;
    vector<String> cameraIDs;

    readJSON(cameraMatricies, cameraDistCoeffs, cameraIDs);
    cout << cameraDistCoeffs[0].at<double>(0) << endl;
    double tagSizeMeters = 0.1651;
    
    Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<Vec3f>(0)[0] = Vec3f(-tagSizeMeters/2.f, tagSizeMeters/2.f, 0);
    objPoints.ptr<Vec3f>(0)[1] = Vec3f(tagSizeMeters/2.f, tagSizeMeters/2.f, 0);
    objPoints.ptr<Vec3f>(0)[2] = Vec3f(tagSizeMeters/2.f, -tagSizeMeters/2.f, 0);
    objPoints.ptr<Vec3f>(0)[3] = Vec3f(-tagSizeMeters/2.f, -tagSizeMeters/2.f, 0);

    vector<VideoCapture> cameras;

    for (int i = 0; i < 3; i++) {
        VideoCapture cap;
        cap.open(cameraIDs[i]);
        if (cap.isOpened()) {
            cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
            cap.set(CAP_PROP_FRAME_WIDTH, 1280);
            cap.set(CAP_PROP_FRAME_HEIGHT, 720);
            cout << cap.get(CAP_PROP_FRAME_WIDTH) << ", " << cap.get(CAP_PROP_FRAME_HEIGHT) << endl;
            cameras.push_back(cap);
        } else {
            cerr << "ERROR: Camera " << i << " could not be opened!" << endl;
            return -1;
        }
    }

    aruco::DetectorParameters detectParams = aruco::DetectorParameters();
    detectParams.cornerRefinementMethod = aruco::CORNER_REFINE_APRILTAG;
    detectParams.cornerRefinementMaxIterations = 20;
    detectParams.useAruco3Detection = true;
    detectParams.aprilTagDeglitch = 10;
    
    aruco::Dictionary dict = aruco::getPredefinedDictionary(aruco::DICT_APRILTAG_36h11);

    auto ntInst = NetworkTableInstance::GetDefault();
    auto ntTable = ntInst.GetTable("fisheye");

    DoubleArrayPublisher publishers[4][3];
    IntegerPublisher idPublishers[4];

    ifstream ntJSON("/root/Fisheye/src/networkTables.json");
    nlohmann::json ntData = nlohmann::json::parse(ntJSON);
    
    auto options = new nt::PubSubOptions();
    options->sendAll = true;
    options->keepDuplicates = true;

    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 2; j++) {
            string thing = ntData["topicNames"][to_string(i)][to_string(j)];
            string_view thing2 = thing;
            DoubleArrayTopic vecTopic = ntTable->GetDoubleArrayTopic(thing2);
            vecTopic.SetPersistent(false);
            vecTopic.SetCached(false);
            publishers[i][j] = vecTopic.Publish(*options);
            cout << "Pub " << i << j << " has been made" << endl;
        }
        string thing = ntData["topicNames"][to_string(i)][to_string(2)];
        string_view thing2 = thing;
        IntegerTopic idTopic = ntTable->GetIntegerTopic(thing2);
        idTopic.SetPersistent(false);
        idTopic.SetCached(false);
        idPublishers[i] = idTopic.Publish(*options);
        cout << "Pub " << i << 2 << " has been made" << endl; 
    }
    
    ntInst.StartClient4("fisheye");
    ntInst.SetServerTeam(8230);
    
    ntInst.AddConnectionListener(true, [] (const nt::Event& event) {
      if (event.Is(nt::EventFlags::kDisconnected)) {
          NetworkTableInstance::GetDefault().StopClient();
          NetworkTableInstance::GetDefault().StartClient4("fisheye");
      }
    });

    vector<vector<vector<vector<double>>>> outputs;
    outputs.resize(3);
    vector<vector<int>> idsOutput;
    idsOutput.resize(3);
    vector<int64_t> timestamps(3);

    while(true) {
        outputs = {{{{0}, {0}}}, {{{0}, {0}}}, {{{0}, {0}}}};
        idsOutput = {{0}, {0}, {0}};
  
        cameras[0].read(image1);
        timestamps[0] = nt::Now();
        cameras[1].read(image2);
        timestamps[1] = nt::Now();
        cameras[2].read(image3);
        timestamps[2] = nt::Now();

        thread thread1(doVision, image1, cameraMatricies[0], cameraDistCoeffs[0], objPoints, ref(detectParams), ref(dict), ref(outputs[0]), ref(idsOutput[0]));

        thread thread2(doVision, image2, cameraMatricies[1], cameraDistCoeffs[1], objPoints, ref(detectParams), ref(dict), ref(outputs[1]), ref(idsOutput[1]));
        
        thread thread3(doVision, image3, cameraMatricies[2], cameraDistCoeffs[2], objPoints, ref(detectParams), ref(dict), ref(outputs[2]), ref(idsOutput[2]));
        
        thread1.join();
        thread2.join();
        thread3.join();

        if(nt::IsConnected(nt::GetDefaultInstance())) {
	    for(int a = 0; a < 3; a++) {
                for(int b = 0; b < outputs[a].size(); b++) {
                    publishers[a][0].Set(outputs[a][b][0], timestamps[a]);
                    publishers[a][1].Set(outputs[a][b][1], timestamps[a]);
                    idPublishers[a].Set(idsOutput[a][b], timestamps[a]);
                }
            }
        }
    }

    return 1;
}  