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

        solvePnP(objPoints, corners.at(0), cameraMatrix, distCoeffs, rvec, tvec, false, SOLVEPNP_IPPE_SQUARE);
        
        Rodrigues(rvec, rmat);

	rmat = rmat.t();
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

void readJSON(vector<Mat> &cameraMatricies, vector<Mat> &cameraDistCoeffs, vector<String> &camIDs) {
    ifstream camJSON("/root/Fisheye/src/cameras.json");
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

    for (int i = 0; i < 3; i++) {
        VideoCapture cap;
        cap.open(cameraIDs[i]);
        if (cap.isOpened()) {
            cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
            cameras.push_back(cap);
        } else {
            cerr << "ERROR: Camera " << i << " could not be opened!" << endl;
            return -1;
        }
    }

    aruco::DetectorParameters detectParams = aruco::DetectorParameters();
    detectParams.cornerRefinementMethod = aruco::CORNER_REFINE_APRILTAG;
    detectParams.cornerRefinementMaxIterations = 10;
    detectParams.useAruco3Detection = true;
    
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
    outputs.resize(2);
    vector<vector<int>> idsOutput;
    idsOutput.resize(2);
    vector<int64_t> timestamps(2);

    vector<int> cameraSet = {0, 1, 2};
    while(true) {
        outputs = {{{{0}, {0}}}, {{{0}, {0}}}};
        idsOutput = {{0}, {0}};
          
        cameras[cameraSet[0]].read(image1);
        timestamps[0] = nt::Now();
        cameras[cameraSet[1]].read(image2);
        timestamps[1] = nt::Now();

        thread thread1(doVision, image1, cameraMatricies[cameraSet[0]], cameraDistCoeffs[cameraSet[0]], objPoints, ref(detectParams), ref(dict), ref(outputs[0]), ref(idsOutput[0]));

        thread thread2(doVision, image2, cameraMatricies[cameraSet[1]], cameraDistCoeffs[cameraSet[1]], objPoints, ref(detectParams), ref(dict), ref(outputs[1]), ref(idsOutput[1]));

        thread1.join();
        thread2.join();
        
        if(nt::IsConnected(nt::GetDefaultInstance())) {
	    for(int a = 0; a < 2; a++) {
                for(int b = 0; b < outputs[a].size(); b++) {
                    publishers[cameraSet[a]][0].Set(outputs[a][b][0], timestamps[a]);
                    publishers[cameraSet[a]][1].Set(outputs[a][b][1], timestamps[a]);
                    idPublishers[cameraSet[a]].Set(idsOutput[a][b], timestamps[a]);
                }
                cout << timestamps[a] << endl;
            }
        }

        rotate(cameraSet.begin(), cameraSet.begin() + 1, cameraSet.end());
    }

    return 1;
}  