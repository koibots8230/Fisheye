#include <algorithm>
#include <fstream>
#include <functional>
#include <future>

#include <opencv2/calib3d.hpp>
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

void setupCameraValues(vector<vector<vector<double>>> &cameraMatricies, vector<vector<double>> &cameraDistCoeffs, vector<String> &camIDs,
    vector<int> &resolution) {
    ifstream camJSON("/root/Fisheye/config/cameras.json");
    nlohmann::json camConfig = nlohmann::json::parse(camJSON);
    for (auto camera : camConfig["Cameras"]) {
        camIDs.push_back(camera["id"]);

        vector<vector<double>> cameraMatrix(3, vector<double>(3, 0));

        cameraMatrix[0][0] = camera["matrix"]["fx"];
        cameraMatrix[0][2] = camera["matrix"]["cx"];
        cameraMatrix[1][1] = camera["matrix"]["fy"];
        cameraMatrix[1][2] = camera["matrix"]["cy"];
        cameraMatrix[2][2] = 1;

        cameraMatricies.push_back(cameraMatrix);

        vector<double> distCoeffs(5);
        distCoeffs[0] = camera["distCoeffs"]["k1"];
        distCoeffs[1] = camera["distCoeffs"]["k2"];
        distCoeffs[2] = camera["distCoeffs"]["p1"];
        distCoeffs[3] = camera["distCoeffs"]["p2"];
        distCoeffs[4] = camera["distCoeffs"]["k3"];

        cameraDistCoeffs.push_back(distCoeffs);
    }
    resolution[0] = camConfig["frameWidth"];
    resolution[1] = camConfig["frameHeight"];
}

aruco::DetectorParameters setupDetectorParameters(nlohmann::json detectorConfig) {
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

    return detectParams;
}

void setupNetworkTables(int numCameras, vector<DoubleArrayPublisher>& tvecPublishers,
    vector<DoubleArrayPublisher>& rmatPublishers, vector<IntegerPublisher>& idPublishers) {
    ifstream ntJSON("/root/Fisheye/config/networkTables.json");
    nlohmann::json ntConfig = nlohmann::json::parse(ntJSON);

    auto ntInst = NetworkTableInstance::GetDefault();
    auto ntTable = ntInst.GetTable("fisheye");

    auto options = new nt::PubSubOptions();
    options->sendAll = true;
    options->keepDuplicates = true;

    for (int i = 0; i < numCameras; i++) {
        DoubleArrayTopic tvecTopic = ntTable->GetDoubleArrayTopic("/camera" + to_string(i) + "/tvec");
        tvecTopic.SetPersistent(false);
        tvecTopic.SetCached(false);
        tvecPublishers.push_back(tvecTopic.Publish(*options));

        DoubleArrayTopic rmatTopic = ntTable->GetDoubleArrayTopic("/camera" + to_string(i) + "/rmat");
        rmatTopic.SetPersistent(false);
        rmatTopic.SetCached(false);
        rmatPublishers.push_back(rmatTopic.Publish(*options));

        IntegerTopic idTopic = ntTable->GetIntegerTopic("/camera" + to_string(i) + "/id");
        idTopic.SetPersistent(false);
        idTopic.SetCached(false);
        idPublishers.push_back(idTopic.Publish(*options));
    }

    ntInst.StartClient4("fisheye");
    ntInst.SetServerTeam(ntConfig["teamNumber"]);

    ntInst.AddConnectionListener(true, [] (const nt::Event& event) {
      if (event.Is(nt::EventFlags::kDisconnected)) {
          NetworkTableInstance::GetDefault().StopClient();
          NetworkTableInstance::GetDefault().StartClient4("fisheye");
      }
    });
}

void caclulatePriority(nlohmann::json threadConfig, vector<Camera>& cameras, vector<int> camsWithPriority) {
    int priorityThreads = threadConfig["totalThreads"].get<int>() - (cameras.size() - camsWithPriority.size()) *
        threadConfig["minThreadsPerCamera"].get<int>();
    for (int i = 0; i < cameras.size(); i++) {
        cameras[i].threadset.totalThreads = (count(camsWithPriority.begin(), camsWithPriority.end(), i) > 0) ?
            priorityThreads / camsWithPriority.size() : (camsWithPriority.size() == 0) ?
                threadConfig["defaultThreadsPerCamera"].get<int>() : threadConfig["minThreadsPerCamera"].get<int>();
    }
    cout << cameras[0].threadset.totalThreads << endl;
	}

int main() {
    vector<vector<vector<double>>> cameraMatricies;
    vector<vector<double>> cameraDistCoeffs;
    vector<String> cameraIDs;
    vector<int> resolution(2);

    setupCameraValues(cameraMatricies, cameraDistCoeffs, cameraIDs, resolution);

    ifstream detectorJSON("/root/Fisheye/config/detector.json");
    nlohmann::json detectorConfig = nlohmann::json::parse(detectorJSON);

    float tagSizeMeters = detectorConfig["tagSizeMeters"];

    Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<Vec3f>(0)[0] = Vec3f(-tagSizeMeters/2.f, tagSizeMeters/2.f, 0);
    objPoints.ptr<Vec3f>(0)[1] = Vec3f(tagSizeMeters/2.f, tagSizeMeters/2.f, 0);
    objPoints.ptr<Vec3f>(0)[2] = Vec3f(tagSizeMeters/2.f, -tagSizeMeters/2.f, 0);
    objPoints.ptr<Vec3f>(0)[3] = Vec3f(-tagSizeMeters/2.f, -tagSizeMeters/2.f, 0);

    aruco::DetectorParameters detectParams = setupDetectorParameters(detectorConfig);

    aruco::Dictionary dict = aruco::getPredefinedDictionary(aruco::DICT_APRILTAG_36h11);

    vector<DoubleArrayPublisher> tvecPublishers;
    vector<DoubleArrayPublisher> rmatPublishers;
    vector<IntegerPublisher> idPublishers;

    setupNetworkTables(cameraIDs.size(), tvecPublishers, rmatPublishers, idPublishers);

    vector<Camera> cameras;

    ifstream threadJSON("/root/Fisheye/config/threading.json");
    nlohmann::json threadConfig = nlohmann::json::parse(threadJSON);

    for (int i = 0; i < cameraIDs.size(); i++) {
        cameras.emplace_back(cameraIDs[i], cameraMatricies[i], cameraDistCoeffs[i],
            std::move(tvecPublishers[i]), std::move(rmatPublishers[i]),
            std::move(idPublishers[i]), objPoints, detectParams, dict, threadConfig["defaultThreadsPerCamera"],
            threadConfig["maxTagSightingsPerCamera"]);
    }

    BS::thread_pool threadPool(threadConfig["totalThreads"]);

    vector<int> camsWithPriority;

    while (true) {
        for (int a = 0; a < cameras.size(); a++) {
            lock_guard<mutex> lock(*cameras[a].comMutex);
            if (cameras[a].threadset.tagSightings >= threadConfig["minTagSightingsForPriority"] &&
                ranges::count(camsWithPriority, a) == 0) {
                camsWithPriority.push_back(a);

                caclulatePriority(threadConfig, cameras, camsWithPriority);
            } else if (cameras[a].threadset.tagSightings < threadConfig["minTagSightingsForPriority"] &&
                ranges::count(camsWithPriority, a) > 0) {
                camsWithPriority.erase(find(camsWithPriority.begin(), camsWithPriority.end(), a));

                caclulatePriority(threadConfig, cameras, camsWithPriority);
            }

            if (cameras[a].threadset.activeThreads != cameras[a].threadset.totalThreads &&
                (nt::Now() - cameras[a].threadset.lastThreadActivateTime) / 1000 >= threadConfig["minThreadOffsetMilliseconds"]) {
                if (cameras[a].availableDetectors.size() != 0) {
                    threadPool.detach_task([&cameras, a]
                        {cameras[a].runIteration(cameras[a].availableDetectors[
                            cameras[a].availableDetectors.size() - 1]);});
                    cameras[a].availableDetectors.erase(cameras[a].availableDetectors.end() - 1);
                    cameras[a].threadset.lastThreadActivateTime = nt::Now();
                } else {
                    aruco::ArucoDetector detector(dict, detectParams);
                    threadPool.detach_task([&cameras, a, detector]
                        {cameras[a].runIteration(detector);});
                    cameras[a].threadset.lastThreadActivateTime = nt::Now();
                }
            }
        }
    }
}
