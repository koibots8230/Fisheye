#ifndef CAMERA_H
#define CAMERA_H

#include <string>
#include <vector>
#include <mutex>

#include <opencv2/opencv.hpp>

#include <ntcore/networktables/NetworkTableInstance.h>
#include <ntcore/networktables/DoubleArrayTopic.h>
#include <ntcore/networktables/IntegerTopic.h>

#include "Utils.h"

class Camera {
    public:
        Camera(std::string& id, std::vector<std::vector<double>> matrix, std::vector<double> distortionCoefficents, nt::DoubleArrayPublisher tvecOut,
            nt::DoubleArrayPublisher rmatOut, nt::IntegerPublisher idOut, cv::Mat objectPoints,
            cv::aruco::DetectorParameters detectParams, cv::aruco::Dictionary dictionary, int totalThreads, int maxTagSightings);

        void runIteration(cv::aruco::ArucoDetector detector);

        CameraThreadset threadset;

        std::mutex* camMutex;
        std::mutex* comMutex;

        std::vector<cv::aruco::ArucoDetector> availableDetectors;
    private:
        cv::VideoCapture camera;
        cv::Mat matrix;
        cv::Mat distortionCoefficients;

        cv::Mat objectPoints;

        nt::DoubleArrayPublisher tvecOut;
        nt::DoubleArrayPublisher rmatOut;
        nt::IntegerPublisher idOut;

        std::vector<Apriltag> findTags(cv::Mat& image,cv::aruco::ArucoDetector&);

        Pose findRelativePose(const Apriltag& apriltag);
};



#endif //CAMERA_H
