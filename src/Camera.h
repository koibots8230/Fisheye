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
        Camera(const std::string& id, cv::Mat matrix, cv::Mat distortionCoefficents, nt::DoubleArrayPublisher tvecOut,
            nt::DoubleArrayPublisher rmatOut, nt::IntegerPublisher idOut, cv::Mat objectPoints,
            cv::aruco::DetectorParameters detectParams, cv::aruco::Dictionary dictionary, int totalThreads, int maxTagSightings);

        void runIteration();

        CameraThreadset threadset;

        std::mutex* camMutex;
    private:
        cv::VideoCapture camera;
        cv::Mat matrix;
        cv::Mat distortionCoefficients;

        cv::Mat objectPoints;

        cv::aruco::ArucoDetector detector;

        nt::DoubleArrayPublisher tvecOut;
        nt::DoubleArrayPublisher rmatOut;
        nt::IntegerPublisher idOut;

        std::vector<Apriltag> findTags(const cv::Mat& image) const;

        Pose findRelativePose(const Apriltag& apriltag) const;
};



#endif //CAMERA_H
