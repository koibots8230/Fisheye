#ifndef UTILS_H
#define UTILS_H
#include <vector>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

struct Apriltag {
    std::vector<cv::Point2f> corners;
    int id;

    Apriltag(std::vector<cv::Point2f> corners, int id);
};

struct Pose {
    cv::Mat tvec;
    cv::Mat rmat;

    Pose(cv::Mat tvec, cv::Mat rmat);
};

struct CameraThreadset {
    int totalThreads;
    int activeThreads;
    int tagSightings;
    int maxTagSightings;
    int64_t lastThreadActivateTime;

    CameraThreadset(int totalThreads, int maxTagSightings);
};

#endif //UTILS_H
