#include <iostream>
#include <chrono>

#include "opencv2/opencv.hpp"

extern "C" {
#include "apriltag.h"
#include "tag16h5.h"
#include "apriltag_pose.h"
#include "common/getopt.h"
}

using namespace std;
using namespace cv;

int main() {
    apriltag_family_t *family = tag16h5_create();
    apriltag_detector_t *detector = apriltag_detector_create();
    apriltag_detector_add_family(detector, family);

    detector->quad_decimate = 5; //decimate
    detector->quad_sigma = 0; //blur
    detector->nthreads = 1;
    detector->debug = 1;
    detector->refine_edges = 1;

    Mat frame;
    Mat im;

    apriltag_detection_info_t info;
        info.tagsize = 0.1524;
        info.fx = 618.78479691;
        info.fy = 616.21150452;
        info.cx = 454.81063213;
        info.cy = 238.92290301;
    
    //while(camera.isOpened()) {
        //camera >> frame;
        auto start = chrono::high_resolution_clock::now();

        frame = imread("/home/koibots/Vision/main/src/apriltag.jpg", IMREAD_GRAYSCALE);
        resize(frame, im, Size(854, 480));
        image_u8_t image = { 
            .width = im.cols,
            .height = im.rows,
            .stride = im.cols,
            .buf = im.data
        };

        zarray_t *detections = apriltag_detector_detect(detector, &image);

        for (int count = 0; count < zarray_size(detections); ++count) {
            apriltag_detection_t *detection;
            zarray_get(detections, count, &detection);

            info.det = detection;

            apriltag_pose_t pose;

            estimate_pose_for_tag_homography(&info, &pose);

            cout << "Rotation Matrix:" << endl << matd_get(pose.R, 0, 0) << ", " << matd_get(pose.R, 0, 1) << ", " << matd_get(pose.R, 0, 2) << endl;
            cout << matd_get(pose.R, 1, 0) << ", " << matd_get(pose.R, 1, 1) << ", " << matd_get(pose.R, 1, 2) << endl;
            cout << matd_get(pose.R, 2, 0) << ", " << matd_get(pose.R, 2, 1) << ", " << matd_get(pose.R, 2, 2) << endl;
            cout << "Pose: " << endl << matd_get(pose.t, 0, 0) << ", " << matd_get(pose.t, 0, 1) << ", " << matd_get(pose.t, 0, 2) << endl;
        }

        auto stop = chrono::high_resolution_clock::now();

        double timeTaken = chrono::duration_cast<chrono::milliseconds>(stop-start).count();

        cout << "Time Taken: " << timeTaken << "ms" << endl;

    apriltag_detector_destroy(detector);
    tag16h5_destroy(family);
    
    return 0;
}