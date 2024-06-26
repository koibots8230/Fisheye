#include <iostream>
#include <thread>
#include <functional>
#include "ntcore/networktables/NetworkTableInstance.h"
#include "ntcore/networktables/NetworkTable.h"
#include "ntcore/networktables/DoubleTopic.h" 
#include "opencv2/videoio.hpp"

using namespace std;
using namespace nt;
using namespace cv;

int main() {
    VideoCapture cap(0, CAP_FFMPEG);
    cout << cap.isOpened();
}
