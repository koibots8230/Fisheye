//
// Created by Gillin, Enzo on 2/27/26.
//

#include "main.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <sstream>
#include "Utils.h"
int main() {
    cv::Mat oneRow = objPointsOffset(2, 0.5).reshape(0,1);    // Treat as vector
    std::ostringstream os;
    os << oneRow;                             // Put to the stream
    std::string asStr = os.str();
    std::cout << asStr << std::endl;
    std::vector<int> ids1;
    ids1.push_back(2);
    ids1.push_back(1);
    ids1.push_back(11);
    ids1.push_back(3);
    ids1.push_back(4);

    cv::Mat objPoints = putItAllTogetherNow(ids1).reshape(0,1);    // Treat as vector
    std::ostringstream os2;
    os2 << objPoints;                             // Put to the stream
    std::string asStr2 = os2.str();
    std::cout << asStr2 << std::endl;
    return 0;
}