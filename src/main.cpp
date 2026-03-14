//
// Created by Gillin, Enzo on 2/27/26.
//

#include "main.h"

#include <iostream>
#include <opencv2/core/types.cpp>
#include <ostringstream>
#include "Utils.h"
int main(int argc, char* argv[]) {
    if(argv[1] != nullptr) {
        switch (argv[1]){
            case "Utils": {
                switch (argv[2]) {
                    case "objPointsOffset": {
                        cv::Mat oneRow = objPointsOffset(argv[3], argv[4]).reshape(0,1);    // Treat as vector
                        std::ostringstream os;
                        os << oneRow;                             // Put to the stream
                        std::string asStr = os.str();
                    }
                }
            }
        }
    } else {
        std::cout << "Usage: " << std::endl;
    }
}