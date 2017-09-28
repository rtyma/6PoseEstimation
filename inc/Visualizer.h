//
// Created by robert on 28.09.17.
//

#ifndef INC_6POSEESTMATION_VISUALIZER_H
#define INC_6POSEESTMATION_VISUALIZER_H

#include <string>
#include "opencv2/opencv.hpp"

class Visualizer {

public:
    Visualizer();

    void Visualize(std::string type);

private:
    cv::viz::Viz3d myWindow;
};


#endif //INC_6POSEESTMATION_VISUALIZER_H
