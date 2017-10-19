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

    void Visualize(std::string type,cv::Affine3d pose,cv::Point2f center=cv::Point2f(0,0));
    void Visualize_points(std::vector<cv::Point2f> points);

private:
    void drawArrow(cv::Mat image, cv::Point2i p, cv::Point2i q, cv::Scalar color, int arrowMagnitude, int thickness, int line_type, int shift);

    cv::viz::Viz3d myWindow;
};


#endif //INC_6POSEESTMATION_VISUALIZER_H
