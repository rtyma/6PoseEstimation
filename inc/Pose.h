//
// Created by robert on 28.09.17.
//

#ifndef INC_6POSEESTMATION_POSE_H
#define INC_6POSEESTMATION_POSE_H

#include "opencv2/opencv.hpp"

class Pose {

public:

    Pose();

    void EstimatePose(std::vector<cv::Point2f> img_points,std::vector<cv::Point3f> model_points);

private:

    cv::Affine3d pose;
    std::vector<cv::Point3f> default_model_points;
};


#endif //INC_6POSEESTMATION_POSE_H
