//
// Created by robert on 28.09.17.
//

#ifndef INC_6POSEESTMATION_POSE_H
#define INC_6POSEESTMATION_POSE_H

#include "opencv2/opencv.hpp"

class Pose {

public:

    Pose(cv::Point2d center,double focal);

    void EstimatePose(std::vector<cv::Point2f> img_points,std::vector<cv::Point3f> model_points);

    cv::Affine3d epose;

private:

    std::vector<cv::Point3f> default_model_points;
    cv::Mat camera_matrix;
};

#endif //INC_6POSEESTMATION_POSE_H
