//
// Created by robert on 28.09.17.
//

#include "../inc/Pose.h"

Pose::Pose()
{
    this->default_model_points.push_back(cv::Point3f(0.0f, 0.0f, 0.0f));
    this->default_model_points.push_back(cv::Point3f(0.0f, -30.0f, -65.0f));
    this->default_model_points.push_back(cv::Point3f(-225.0f, 170.0f, -135.0f));
    this->default_model_points.push_back(cv::Point3f(225.0f, 170.0f, -135.0f));
    this->default_model_points.push_back(cv::Point3f(-150.0f, -150.0f, -125.0f));
    this->default_model_points.push_back(cv::Point3f(150.0f, -150.0f, -125.0f));
}

void Pose::EstimatePose(std::vector<cv::Point2f> img_points,std::vector<cv::Point3f> model_points)
{
    if(model_points.empty())
    {
        model_points=this->default_model_points;
    }

    for(auto i=model_points.begin();i!=model_points.end();++i)
        std::cout<<*i<<std::endl;
}