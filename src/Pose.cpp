//
// Created by robert on 28.09.17.
//

#include "../inc/Pose.h"

Pose::Pose(cv::Point2d center,double focal)
{
    this->default_model_points.push_back(cv::Point3f(0.0f, 0.0f, 0.0f));
    this->default_model_points.push_back(cv::Point3f(0.0f, 30.0f, 0.0f));
    this->default_model_points.push_back(cv::Point3f(30.0f, 0.0f, 0.0f));
    this->default_model_points.push_back(cv::Point3f(30.0f, 30.0f, 0.0f));
    //this->default_model_points.push_back(cv::Point3f(-150.0f, -150.0f, -125.0f));
    //this->default_model_points.push_back(cv::Point3f(150.0f, -150.0f, -125.0f));

    this->camera_matrix=(cv::Mat_<double>(3, 3)
            << focal, 0, center.x, 0, focal, center.y, 0, 0, 1);
}

void Pose::EstimatePose(std::vector<cv::Point2f> img_points,std::vector<cv::Point3f> model_points)
{
    if(model_points.empty())
    {
        model_points=this->default_model_points;
    }

//    for(auto i=model_points.begin();i!=model_points.end();++i)
//        std::cout<<*i<<std::endl;

    cv::Mat rotation_vector;
    cv::Mat translation_vector;

    cv::solvePnP(model_points, img_points, this->camera_matrix, cv::Mat::zeros(4, 1, cv::DataType<double>::type),
                 rotation_vector, translation_vector);

    cv::Mat rot_mat;
    cv::Rodrigues(rotation_vector, rot_mat);

    this->epose = cv::Affine3d(rot_mat, cv::Vec3f(0, 0, 0));
}