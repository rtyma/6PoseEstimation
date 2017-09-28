//
// Created by robert on 28.09.17.
//

#include "../inc/Visualizer.h"

Visualizer::Visualizer()
{
    this->myWindow=cv::viz::Viz3d("Visualizer");
}

void Visualizer::Visualize(std::string type)
{
    std::vector<cv::Point3f> model_points;

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3)
             << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);

    cv::Mat rotation_vector;
    cv::Mat translation_vector;

    cv::solvePnP(model_points, img_points, camera_matrix, cv::Mat::zeros(4, 1, cv::DataType<double>::type),
                     rotation_vector, translation_vector);

    myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());

    cv::Mat rot_mat;
    cv::Rodrigues(rotation_vector, rot_mat);

    cv::Affine3d pose = cv::Affine3d(rot_mat, cv::Vec3f(0, 0, 0));

    myWindow.setWidgetPose("Coordinate Widget", pose);
    //myWindow.setWidgetPose("Cube Widget", pose);
    myWindow.spinOnce(1, true);
}
