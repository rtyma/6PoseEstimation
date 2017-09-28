//
// Created by robert on 28.09.17.
//

#include "../inc/Visualizer.h"

Visualizer::Visualizer()
{
    this->myWindow=cv::viz::Viz3d("Visualizer");
}

void Visualizer::Visualize(std::string type,cv::Affine3d pose)
{
    if(type=="axis")
    {
        this->myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
        this->myWindow.setWidgetPose("Coordinate Widget", pose);
    }

//    //myWindow.setWidgetPose("Cube Widget", pose);
    this->myWindow.spinOnce(1, true);
}
