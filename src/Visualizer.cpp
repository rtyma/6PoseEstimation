//
// Created by robert on 28.09.17.
//

#include "../inc/Visualizer.h"

Visualizer::Visualizer()
{
    this->myWindow=cv::viz::Viz3d("Visualize pose");
    //this->myWindow.setCamera(cv::viz::Camera(10,10,250,250,cv::Size(500,500)));
}

void Visualizer::Visualize(std::string type,cv::Affine3d pose,cv::Point2f center)
{
    if(type=="axis")
    {
        this->myWindow.showWidget(type, cv::viz::WCoordinateSystem());
        this->myWindow.setWidgetPose(type, pose);
    }

    if(type=="dot")
    {
        cv::Point3d c(center.x,center.y,-2);
        this->myWindow.showWidget(type, cv::viz::WSphere(c,2));
    }

    this->myWindow.spinOnce(1, true);
}

void Visualizer::Visualize_points(std::vector<cv::Point2f> points)
{
    cv::Mat img=cv::Mat(500,500, CV_8UC3, cvScalar(0));

    for(int i=0;i<points.size();i++)
    {
        cv::circle(img,cv::Point(10*points[i].x+250,10*points[i].y+250),3,cv::Scalar(0,255,0));
    }

    this->drawArrow(img,cv::Point2i(20,20),cv::Point2i(20,40),cv::Scalar(0,0,255),5,1,CV_AA,0);
    this->drawArrow(img,cv::Point2i(20,20),cv::Point2i(40,20),cv::Scalar(0,255,0),5,1,CV_AA,0);
    this->drawArrow(img,cv::Point2i(20,20),cv::Point2i(40,40),cv::Scalar(255,0,0),5,1,CV_AA,0);

    cv::imshow("point_viz",img);
}

void Visualizer::drawArrow(cv::Mat image, cv::Point2i p, cv::Point2i q, cv::Scalar color, int arrowMagnitude, int thickness, int line_type, int shift)
{
    //Draw the principle line
    cv::line(image, p, q, color, thickness, line_type, shift);
    const double PI = CV_PI;
    //compute the angle alpha
    double angle = atan2((double)p.y-q.y, (double)p.x-q.x);
    //compute the coordinates of the first segment
    p.x = (int) ( q.x +  arrowMagnitude * cos(angle + PI/4));
    p.y = (int) ( q.y +  arrowMagnitude * sin(angle + PI/4));
    //Draw the first segment
    cv::line(image, p, q, color, thickness, line_type, shift);
    //compute the coordinates of the second segment
    p.x = (int) ( q.x +  arrowMagnitude * cos(angle - PI/4));
    p.y = (int) ( q.y +  arrowMagnitude * sin(angle - PI/4));
    //Draw the second segment
    cv::line(image, p, q, color, thickness, line_type, shift);
}