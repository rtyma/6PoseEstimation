#include <iostream>
#include "inc/Descriptor.h"
#include "inc/Visualizer.h"
#include "inc/Pose.h"

int main() {

    cv::Mat img_ref=cv::imread("./box_img.jpg",cv::IMREAD_GRAYSCALE);
    Descriptor descriptor("./box_img.jpg");
    cv::resize(img_ref,img_ref,cv::Size(),descriptor.resize_scale,descriptor.resize_scale);

    cv::VideoCapture cap("box.mp4");
    cv::Mat frame;

    std::vector<cv::Point3f> model_points;

    double max_fps=0;
    clock_t start, stop;
    while(cv::waitKey(1)!='a')
    {
        start=clock();

        cap >> frame;

        if(frame.empty())
        {
            break;
        }

        cv::Mat grayscale;
        cv::cvtColor(frame, grayscale, CV_BGR2GRAY);

        cv::resize(grayscale,grayscale,cv::Size(),descriptor.resize_scale,descriptor.resize_scale);

        descriptor.SURF_GPU(grayscale);
        descriptor.Visualizer(grayscale,img_ref);

        stop=clock();
        double fps=1/(((double)(stop-start))/CLOCKS_PER_SEC);
        std::cout<< fps<<std::endl;

        //descriptor.DrawBoundingBox(grayscale);
        if(fps>max_fps)
        {
            max_fps=fps;
        }
    }

    std::vector<cv::Point2f> img_points;
    img_points.push_back(cv::Point2f(359, 391)); // Nose tip
    img_points.push_back(cv::Point2f(399, 561));    // Chin
    img_points.push_back(cv::Point2f(337, 297));    // Left eye left corner
    img_points.push_back(cv::Point2f(513, 301));    // Right eye right corner
    img_points.push_back(cv::Point2f(345, 465));    // Left Mouth corner
    img_points.push_back(cv::Point2f(453, 469));

    Pose pose;
    pose.EstimatePose(img_points,model_points);

    std::cout<<"max fps:"<< max_fps<<std::endl;
    return 0;
}