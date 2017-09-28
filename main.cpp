#include <iostream>
#include "inc/Descriptor.h"
#include "inc/Visualizer.h"
#include "inc/Pose.h"

int main() {

    Descriptor descriptor("./box_img.jpg");
    Visualizer visualizer;
    Pose pose(cv::Point2d(0,10),40);

    cv::Mat img_ref=cv::imread("./box_img.jpg",cv::IMREAD_GRAYSCALE);
    cv::resize(img_ref,img_ref,cv::Size(),descriptor.resize_scale,descriptor.resize_scale);

    cv::VideoCapture cap("box.mp4");
    cv::Mat frame;

    std::vector<cv::Point3f> model_points;
    std::vector<cv::Point2f> img_points;
    img_points.push_back(cv::Point2f(0, 0));
    img_points.push_back(cv::Point2f(0, 30));
    img_points.push_back(cv::Point2f(30, 0));
    img_points.push_back(cv::Point2f(30, 30));
    //img_points.push_back(cv::Point2f(345, 465));
    //img_points.push_back(cv::Point2f(453, 469));

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

        img_points[5].x=std::fmod(img_points[5].x+1,500);
        pose.EstimatePose(img_points,model_points);
        visualizer.Visualize("axis",pose.epose);

        //descriptor.DrawBoundingBox(grayscale);
        if(fps>max_fps)
        {
            max_fps=fps;
        }
    }

    std::cout<<"max fps:"<< max_fps<<std::endl;

    //std::cout<<"Pose: "<<pose.epose.matrix<<std::endl;
    return 0;
}