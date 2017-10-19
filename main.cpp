#include <iostream>
#include "inc/Descriptor.h"
#include "inc/Visualizer.h"
#include "inc/Pose.h"

int main() {

    Descriptor descriptor("./box.png");
    Visualizer visualizer;
    Pose pose(cv::Point2d(0,10),40);

    cv::Mat img_ref=cv::imread("./box.png",cv::IMREAD_GRAYSCALE);
    cv::resize(img_ref,img_ref,cv::Size(),descriptor.resize_scale,descriptor.resize_scale);

    cv::VideoCapture cap("box.mp4");
    cv::Mat frame;

    std::vector<cv::Point3f> model_points;
    std::vector<cv::Point2f> img_points;
    img_points.push_back(cv::Point2f(0, 0));
    img_points.push_back(cv::Point2f(0, 334));
    img_points.push_back(cv::Point2f(486, 334));
    img_points.push_back(cv::Point2f(486, 0));

    int i=180;

    double max_fps=0;
    clock_t start, stop;
    while(cv::waitKey(1)!='q')
    {
        start=clock();

        //cap >> frame;

        frame=cv::imread("./box_in_scene.png");

        cv::Mat rot_mat = cv::getRotationMatrix2D(cv::Point(frame.cols/2, frame.rows/2), i, 1.0);

        cv::Mat warp_img;
        warpAffine(frame, warp_img, rot_mat, frame.size());

        if(frame.empty())
        {
            break;
        }

        cv::Mat grayscale;
        cv::cvtColor(warp_img, grayscale, CV_BGR2GRAY);

        cv::resize(grayscale,grayscale,cv::Size(),descriptor.resize_scale,descriptor.resize_scale);

        descriptor.SURF_GPU(grayscale);
        descriptor.Visualizer(grayscale,img_ref);

        stop=clock();
        double fps=1/(((double)(stop-start))/CLOCKS_PER_SEC);
        std::cout<< fps<<std::endl;

        pose.EstimatePose(img_points,model_points);
        visualizer.Visualize("axis",pose.epose);

        //visualizer.Visualize_points(img_points);

        descriptor.DrawBoundingBox(grayscale);

        std::cout<<descriptor.scene_corners[0]<<" , "<<descriptor.scene_corners[1]<<" , "
                 <<descriptor.scene_corners[2]<<" , "<<descriptor.scene_corners[3]<<" , "<<std::endl;

        img_points=descriptor.scene_corners;

//        switch (cv::waitKey(0))
//        {
//            case 'a':
//            {
//                img_points[0].y = std::fmod(img_points[0].y + 0.2, 50);
//                img_points[1].y = std::fmod(img_points[1].y + 0.0, 50);
//                img_points[2].y = std::fmod(img_points[2].y + 0.2, 50);
//                img_points[3].y = std::fmod(img_points[3].y + 0.0, 50);
//                break;
//            }
//            case 's':
//            {
//                img_points[0].y = std::fmod(img_points[0].y - 0.2, 50);
//                img_points[1].y = std::fmod(img_points[1].y + 0.0, 50);
//                img_points[2].y = std::fmod(img_points[2].y - 0.2, 50);
//                img_points[3].y = std::fmod(img_points[3].y + 0.0, 50);
//                break;
//            }
//            case 'z':
//            {
//                img_points[0].x = std::fmod(img_points[0].x + 0.2, 50);
//                img_points[1].x = std::fmod(img_points[1].x + 0.0, 50);
//                img_points[2].x = std::fmod(img_points[2].x + 0.2, 50);
//                img_points[3].x = std::fmod(img_points[3].x + 0.0, 50);
//                break;
//            }
//            case 'x':
//            {
//                img_points[0].x = std::fmod(img_points[0].x - 0.2, 50);
//                img_points[1].x = std::fmod(img_points[1].x + 0.0, 50);
//                img_points[2].x = std::fmod(img_points[2].x - 0.2, 50);
//                img_points[3].x = std::fmod(img_points[3].x + 0.0, 50);
//                break;
//            }
//            case 'f':
//            {
//                img_points[0].x = std::fmod(img_points[0].x + 0.2, 50);
//                img_points[1].x = std::fmod(img_points[1].x + 0.2, 50);
//                img_points[2].x = std::fmod(img_points[2].x + 0.2, 50);
//                img_points[3].x = std::fmod(img_points[3].x + 0.2, 50);
//                break;
//            }
//        }

        switch (cv::waitKey(0))
        {
            case 'a':
            {
                i++;
                break;
            }
            case 's':
            {
                i--;
                break;
            }
        }

        if(fps>max_fps)
        {
            max_fps=fps;
        }
}
    std::cout<<"max fps:"<< max_fps<<std::endl;

    //std::cout<<"Pose: "<<pose.epose.matrix<<std::endl;
    return 0;
}