//
// Created by robert on 21.09.17.
//

#ifndef INC_6POSEESTMATION_DESCRIPTOR_H
#define INC_6POSEESTMATION_DESCRIPTOR_H

#include <opencv/cv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/cudafeatures2d.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"
#include <opencv2/cudaimgproc.hpp>

class Descriptor {

public:
    Descriptor(std::string img_path);
    Descriptor(cv::Mat img);

    void SURF_GPU(cv::Mat scene_img);
    void Visualizer(cv::Mat scene_img,cv::Mat ref_img);
    void DrawBoundingBox(cv::Mat img);

    float resize_scale=1;
    std::vector<cv::Point2f> point_list_2d;
    std::vector<cv::Point2f> scene_corners;

private:

    void Ratio_test(int size,std::vector< cv::DMatch > matches);
    void create_2dpoint_list();

    cv::cuda::GpuMat img_ref;
    cv::cuda::GpuMat img_scene;

    std::vector< cv::KeyPoint > keypoints_1;
    std::vector< cv::KeyPoint > keypoints_2;
    std::vector< cv::DMatch > good_matches;
};


#endif //INC_6POSEESTMATION_DESCRIPTOR_H
