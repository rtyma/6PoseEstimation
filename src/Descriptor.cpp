//
// Created by robert on 21.09.17.
//

#include <iostream>
#include "../inc/Descriptor.h"

Descriptor::Descriptor(std::string img_path)
{
    cv::Mat img_temp=cv::imread( img_path, cv::IMREAD_GRAYSCALE );
    cv::resize(img_temp,img_temp,cv::Size(),this->resize_scale,this->resize_scale);
    this->img_ref.upload(img_temp);

    cv::cuda::printCudaDeviceInfo(cv::cuda::getDevice());
    cv::cuda::setDevice(0);
}

void Descriptor::SURF_GPU(cv::Mat scene_img)
{
    this->keypoints_1.clear();
    this->keypoints_2.clear();
    this->good_matches.clear();

    this->img_scene.upload(scene_img);

    cv::cuda::SURF_CUDA surf;

    cv::cuda::GpuMat keypoints1GPU, keypoints2GPU;
    cv::cuda::GpuMat descriptors1GPU, descriptors2GPU;

    surf(this->img_ref, cv::cuda::GpuMat(), keypoints1GPU, descriptors1GPU);
    surf(this->img_scene, cv::cuda::GpuMat(), keypoints2GPU, descriptors2GPU);

    cv::Ptr<cv::cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(surf.defaultNorm());
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors1GPU, descriptors2GPU, matches);

    std::vector<float> descriptors_1, descriptors_2;

    surf.downloadKeypoints(keypoints1GPU, this->keypoints_1);
    surf.downloadKeypoints(keypoints2GPU, this->keypoints_2);

    Ratio_test(descriptors1GPU.rows,matches);
}

void Descriptor::Ratio_test(int size,std::vector< cv::DMatch > matches)
{
    double max_dist = 0; double min_dist = 1000;

    for( int i = 0; i < size; i++ )
    { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    for( int i = 0; i < size; i++ )
    { if( matches[i].distance <= std::max(2*min_dist, 0.001) )
        { this->good_matches.push_back( matches[i]);
        }
    }
}

void Descriptor::Visualizer(cv::Mat scene_img,cv::Mat ref_img)
{
    cv::Mat img_matches;
    drawMatches( ref_img, this->keypoints_1, scene_img, this->keypoints_2,
                 this->good_matches, img_matches, cv::Scalar::all(255), cv::Scalar::all(255),
                 std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    cv::imshow("test",img_matches);
}

void Descriptor::DrawBoundingBox(cv::Mat img)
{
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;

    for (int i = 0; i < this->good_matches.size(); i++) {
        obj.push_back(this->keypoints_1[good_matches[i].queryIdx].pt);
        scene.push_back(this->keypoints_2[good_matches[i].trainIdx].pt);
    }

    cv::Mat H;
    if(!obj.empty()&&!scene.empty()) {
        H = cv::findHomography(obj, scene, CV_RANSAC);
    }

    std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0]=(cv::Point2f(107,158));
    obj_corners[1]=(cv::Point2f(303,287));
    obj_corners[2]=(cv::Point2f(487,191));
    obj_corners[3]=(cv::Point2f(288,75));

    std::vector<cv::Point2f> scene_corners(4);

    if(!H.empty()) {
        cv::perspectiveTransform(obj_corners, scene_corners, H);
    }

    cv::Mat frame;
    cv::cvtColor(img,frame,CV_GRAY2BGR);

    line( frame, scene_corners[0] , scene_corners[1] , cv::Scalar(0, 255, 0), 4 );
    line( frame, scene_corners[1] , scene_corners[2] , cv::Scalar( 0, 255, 0), 4 );
    line( frame, scene_corners[2] , scene_corners[3] , cv::Scalar( 0, 255, 0), 4 );
    line( frame, scene_corners[3] , scene_corners[0] , cv::Scalar( 0, 255, 0), 4 );

    imshow( "frame", frame);
}