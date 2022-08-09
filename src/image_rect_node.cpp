//
// Created by Yuchen WU on 5/23/20.
//

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "std_msgs/String.h"
#include <opencv2/core/eigen.hpp>
// #include <opencv4/opencv2/cudawarping.hpp>
#include "camodocal/camera_models/CameraFactory.h"

cv::Mat rect_map1_, rect_map2_, R_rect_cv_;
image_transport::Publisher img_pub;
ros::Publisher info_pub;
sensor_msgs::CameraInfo info;
double scale;
static tf::TransformBroadcaster * br;

void image_callback(const sensor_msgs::ImageConstPtr im_msg) {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(im_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat undistort;

    cv::remap(cv_ptr->image, undistort, rect_map1_, rect_map2_, cv::INTER_CUBIC);
    // cv::resize(undistort,undistort,cv::Size(),scale,scale);
    // cv::Rect myROI(undistort.cols/2-2448/2, undistort.rows/2-1840/2, 2448, 1840);
    // cv::Mat croppedImage = undistort(myROI);

    cv_bridge::CvImagePtr cv_ptr2(new cv_bridge::CvImage);

    cv_ptr2->encoding = "bgr8";
    cv_ptr2->header.stamp = ros::Time::now();
    cv_ptr2->header.frame_id = cv_ptr->header.frame_id;
    cv_ptr2->image = undistort;
    img_pub.publish(cv_ptr2->toImageMsg());

    info.header.stamp = cv_ptr2->header.stamp;
    info_pub.publish(info);
    //std::cout<<ros::Time::now()-im_msg->header.stamp<<std::endl;

    // tf::Vector3 t(0,0,0.126);
    // tf::Quaternion q(0,-1,0,0);
    // tf::Transform transform;
    // transform.setOrigin(t);
    // transform.setRotation(q);
    // br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/tool0_controller", "/tag_5_gt"));

    // tf::Vector3 t2(-0.04407151,0.0005267,0.04110016);
    // tf::Quaternion q2(-0.13571394,0.12126001,-0.69215282,0.69842839);
    // tf::Transform transform2;
    // transform2.setOrigin(t2);
    // transform2.setRotation(q2);
    // br->sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "/tool0_controller", "/blaser_cam"));

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_rect_node");
    // ROS_INFO("Press Enter to start...\n");
    // std::cin.get();
    std::string config = argv[1];
    std::cout<<config<<std::endl;
    camodocal::CameraPtr m_camera_;

    ros::NodeHandle nh_;
    ros::Subscriber img_sub = nh_.subscribe("/ximea_cam/image_raw",1,image_callback);
    info_pub = nh_.advertise<sensor_msgs::CameraInfo>("/blaser_cam/camera_info",1);
    image_transport::ImageTransport it_(nh_);
    img_pub = it_.advertise("/blaser_cam/image_rect_color",1);
    br = new tf::TransformBroadcaster();

    m_camera_ = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config);
    
    // if(nh_.hasParam("mirrorScale")){
    //     nh_.getParam("mirrorScale",scale);
    // }
    // else{
    //     ROS_ERROR("Missing camera mirror parameters, exiting");
    //     ros::shutdown();
    // }

    R_rect_cv_ = cv::Mat::eye(3, 3, CV_32F);
    R_rect_cv_.at<float>(0, 0) = 0.2;
    R_rect_cv_.at<float>(1, 1) = 0.2;

    cv::Mat K_rect;
    K_rect = m_camera_->initUndistortRectifyMap(rect_map1_, rect_map2_, -1.0, -1.0,
      cv::Size(0,0), -1.0, -1.0, R_rect_cv_);

    std::vector<double> cameraMatrix;

    double fx,fy,cx,cy;

    if(nh_.hasParam("rectCameraMatrix")){
        nh_.getParam("rectCameraMatrix",cameraMatrix);
        fx = cameraMatrix[0];
        fy = cameraMatrix[1];
        cx = cameraMatrix[2];
        cy = cameraMatrix[3];
    }
    else{
        ROS_ERROR("Missing camera matrix, exiting");
        ros::shutdown();
    }
    

    info.header.frame_id = "tool0";
    info.width = 2448;
    info.height = 1840;
    info.distortion_model = "plumb_bob";
    info.D = std::vector<double> {0,0,0,0,0};
    info.K = boost::array<double, 9> {fx,0,cx,0,fy,cy,0,0,1};
    info.P = boost::array<double, 12> {fx,0,cx,0,0,fy,cy,0,0,0,1,0};

    while (true) {
        ros::spinOnce();
    }
    return 0;
}