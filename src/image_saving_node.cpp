#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "std_msgs/String.h"

bool flag = false;
std::string dir;
int imgCount = 0;

void status_callback(const std_msgs::String::ConstPtr& msg){
    flag = true;
    ROS_INFO("Image saving node received confirmation!");
}

void image_callback(const sensor_msgs::ImageConstPtr im_msg) {
    if(flag){
        flag = false;

        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(im_msg, sensor_msgs::image_encodings::BGR8);
        
        std::string fileName = dir + "/" + std::to_string(imgCount) + ".png";
        imwrite(fileName,cv_ptr->image);
        ROS_INFO("Image %d saved successfully!",imgCount++);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_saving_node");
    ros::NodeHandle nh_;

    dir = argv[1]; 
    std::string topicName;
    if(nh_.hasParam("cameraTopic")){
        nh_.getParam("cameraTopic",topicName);
    }
    else{
        ROS_ERROR("Missing camera topic name, exiting");
        ros::shutdown();
    }

    ros::Subscriber img_sub = nh_.subscribe(topicName,10,image_callback);
    ros::Subscriber status_sub = nh_.subscribe("execution_status",10,status_callback);

    while (true) {
        ros::spinOnce();
    }
    return 0;
}
