#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include "std_msgs/String.h"

#include "aruco_detector.hpp"

static ArucoDetector aruco;
static ros::Publisher cloud_pub;
static tf::TransformListener *listener;
bool flag = false;

void image_callback(const sensor_msgs::ImageConstPtr im_msg) {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(im_msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat camr, camt;
    bool detec = aruco.detectAruco(cv_ptr->image, camr, camt);
    // std::cout << "detec: " << detec << std::endl;
    auto key = cv::waitKey(1);
    if (key == 27) exit(0);

    if (!detec) {
        return;
    }

    tf::StampedTransform tf_w2c;
    try
    {
        listener->lookupTransform("/base_link", "/tool0", im_msg->header.stamp, tf_w2c);
    }
    catch (tf::TransformException &ex) {
        //ROS_WARN("TF lookup failed.");
        return;
    }
    tf::Vector3 armv = tf_w2c.getOrigin();
    tf::Quaternion armq = tf_w2c.getRotation();

    if (flag) {
        double radius = std::sqrt(pow(camt.at<double>(0, 0),2)+pow(camt.at<double>(1, 0),2)+pow(camt.at<double>(2, 0),2));
        double threshold = 0.2;
        if(radius > threshold){
            ROS_INFO("Camera distance to tag: %f, which is greater than threshold, current tf pair abandoned\n",radius);
        }
        else{
            ROS_INFO("Camera distance to tag: %f, which is within threshold, saving transform pair!\n",radius);
            std::ofstream out;
            out.open("/home/wu/Documents/biorobotics/tf_pairs.txt", std::ios_base::app);

            out << armv.getX() << "," << armv.getY() << "," << armv.getZ() << ",";
            out << armq.getW() << "," << armq.getX() << "," << armq.getY() << "," << armq.getZ() << ",";
            out << camt.at<double>(0, 0) << "," << camt.at<double>(1, 0) << "," << camt.at<double>(2, 0) << ",";
            out << camr.at<double>(0, 0) << "," << camr.at<double>(1, 0) << "," << camr.at<double>(2, 0) << std::endl;

            out.close();
        }
        flag = false;
    }
}

void status_callback(const std_msgs::String::ConstPtr& msg){
    flag = true;
    ROS_INFO("Aruco Node received pose reached confirmation!\n");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_node");
    ros::NodeHandle nh_;
    listener = new tf::TransformListener();
    ros::Subscriber img_sub = nh_.subscribe("/fujun", 10, image_callback);
    ros::Subscriber status_sub = nh_.subscribe("execution_status",10,status_callback);

    while (true) {
        ros::spinOnce();
    }
    return 0;
}
