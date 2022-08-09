#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "laser_3d.hpp"
#include "std_msgs/String.h"

static Laser3D laser3d;
static ros::Publisher cloud_pub;
static tf::TransformBroadcaster *br;
static tf::TransformListener *listener;
tf::Transform handeyeTransform;
bool enableScan = false;

void laser_image_callback(const sensor_msgs::CompressedImageConstPtr im_msg)
{   
    if(enableScan == false) return;

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(im_msg, sensor_msgs::image_encodings::BGR8);

    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    laser3d.laser_image_to_3d(cv_ptr->image, laser_point_cloud);

    // send transformation between the end effector to camera (obtained from hand-eye calibration)
    br->sendTransform(tf::StampedTransform(handeyeTransform, im_msg->header.stamp, "/tool0_controller", "/camera"));
    
    tf::StampedTransform tf_w2c;
    listener->waitForTransform("/tool0_controller", "/camera", im_msg->header.stamp, ros::Duration(1));
    try
    {
        listener->lookupTransform("/base", "/camera", im_msg->header.stamp, tf_w2c);
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("TF lookup failed.");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Affine3d tf_w2c_eigen;
    tf::transformTFToEigen(tf_w2c,tf_w2c_eigen);
    pcl::transformPointCloud(*laser_point_cloud,*transformed_point_cloud,tf_w2c_eigen);
    
    transformed_point_cloud->header.frame_id = "base";
    transformed_point_cloud->header.stamp = im_msg->header.stamp.toNSec() / 1e3;;

    // for (unsigned int i = 0; i < laser_pts_3d.size(); i++)
    // {
    //     geometry_msgs::PointStamped p;
    //     geometry_msgs::PointStamped p_out;
    //     p.header.frame_id = "/camera";
    //     p.point.x = laser_pts_3d[i].x;
    //     p.point.y = laser_pts_3d[i].y;
    //     p.point.z = laser_pts_3d[i].z;

    //     listener->transformPoint("/base", im_msg->header.stamp, p, "/camera", p_out);
    //     point_cloud.points[i].x = p_out.point.x;
    //     point_cloud.points[i].y = p_out.point.y;
    //     point_cloud.points[i].z = p_out.point.z;

    // }

    pcl::PCLPointCloud2 point_cloud2;
    pcl::toPCLPointCloud2(*transformed_point_cloud, point_cloud2);

    cloud_pub.publish(point_cloud2);
}

void status_callback(const std_msgs::String::ConstPtr& msg)
{
    ros::Time receivedTime = ros::Time::now();
    std::stringstream ss(msg->data);
    std::string status;
    uint32_t sec_in,nsec_in;
    ss >> status >> sec_in >> nsec_in;

    if(status == "Start"){
        enableScan = true;
    }
    else if(status == "End"){
        enableScan = false;
    }

    ROS_INFO("Message Delay in sec: %zu, in nsec: %zu",receivedTime.sec-sec_in,receivedTime.nsec-nsec_in);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_blaser_node");

    br = new tf::TransformBroadcaster();
    listener = new tf::TransformListener();

    ros::NodeHandle nh_;

    std::string cameraTopic,statusTopic,pcTopic;
    nh_.getParam("cameraTopic",cameraTopic);
    nh_.getParam("statusTopic",statusTopic);
    nh_.getParam("pcTopic",pcTopic);
    
    ros::Subscriber img_sub = nh_.subscribe(cameraTopic, 10, laser_image_callback);
    
    ros::Subscriber status_sub = nh_.subscribe(statusTopic,10,status_callback);

    cloud_pub = nh_.advertise<pcl::PCLPointCloud2>(pcTopic, 1);

    std::vector<double> transform_in;

    if(nh_.hasParam("handeyeTransform")){
        nh_.getParam("handeyeTransform",transform_in);
        tf::Vector3 t(transform_in[0],transform_in[1],transform_in[2]);
        tf::Quaternion q(transform_in[3],transform_in[4],transform_in[5],transform_in[6]);
        handeyeTransform.setOrigin(t);
        handeyeTransform.setRotation(q);
    }
    else{
        ROS_ERROR("Missing handeye transformation, exiting");
        ros::shutdown();
    }

    laser3d = Laser3D(nh_,argv[1]);

    while(ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}
