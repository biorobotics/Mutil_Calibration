#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include "std_msgs/String.h"

static tf::TransformListener *listener;
std::string dir;
std::string filePath;
tf::StampedTransform tf_e2b,tf_c2t;
std::vector<tf::Transform> transformsToTag5;
tf::StampedTransform tf_tag5,tf_tag10,tf_tag15,tf_tag20,tf_tag25;
static tf::TransformBroadcaster *br;
int sampleNum = 0;

double getRadius(tf::Vector3 translation);

double getRadius(tf::Vector3 translation){
    return std::sqrt(pow(translation.getX(),2)+pow(translation.getY(),2)+pow(translation.getZ(),2));
}

void printHelper(tf::Vector3 tf){
    std::cout<<tf.getX()<<" "<<tf.getY()<<" "<<tf.getZ()<<std::endl;
}

void printHelper(tf::Quaternion quat){
    std::cout<<quat.getX()<<" "<<quat.getY()<<" "<<quat.getZ()<<" "<<quat.getW()<<std::endl;
}

void status_callback(const std_msgs::String::ConstPtr& msg){
    ros::Time receivedTime = ros::Time::now();
    try{
        // This is giving the coordinates of /tool0 in the /base_link's frame
        listener->lookupTransform("/base_link", "/tool0", ros::Time(0), tf_e2b);
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("Arm TF lookup failed.");
        return;
    }
    try{
        // This is giving the coordinates of /usb_cam in the /tag_0's frame
        listener->lookupTransform("/tag_5", "/0", ros::Time(0), tf_tag5);
        // printHelper(tf_tag5.getOrigin());
        // printHelper(tf_tag5.getRotation());
        //std::cout<<"Tag 5 Distance "<<getRadius(tf_tag5.getOrigin())<<std::endl;
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("Tag 5 TF lookup failed.");
    }
    try{
        listener->lookupTransform("/tag_10", "/0", ros::Time(0), tf_tag10);
        //std::cout<<"Tag 10 Distance "<<getRadius(tf_tag10.getOrigin())<<std::endl;
        // printHelper(tf_tag10.getOrigin());
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("Tag 10 TF lookup failed.");
    }
    try{
        listener->lookupTransform("/tag_15", "/0", ros::Time(0), tf_tag15);
        //std::cout<<"Tag 15 Distance "<<getRadius(tf_tag15.getOrigin())<<std::endl;
        // printHelper(tf_tag15.getOrigin());
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("Tag 15 TF lookup failed.");
    }
    try{
        listener->lookupTransform("/tag_20", "/0", ros::Time(0), tf_tag20);
        //std::cout<<"Tag 20 Distance "<<getRadius(tf_tag20.getOrigin())<<std::endl;
        // printHelper(tf_tag20.getOrigin());
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("Tag 20 TF lookup failed.");
    }
    try{
        listener->lookupTransform("/tag_25", "/0", ros::Time(0), tf_tag25);
        //std::cout<<"Tag 25 Distance "<<getRadius(tf_tag25.getOrigin())<<std::endl;
        // printHelper(tf_tag25.getOrigin());
        // printHelper(tf_tag25.getRotation());
    }
    catch (tf::TransformException &ex) {
        ROS_WARN("Tag 25 TF lookup failed.");
    }
           
    //Get the ground truth radius from the msg
    std::stringstream ss(msg->data);
    std::string temp;
    double gt_radius;
    ss >> temp >> gt_radius;
    double threshold = 0.003;

    std::vector<tf::StampedTransform> rawTFs {tf_tag5,tf_tag10,tf_tag15,tf_tag20,tf_tag25};
    std::vector<tf::Transform> regulatedTF;
    std::vector<double> weights;

    double tag5_radius = getRadius(tf_tag5.getOrigin());
    regulatedTF.push_back(rawTFs[0]);
    weights.push_back(1);

    // br->sendTransform(tf::StampedTransform(rawTFs[1]*transformsToTag5[1].inverse(), ros::Time::now(), "/0", "/tag_5_10"));
    // br->sendTransform(tf::StampedTransform(rawTFs[2]*transformsToTag5[2].inverse(), ros::Time::now(), "/0", "/tag_5_15"));
    // br->sendTransform(tf::StampedTransform(rawTFs[3]*transformsToTag5[3].inverse(), ros::Time::now(), "/0", "/tag_5_20"));
    // br->sendTransform(tf::StampedTransform(rawTFs[4]*transformsToTag5[4].inverse(), ros::Time::now(), "/0", "/tag_5_25"));
    // br->sendTransform(tf::StampedTransform(transformsToTag5[1].inverse(),ros::Time::now(),"/tag_10","/tag_10_2"));
    for(int i = 1; i< 5;i++){
        
        regulatedTF.push_back(transformsToTag5[i]*rawTFs[i]);

        printHelper(regulatedTF[i].getOrigin());
        double perceivedRadius = getRadius(regulatedTF[i].getOrigin());

        if(abs(perceivedRadius-tag5_radius) < threshold && abs(rawTFs[i].stamp_.toSec() - receivedTime.toSec()) < 1){
            weights.push_back(1);
        }
        else{
            weights.push_back(0);
        }
    }

    double sum = 0;
    for(int i = 0; i < weights.size(); i ++){
        sum += weights[i];
    }

    if(sum == 0){
        return;
    }

    tf::Vector3 tagv(0,0,0);
    tf::Quaternion tagq(0,0,0,0);
    std::string frame_ID = "/sample_" + std::to_string(sampleNum++);

    for(int i = 0; i < weights.size(); i ++){
        weights[i] /= sum;
        tagv += regulatedTF[i].getOrigin()*weights[i];
        tagq += regulatedTF[i].getRotation()*weights[i];
    }

    // br->sendTransform(tf::StampedTransform(tf::Transform(tagq,tagv),ros::Time::now(),"/0",frame_ID));

    tf::Vector3 armv = tf_e2b.getOrigin();
    tf::Quaternion armq = tf_e2b.getRotation();
    
    std::ofstream out;
    out.open(filePath, std::ios_base::app);
    
    out << armv.getX() << "," << armv.getY() << "," << armv.getZ() << ",";
    out << armq.getW() << "," << armq.getX() << "," << armq.getY() << "," << armq.getZ() << ",";
    out << tagv.getX() << "," << tagv.getY() << "," << tagv.getZ() << ",";
    out << tagq.getW() << "," << tagq.getX() << "," << tagq.getY() << "," << tagq.getZ() << ","<<getRadius(tagv) << std::endl;

    out.close();
    ROS_INFO("TF Pairs saved sucessfully!");
    std::cout<<"Tag source accepted: ";
    for(int i = 0; i < weights.size(); i ++){
        if(weights[i] != 0){
            std::cout<<(i+1) * 5<<" ";
        }
        
    }
    std::cout<<std::endl;
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_saving_node");
    ros::NodeHandle nh_;

    tf::Transform tag5_5,tag5_10,tag5_15,tag5_20,tag5_25;
    tag5_5.setOrigin(tf::Vector3(0,0,0));
    tag5_5.setRotation(tf::Quaternion(0,0,0,1));
    tag5_10.setOrigin(tf::Vector3(-0.033,0.033,0));
    tag5_10.setRotation(tf::Quaternion(0,0,0,1));
    tag5_15.setOrigin(tf::Vector3(-0.033,-0.033,0));
    tag5_15.setRotation(tf::Quaternion(0,0,0,1));
    tag5_20.setOrigin(tf::Vector3(0.033,0.033,0));
    tag5_20.setRotation(tf::Quaternion(0,0,0,1));
    tag5_25.setOrigin(tf::Vector3(0.033,-0.033,0));
    tag5_25.setRotation(tf::Quaternion(0,0,0,1));
    transformsToTag5 = {tf::Transform(),tag5_10,tag5_15,tag5_20,tag5_25};
    br = new tf::TransformBroadcaster();

    dir = argv[1];
    filePath = dir + "/tf_pairs.txt";
    listener = new tf::TransformListener();
    ros::Subscriber status_sub = nh_.subscribe("execution_status",10,status_callback);

    std::ofstream out;
    out.open(filePath);
    out << "armx,army,armz,armqw,armqx,armqy,armqz,camx,camy,camz,camrw,camrx,camry,camrz,camRadius"<<std::endl;
    out.close();

    while (true) {
        ros::spinOnce();
    }
    return 0;
}
