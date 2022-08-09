#include "plannerFactory.hpp"

plannerFactory::plannerFactory(std::string name_in,std::shared_ptr<ros::NodeHandle> nh_) : \
    group_name(name_in), nh(nh_) {}

std::shared_ptr<BaseMotionPlanner> plannerFactory::generatePlanner(calibType type){
    switch(type){
        case(camera_intrinsic):
            my_planner = std::make_shared<CameraIntrinsicMotionPlanner> (group_name,nh);
            break;
        case(hand_eye):
            my_planner = std::make_shared<HandEyeMotionPlanner> (group_name,nh);
            break;
        case(laser_cam):
            my_planner = std::make_shared<LaserCamMotionPlanner> (group_name,nh);
            break;
        case(zig_zag):
            my_planner = std::make_shared<ScanningMotionPlanner> (group_name,nh);
            break;
        default:
            ROS_ERROR("Undefined planner type");
            break;
    }
    return my_planner;
}