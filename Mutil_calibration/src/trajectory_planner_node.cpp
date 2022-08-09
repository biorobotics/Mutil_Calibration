/**
 * trajectory_planner_node.cpp
 * 
 * Author: Yuchen Wu
 * Contact: wuyc@umich.edu
 *
 */
#include "ros/ros.h"
#include "plannerFactory.hpp"


int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_planner_node");
    ros::NodeHandle nh_;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    std::cout<<"hi"<<std::endl;

    static const std::string PLANNING_GROUP = "manipulator";

    plannerFactory factory = plannerFactory(PLANNING_GROUP,std::make_shared<ros::NodeHandle>());
    plannerFactory::calibType type;

    if(atoi(argv[1]) == 0){
        type = plannerFactory::camera_intrinsic;
        std::shared_ptr<BaseMotionPlanner> planner = factory.generatePlanner(type);
    } 
    else if(atoi(argv[1]) == 1){
        type = plannerFactory::hand_eye;
        std::shared_ptr<BaseMotionPlanner> planner = factory.generatePlanner(type);
    }
    else if(atoi(argv[1]) == 2){
        type = plannerFactory::laser_cam;
        std::shared_ptr<BaseMotionPlanner> planner = factory.generatePlanner(type);
    }
    else if(atoi(argv[2]) == 3){
        type = plannerFactory::zig_zag;   
        std::shared_ptr<BaseMotionPlanner> planner = factory.generatePlanner(type);
    }

    return 0;
}

