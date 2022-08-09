#include "trajectory_planner.hpp"
MOVEIT_CLASS_FORWARD(MoveGroupInterface);
MOVEIT_CLASS_FORWARD(Plan);

BaseMotionPlanner::BaseMotionPlanner(std::string name_in,std::shared_ptr<ros::NodeHandle> nh_){
    move_group_name = name_in;
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface> (move_group_name);
    joint_model_group = move_group->getCurrentState()->getJointModelGroup(move_group_name);
    my_plan = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>();
    
    nh = nh_;

    if(nh->hasParam("homePositions")){
        nh->getParam("homePositions",start_positions);
    }
    else{
        ROS_ERROR("Missing home position joint values, exiting");
        ros::shutdown();
    }
    

    if(!goHomePosition()){
        ROS_ERROR("Error in initializing Robot's state");
    }
    else{
        ROS_INFO("Motion Planner successfully initializied");
    }

    if(nh->hasParam("bedOrigin")){
        std::vector<double> origin_in;
        nh->getParam("bedOrigin",origin_in);
        origin.position.x = origin_in[0];
        origin.position.y = origin_in[1];
        origin.position.z = origin_in[2];
    }
    else{
        ROS_ERROR("Missing bed origin coordinates, exiting");
        ros::shutdown();
    }
    

}

BaseMotionPlanner::~BaseMotionPlanner(){
    if(goHomePosition()){
        ROS_INFO("Trajectory successfully finished, press Ctrl+C to exit!\n");
    }
}

geometry_msgs::Pose BaseMotionPlanner::getCurrentPose(){
    return move_group->getCurrentPose().pose;
}

bool BaseMotionPlanner::plan(std::vector<geometry_msgs::Pose>  &waypoints){
    moveit_msgs::RobotTrajectory trajectory_msg;
    move_group->setPlanningTime(10.0);
        
    double fraction = move_group->computeCartesianPath(waypoints,
                                                0.01,  // eef_step
                                                0.0,   // jump_threshold
                                                trajectory_msg, true);

    // The trajectory needs to be modified so it will include velocities as well.
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(move_group->getCurrentState()->getRobotModel(), "manipulator");

    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*move_group->getCurrentState(), trajectory_msg);
    
    // Third compute computeTimeStamps
    bool success = iptp.computeTimeStamps(rt);

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory_msg);

    // Finally plan and execute the trajectory
    my_plan->trajectory_ = trajectory_msg;

    return success;
}

bool BaseMotionPlanner::execute(){
    moveit::core::MoveItErrorCode result = move_group->execute(*my_plan);
    // move_group->stop();
    
    if(result != moveit::core::MoveItErrorCode::SUCCESS){
        return false;
    }
    else{
        return checkEERotation();
    }
}

bool BaseMotionPlanner::checkEERotation(){
    //We need to make sure EE joint is always away from [-2pi,2pi]
    //So correction is necessary
    bool correctionResult = true;
    moveit::core::RobotStatePtr current_state = move_group->getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    if(abs(joint_group_positions[5]) > 1.2*M_PI){
        ros::Publisher status_pub = nh->advertise<std_msgs::String>("execution_status", 1000);
        
        std_msgs::String msg2;
        std::stringstream end;
        end<<"End"<<" "<<ros::Time::now().sec<<" "<<ros::Time::now().nsec;
        msg2.data = end.str();
        status_pub.publish(msg2);

        ROS_INFO("Warning: EE joint rotation closed to limit: %f, applying correction\n",joint_group_positions[5]);
        double step = 0.0;

        if (joint_group_positions[5] > 0){
            step = -2*M_PI;
        }
        else{
            step = 2*M_PI;
        }       

        joint_group_positions[5] += step;
        move_group->setJointValueTarget(joint_group_positions);
        correctionResult = (move_group->plan(*my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        correctionResult = (move_group->execute(*my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        move_group->stop();

        if(correctionResult){
            ROS_INFO("EE joint correction successful\n");
        }
        else{
            ROS_WARN("EE joint correction UNsuccessful\n");
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        }
        ROS_INFO("Current end effector orientation: %f \n",joint_group_positions[5]);
            
        std_msgs::String msg;
        std::stringstream start;
        start<<"Start"<<" "<<ros::Time::now().sec<<" "<<ros::Time::now().nsec;
        msg.data = start.str();
        status_pub.publish(msg);    
    }
    return correctionResult;
}

bool BaseMotionPlanner::goHomePosition(){
    return goJointPosition(start_positions);
}

bool BaseMotionPlanner::goJointPosition(std::vector<double> jointGoals){
    bool result = false;
    move_group->setJointValueTarget(jointGoals);
    result = (move_group->plan(*my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    result = (move_group->execute(*my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    return result;
}

CameraIntrinsicMotionPlanner::CameraIntrinsicMotionPlanner(std::string name_in,std::shared_ptr<ros::NodeHandle> nh_) \
: BaseMotionPlanner(name_in,nh_){
    nh->getParam("pointPerSide",pointPerSide);
    nh->getParam("pointPerHeight",pointPerHeight);

    //Prepare quaternions, 6 different orientation per point
    double roll = deg2rad(-90),  pitch = deg2rad(-20);    
    Eigen::Quaterniond q1 = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
    orientations.push_back(q1);

    Eigen::Quaterniond q2 = q1 * Eigen::AngleAxisd(deg2rad(45),Eigen::Vector3d::UnitZ());
    orientations.push_back(q2);

    Eigen::Quaterniond q3 = q1 * Eigen::AngleAxisd(deg2rad(-45),Eigen::Vector3d::UnitZ());
    orientations.push_back(q3);

    Eigen::Quaterniond q4 = q1 * Eigen::AngleAxisd(deg2rad(30),Eigen::Vector3d::UnitX());
    orientations.push_back(q4);

    Eigen::Quaterniond q5 = q1 * Eigen::AngleAxisd(deg2rad(-30),Eigen::Vector3d::UnitX());
    orientations.push_back(q5);

    Eigen::Quaterniond q6(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    orientations.push_back(q6);

    for(int i = 0; i < 6; i++){
        std::cout<<orientations[i].coeffs()[0]<<" "<<orientations[i].coeffs()[1]<<" "<<orientations[i].coeffs()[2]<<" "<<orientations[i].coeffs()[3]<<std::endl;
    }

    //Prepare x and z positions
    double start1 = 0.06, end1 = 0; 
    double step1 = (end1-start1)/(pointPerSide-1);
    double temp1 = start1;
    do{
        sidePositions.push_back(temp1);
        temp1 += step1;
    }
    while(temp1 > end1);
    sidePositions.push_back(end1);
    for(int i = 0; i < 3; i++){
        std::cout<<sidePositions[i]<<std::endl;
    }
    //Prepare y positions
    double start2 = 0.125, end2 = 0.155; 
    double step2 = (end2-start2)/(pointPerHeight-1);
    double temp2 = start2;
    do{
        heightPositions.push_back(temp2);
        temp2 += step2;
    }
    while(temp2  < end2);
    heightPositions.push_back(end2);

    generateMotion();
}

void CameraIntrinsicMotionPlanner::generateMotion(){
    ROS_INFO("Press Enter to start...\n");
    std::cin.get();
    
    tf::StampedTransform tf_e2b;
    tf::TransformListener listener;

    std::string filePath = "/home/wu/Documents/biorobotics/cam_tf.txt";
    std::ofstream out;
    out.open(filePath);
    out << "armx,army,armz,armqw,armqx,armqy,armqz"<<std::endl;
    out.close();

    ros::Publisher status_pub = nh->advertise<std_msgs::String>("execution_status", 1000);
    int totalPoints = pointPerHeight*pointPerSide*pointPerSide*6;
    int currentPoint = 0;

    for(int i = 0; i < pointPerHeight; i++){
        for(int j = 0; j < pointPerSide; j++){
            for(int k = 0; k < pointPerSide; k++){
                for(int l = 0; l < 6; l++){

                    geometry_msgs::Pose nextPose = origin;
                    nextPose.position.x += sidePositions[j];
                    nextPose.position.y -= heightPositions[i];
                    nextPose.position.z += sidePositions[k];

                    nextPose.orientation.x = orientations[l].coeffs()[0];
                    nextPose.orientation.y = orientations[l].coeffs()[1];
                    nextPose.orientation.z = orientations[l].coeffs()[2];
                    nextPose.orientation.w = orientations[l].coeffs()[3];

                    std::vector<geometry_msgs::Pose> waypoints;
                    // waypoints.push_back(move_group->getCurrentPose().pose);
                    waypoints.push_back(nextPose);
                
                    currentPoint++;

                    if(plan(waypoints) && execute()){
                        //Send message to aruco node to let it know it's time to take pictures and tf pairs
                        std_msgs::String msg;

                        std::stringstream ss;
                        ss << "Pose successfully reached, total progress " << (currentPoint)/double(totalPoints)*100.0 <<"%.";

                        msg.data = ss.str();
                        ROS_INFO("%s", msg.data.c_str());
                        status_pub.publish(msg);

                        ros::Duration(0.5).sleep();

                        listener.lookupTransform("/base_link", "/tool0", ros::Time(0), tf_e2b);
                        tf::Vector3 armv = tf_e2b.getOrigin();
                        tf::Quaternion armq = tf_e2b.getRotation();
                        
                        std::ofstream out;
                        out.open(filePath, std::ios_base::app);
                        
                        out << armv.getX() << "," << armv.getY() << "," << armv.getZ() << ",";
                        out << armq.getW() << "," << armq.getX() << "," << armq.getY() << "," << armq.getZ() << std::endl;
                    }
                }
            }
        }
    }

}

HandEyeMotionPlanner::HandEyeMotionPlanner(std::string name_in,std::shared_ptr<ros::NodeHandle> nh_)\
: BaseMotionPlanner(name_in,nh_){
    //Collect Parameters
    nh->getParam("numPointsPerVertArc",numPointsPerVertArc);
    nh->getParam("numVertArc",numVertArc);
    nh->getParam("numRadius",numRadius);   

    //Prepare the vertical degree lists
    double startAngle1 = 35.0, endAngle1 = 135.0; 
    double step1 = (endAngle1-startAngle1)/(numPointsPerVertArc-1);
    double temp1 = startAngle1;
    do{
        vertDegreeList.push_back(temp1);
        temp1 += step1;
    }
    while(temp1  < endAngle1);
    vertDegreeList.push_back(endAngle1);

    //Prepare the horizontal degree lists
    double startAngle2 = 0.0, endAngle2 = 180.0; 
    double step2 = (endAngle2-startAngle2)/(numVertArc/2);
    double temp2 = startAngle2;
    do{
        horiDegreeList.push_back(temp2);
        horiDegreeList.push_back(temp2+180);
        temp2 += step2;
    }
    while(temp2  < endAngle2);

    //Prepare the radius lists
    double startRadius = 0.17, endRadius = 0.18;
    double step3 = (endRadius-startRadius)/(numRadius-1);
    double temp3 = startRadius;
    do{
        radiusList.push_back(temp3);
        temp3 += step3;
    }
    while(temp3  < endRadius);
    radiusList.push_back(endRadius);

    generateMotion();
}

void HandEyeMotionPlanner::generateMotion(){
    ros::Publisher status_pub = nh->advertise<std_msgs::String>("execution_status", 1000);
    int totalPoints = numRadius*numVertArc*numPointsPerVertArc;
    int currentPoint = 0;

    ROS_INFO("Press Enter to start...\n");
    std::cin.get();


    // geometry_msgs::Pose nextPose = origin;
    // nextPose.position.y -= 0.20;
    // std::vector<geometry_msgs::Pose> waypoints;
    // // waypoints.push_back(move_group->getCurrentPose().pose);
    // waypoints.push_back(nextPose);
    // plan(waypoints);
    // execute();

    // ROS_INFO("Press Enter to start...\n");
    // std::cin.get();


    for(int u = 0; u < numRadius; u++){
        for(int i = 0; i < numVertArc; i++){
            ROS_INFO("Starting new arc %f of radius %f\n",horiDegreeList[i],radiusList[u]);
            
            std_msgs::String msg2;
            std::stringstream end;
            end<<"End"<<" "<<ros::Time::now().sec<<" "<<ros::Time::now().nsec;
            msg2.data = end.str();
            status_pub.publish(msg2);
            
            for(int j = 0; j < numPointsPerVertArc; j++){
                
                geometry_msgs::Pose nextPose = origin;
                nextPose.position.x -= radiusList[u]*cos(deg2rad(vertDegreeList[j]))*cos(deg2rad(horiDegreeList[i]+5.0*u));
                nextPose.position.y -= radiusList[u]*sin(deg2rad(vertDegreeList[j]));
                nextPose.position.z -= radiusList[u]*cos(deg2rad(vertDegreeList[j]))*sin(deg2rad(horiDegreeList[i]+5.0*u));

                double roll = deg2rad(-90), pitch = deg2rad(90-vertDegreeList[j]), yaw = deg2rad(-horiDegreeList[i]);    
                Eigen::Quaterniond q;
                q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());

                nextPose.orientation.x = q.coeffs()[0];
                nextPose.orientation.y = q.coeffs()[1];
                nextPose.orientation.z = q.coeffs()[2];
                nextPose.orientation.w = q.coeffs()[3];

                std::vector<geometry_msgs::Pose> waypoints;
                // waypoints.push_back(move_group->getCurrentPose().pose);
                waypoints.push_back(nextPose);

                currentPoint++;
 
                if(j == 1){
                    std_msgs::String msg;
                    std::stringstream start;
                    start<<"Start"<<" "<<ros::Time::now().sec<<" "<<ros::Time::now().nsec;
                    msg.data = start.str();
                    status_pub.publish(msg);
                }

                if(plan(waypoints) && execute()){
                    // ros::Duration(1).sleep();

                    std::stringstream ss;
                    ss << "Radius "<<radiusList[u]<<" Waypoint at vertical degree "<<vertDegreeList[j]<<" and horizontal degree "<<horiDegreeList[i]<<" reached,";
                    ss << "total progress " << (currentPoint)/double(totalPoints)*100.0 <<"%.";
                    ROS_INFO("%s", ss.str().c_str());
                    
                }
                // ROS_INFO("Press Enter to start...\n");
                // std::cin.get();
            }
        }
    }
    std_msgs::String msg2;
    std::stringstream end;
    end<<"End"<<" "<<ros::Time::now().sec<<" "<<ros::Time::now().nsec;
    msg2.data = end.str();
    status_pub.publish(msg2);

}

LaserCamMotionPlanner::LaserCamMotionPlanner(std::string name_in, std::shared_ptr<ros::NodeHandle> nh_)\
    : BaseMotionPlanner(name_in,nh_){

    //Collect Parameters
    nh->getParam("numLinePerPlane",numLinePerPlane);
    nh->getParam("numPlaneAngles",numPlaneAngles);
    nh->getParam("numPlanePerCircle",numPlanePerCircle);

    origin.position.y -= 0.15;

    //Prepare the vertical degree lists
    double startAngle1 = 10.0, endAngle1 = 20.0; 
    double step1 = (endAngle1-startAngle1)/(numPlaneAngles);
    double temp1 = startAngle1;
    do{
        vertDegreeList.push_back(temp1);
        temp1 += step1;
    }
    while(temp1  < endAngle1);

    //Prepare the horizontal degree lists
    double startAngle2 = 0.0, endAngle2 = 360.0; 
    double step2 = (endAngle2-startAngle2)/(numPlanePerCircle);
    double temp2 = startAngle2;
    do{
        horiDegreeList.push_back(temp2);
        temp2 += step2;
    }
    while(temp2  < endAngle2);

    //Prepare the radius lists
    double startRadius = 0, endRadius = 0.07;
    double step3 = (endRadius-startRadius)/(numLinePerPlane-1);
    double temp3 = startRadius;
    do{
        radiusList.push_back(temp3);
        temp3 += step3;
    }
    while(temp3  < endRadius);
    radiusList.push_back(endRadius);
    
    generateMotion();
}

void LaserCamMotionPlanner::generateMotion(){
    ros::Publisher status_pub = nh->advertise<std_msgs::String>("execution_status", 1000);
    int totalPoints = numLinePerPlane * numPlaneAngles * numPlanePerCircle;
    int currentPoint = 0;

    ROS_INFO("Press Enter to start...\n");
    std::cin.get();

    for(int u = 0; u < numPlanePerCircle; u++){
        for(int i = 0; i < numPlaneAngles; i++){
            ROS_INFO("Starting new arc %f of radius %f\n",horiDegreeList[i],radiusList[u]);
            for(int j = 0; j < numLinePerPlane; j++){
                geometry_msgs::Pose nextPose = origin;
                nextPose.position.x += radiusList[j]*sin(deg2rad(vertDegreeList[i]))*cos(deg2rad(horiDegreeList[u]));
                nextPose.position.y -= radiusList[j]*cos(deg2rad(vertDegreeList[i]));
                nextPose.position.z -= radiusList[j]*sin(deg2rad(vertDegreeList[i]))*sin(deg2rad(horiDegreeList[u]));

                double roll = deg2rad(-90), pitch = deg2rad(vertDegreeList[i] - 20), yaw = deg2rad(horiDegreeList[u]);    
                Eigen::Quaterniond q;
                q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());

                nextPose.orientation.x = q.coeffs()[0];
                nextPose.orientation.y = q.coeffs()[1];
                nextPose.orientation.z = q.coeffs()[2];
                nextPose.orientation.w = q.coeffs()[3];

                std::vector<geometry_msgs::Pose> waypoints;
                // waypoints.push_back(move_group->getCurrentPose().pose);
                waypoints.push_back(nextPose);

                currentPoint++;

                if(plan(waypoints) && execute()){
                    ros::Duration(1).sleep();

                    //Send message to aruco node to let it know it's time to take pictures and tf pairs
                    std_msgs::String msg;

                    std::stringstream ss;
                    ss << "Distance "<<radiusList[j]<<" Waypoint at vertical degree "<<vertDegreeList[i]<<" and horizontal degree "<<horiDegreeList[u]<<" reached,";
                    ss << "total progress " << (currentPoint)/double(totalPoints)*100.0 <<"%.";

                    msg.data = ss.str();
                    ROS_INFO("%s", msg.data.c_str());
                    status_pub.publish(msg);

                    
                }
                // ROS_INFO("Press Enter to start...\n");
                // std::cin.get();
            }
        }
    }

}

ScanningMotionPlanner::ScanningMotionPlanner(std::string name_in, std::shared_ptr<ros::NodeHandle> nh_)\
    : BaseMotionPlanner(name_in,nh_){

    //Collect Parameters
    nh->getParam("pointPerSide",pointPerSide);
    absoluteStep = 0.2/pointPerSide;

    scanPlane front,back,left,right,top;

    top.orientation = Eigen::AngleAxisd(-90, Eigen::Vector3d::UnitX());
    right.orientation = top.orientation * Eigen::AngleAxisd(90, Eigen::Vector3d::UnitY());
    front.orientation = right.orientation * Eigen::AngleAxisd(90, Eigen::Vector3d::UnitX());
    back.orientation = right.orientation * Eigen::AngleAxisd(-90,Eigen::Vector3d::UnitX());
    left.orientation = right.orientation * Eigen::AngleAxisd(-180,Eigen::Vector3d::UnitX());

    back.move = Eigen::Vector3d(0,1,0);
    left.move = Eigen::Vector3d(0,1,0);
    front.move = Eigen::Vector3d(0,1,0);
    right.move = Eigen::Vector3d(0,1,0);
    top.move = Eigen::Vector3d(-1,0,0);

    // back.shift = Eigen::Vector3d(1,0,0).transpose() * Eigen::AngleAxisd(-90,Eigen::Vector3d::UnitX());
    left.shift = Eigen::Vector3d(0,0,1);
    front.shift = Eigen::Vector3d(-1,0,0);
    right.shift = Eigen::Vector3d(0,0,-1);
    top.shift = Eigen::Vector3d(0,0,-1);

    std::cout<<back.shift<<std::endl;
    std::cout<<left.orientation.vec()<<std::endl;

    generateMotion();
}

std::vector<geometry_msgs::Pose> ScanningMotionPlanner::generatePath(scanPlane plane){
    std::vector<geometry_msgs::Pose> path;

    return path ;
}

void ScanningMotionPlanner::generateMotion(){

}