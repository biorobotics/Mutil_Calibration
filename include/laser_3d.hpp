#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "ros/ros.h"

#include "laser_detector.hpp"
#include "mei_camera.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Laser3D
{
public:
    Laser3D();
    explicit Laser3D(ros::NodeHandle nh,std::string fileName);
    void laser_image_to_3d(cv::Mat img, pcl::PointCloud<pcl::PointXYZ>::Ptr laser_point_cloud);

private:
    
    LaserDetector laser_detector;
    MeiCamera camera_model;
   
    bool is_fisheye;

    void ptsNormTo3D(const std::vector<cv::Point2f> &pts_norm,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr pts_3d);

    std::vector<double> plane_;

};