#include "laser_3d.hpp"

/*
int main() {
    cv::Mat img = cv::imread("./img007.png");
    Laser3D laser3d;
    std::vector<cv::Point3f> laser_pts_3d;
    laser3d.laser_image_to_3d(img, laser_pts_3d);
    std::cout << laser_pts_3d << std::endl;
    return 0;
}
*/

Laser3D::Laser3D(){}  


Laser3D::Laser3D(ros::NodeHandle nh,std::string fileName)
{   
    if(nh.hasParam("laserPlane")){
        nh.getParam("laserPlane",this->plane_);
    }
    else{
        ROS_ERROR("Missing laser plane equations, exiting");
        ros::shutdown();
    }

    std::vector<double> x;
    this->laser_detector = LaserDetector(nh);
    this->camera_model = MeiCamera(fileName);
    

    // /* PINHOLE camera model */
    // this->is_fisheye = false;
    // this->pinhole_model = camodocal::PinholeCamera("pinhole", 1280, 720, 
    //   -4.1466571075099856e-01, 1.9335424492415187e-01, 2.3748871937276296e-04, -3.8193638143259952e-04, 
    //   7.1570765642552806e+02, 7.1434747013342599e+02, 6.0734131555868635e+02, 3.7237235679525492e+02);
    // this->plane_[0] = 17.0859;
    // this->plane_[1] = 348.16;
    // this->plane_[2] = -100;
    // this->plane_[3] = 9.92954;

}

void Laser3D::laser_image_to_3d(cv::Mat img, pcl::PointCloud<pcl::PointXYZ>::Ptr laser_point_cloud)
{
  // cv::Mat img, std::vector<cv::Point3f> &laser_pts_3d

  //! 1. find laser points on original image
  std::vector<cv::Point2f> laser_uv_d; // distorted
  this->laser_detector.detectLaserStripe(img, laser_uv_d);

  //! 2. undistort and normalized laser pixels
  std::vector<cv::Point2f> laser_uv_norm;
  laser_uv_norm.reserve(laser_uv_d.size());
  for (auto pt_uv_d : laser_uv_d)
  {
    Eigen::Vector3d pt_norm_3d;
    Eigen::Vector2d uv_d(pt_uv_d.x, pt_uv_d.y);
    this->camera_model.liftProjective(uv_d, pt_norm_3d);
    // if (this->is_fisheye) {
      // TODO: this is ugly... should refactor with OOP design
        
    // } else {
    //     this->pinhole_model.liftProjective(uv_d, pt_norm_3d);
    // }
    
    pt_norm_3d /= pt_norm_3d(2);
    laser_uv_norm.emplace_back(pt_norm_3d[0], pt_norm_3d[1]);
  }

  //! 3. compute 3D positions of laser pixels
  ptsNormTo3D(laser_uv_norm, laser_point_cloud);

  
}

void Laser3D::ptsNormTo3D(const std::vector<cv::Point2f> &pts_norm,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr pts_3d)
{
  pts_3d->width = pts_norm.size();
  pts_3d->height = 1;
  pts_3d->reserve(pts_3d->width * pts_3d->height);
  int i = 0;

  for (auto pt_norm : pts_norm)
  {
    double z = -plane_[3] /
        (pt_norm.x * plane_[0] + pt_norm.y * plane_[1] + plane_[2]);
    pts_3d->push_back(pcl::PointXYZ(pt_norm.x * z, pt_norm.y * z, z));
  }
}

// pts_3d.clear();
//   pts_3d.reserve(pts_norm.size());

//   for (auto pt_norm : pts_norm)
//   {
//     double z = -plane_[3] /
//         (pt_norm.x * plane_[0] + pt_norm.y * plane_[1] + plane_[2]);
//     pts_3d.emplace_back(pt_norm.x * z, pt_norm.y * z, z);
//   }