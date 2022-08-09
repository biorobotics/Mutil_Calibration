#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <iostream>
#include <string>
#include "ros/ros.h"

class LaserDetector
{
public:
    explicit LaserDetector();
    LaserDetector(ros::NodeHandle nh); 

    bool detectLaserStripe(cv::Mat &im, std::vector<cv::Point2f> &laser_pts);

private:
    bool findCoMColumn(Eigen::MatrixXd &im,
                                        std::vector<cv::Point2f> &CoM_pts);

    bool findCoMRow(Eigen::MatrixXd &im,
                                     std::vector<cv::Point2f> &CoM_pts);

    void rejectOutliers(const std::vector<cv::Point2f> &pts_in,
                                         std::vector<cv::Point2f> &pts_out);

    void visualize(const cv::Mat &im_ori, const cv::Mat &hsv_mask,
                               const cv::Mat &im_v,
                               const std::vector<cv::Point2f> &laser_pixels) const;

    void generateHSVMasks(const cv::Mat& im_hsv, cv::Mat& hsv_mask) const;

     // camera intrinsics
    // cv::Mat K_;
    // cv::Mat D_;

    int img_height_;
    int img_width_;

    double ROI_x_, ROI_y_, ROI_width_, ROI_height_;
    double hue_min_, hue_max_, sat_min_, val_min_, val_ratio_;

    cv::Rect laser_ROI_;

    cv::Mat mask_dilate_kernel_, mask_close_kernel_;

    // control parameters
    bool f_vis_;
    bool find_laser_per_column;
};