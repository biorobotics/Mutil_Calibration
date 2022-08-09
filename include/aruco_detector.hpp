#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/aruco.hpp>

#include "mei_camera.hpp"
#include "pinhole_camera.hpp"

class ArucoDetector {
public:
    ArucoDetector();

    bool detectAruco(const cv::Mat& img, cv::Mat& rvec, cv::Mat& tvec);

private:
    // Pinhole Camera camera_model;
    camodocal::PinholeCamera camera_model;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    std::vector<cv::Point3d> obj_pts;

    int targetID = 3;
};
