#include <iostream>
// #include <opencv2/core/eigen.hpp>
#include "aruco_detector.hpp"

ArucoDetector::ArucoDetector() : camera_model()
{
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    parameters = cv::aruco::DetectorParameters::create();
    //parameters->cornerRefinementMethod =  cv::aruco::CORNER_REFINE_SUBPIX;
    //parameters->adaptiveThreshWinSizeStep = 3;

    // double tag_length = 25.0; // mm
    double tag_length = .025; // meter
    obj_pts.emplace_back(0, 0, 0);
    obj_pts.emplace_back(tag_length, 0, 0);
    obj_pts.emplace_back(tag_length, tag_length, 0);
    obj_pts.emplace_back(0, tag_length, 0);

    camera_model.readParametersFromYamlFile("/home/wu/Documents/biorobotics/usbcam_camera_calib.yaml");
}

bool ArucoDetector::detectAruco(const cv::Mat& img, cv::Mat& rvec, cv::Mat& tvec) {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    std::vector<cv::Point2f> corners;
    bool targetFound = false;
    for (size_t i = 0; i < markerIds.size(); ++i) {
        if (markerIds[i] == targetID) {
            targetFound = true;
            corners = markerCorners[i];
            break;
        }
    }

    // DRAW DISPLAY
    cv::Mat outputImage = img.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

    cv::Mat show;
    cv::resize(outputImage, show, cv::Size(1080, 720));
    cv::imshow("window0", show);
    // int key = cv::waitKey(1);
    // END DRAW DISPLY


    if (!targetFound) return false;

    std::vector<cv::Point2d> image_points;
    for (const auto& c : corners) {
        Eigen::Vector2d v;
        Eigen::Vector3d p;
        v << c.x, c.y;
        camera_model.liftProjective(v, p);

        //std::cout<<p<<std::endl;
        double mx_u = p.coeff(0), my_u = p.coeff(1), mz = p.coeff(2);
        image_points.emplace_back(mx_u / mz, my_u / mz);
    }

    // cv::Mat rvec, tvec;
    cv::solvePnP(obj_pts, image_points, cv::Mat::eye(3, 3, CV_64F), cv::noArray(), rvec, tvec);
    //std::cout<<tvec<<std::endl;
    return true;
}
