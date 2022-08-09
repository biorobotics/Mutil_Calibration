#include "laser_detector.hpp"

#include <opencv2/core/eigen.hpp>

#define MIN_LASER_PTS 20

LaserDetector::LaserDetector(){}

LaserDetector::LaserDetector(ros::NodeHandle nh){
    nh.getParam("ROI_x",this->ROI_x_);
    nh.getParam("ROI_y",this->ROI_y_);
    nh.getParam("ROI_width",this->ROI_width_);
    nh.getParam("ROI_height",this->ROI_height_);

    this->laser_ROI_ = cv::Rect(this->ROI_x_, this->ROI_y_, this->ROI_width_, this->ROI_height_);
    
    nh.getParam("img_width",this->img_width_);
    nh.getParam("img_height",this->img_height_);

    nh.getParam("hue_min",this->hue_min_);
    nh.getParam("hue_max",this->hue_max_);

    nh.getParam("sat_min",this->sat_min_);
    nh.getParam("val_min",this->val_min_);
    nh.getParam("val_ratio",this->val_ratio_);

    nh.getParam("f_vis",this->f_vis_);
    this->find_laser_per_column = true;

    this->mask_dilate_kernel_ = cv::Mat::ones(cv::Size(25, 50), CV_8U);
    this->mask_close_kernel_ = cv::Mat::ones(cv::Size(25, 35), CV_8U);
}

bool LaserDetector::detectLaserStripe(cv::Mat &im, std::vector<cv::Point2f> &laser_pts)
{
    cv::Mat im_blur, im_hsv, im_v;
    cv::Mat hsv_mask;
    laser_pts.clear();
    laser_pts.reserve(laser_ROI_.width);

    // 1. pre-processing
    cv::medianBlur(im(laser_ROI_), im_blur, 3);

    // 2. get red mask
    cv::cvtColor(im_blur, im_hsv, cv::COLOR_BGR2HSV);
    generateHSVMasks(im_hsv, hsv_mask);

    cv::morphologyEx(hsv_mask, hsv_mask, cv::MORPH_DILATE, mask_dilate_kernel_);
    cv::morphologyEx(hsv_mask, hsv_mask, cv::MORPH_CLOSE, mask_close_kernel_);

    // 3. get masked v channel from hsv image
    cv::Mat im_hsv_masked, im_hsv_split[3];
    im_hsv.copyTo(im_hsv_masked, hsv_mask);
    cv::split(im_hsv_masked, im_hsv_split);
    im_v = im_hsv_split[2];

    // 4. find center of mass on each column
    std::vector<cv::Point2f> CoM_pts;
    Eigen::MatrixXd ch_v;
    cv::cv2eigen(im_v, ch_v);
    val_min_ = ch_v.maxCoeff() * val_ratio_;
    val_min_ = (val_min_ > 0) ? val_min_ : 10;

    if (this->find_laser_per_column)
        findCoMColumn(ch_v, CoM_pts);
    else
        findCoMRow(ch_v, CoM_pts);

    // 5. reject outlier
    rejectOutliers(CoM_pts, laser_pts);
    //laser_pts = CoM_pts;

    // 6. visualize
    if (f_vis_)
    {
        visualize(im, hsv_mask, im_v, laser_pts);
    }

    return laser_pts.size() >= MIN_LASER_PTS;
}

bool LaserDetector::findCoMColumn(Eigen::MatrixXd &im,
                                        std::vector<cv::Point2f> &CoM_pts)
{
    Eigen::VectorXi col_max(im.cols());
    Eigen::VectorXi val_max(im.cols());

    Eigen::MatrixXf::Index max_index;
    for (int cc = 0; cc < im.cols(); cc++)
    {
        val_max[cc] = (int)im.col(cc).maxCoeff(&max_index);
        col_max[cc] = (int)max_index;
        if (val_max[cc] < val_min_) // set a low threshold on dark columns
            continue;

        int j = col_max[cc] - 1, k = col_max[cc] + 1;
        while (j >= 0 && im(j--, cc) > 0.8 * val_max[cc])
            ;
        while (k < im.rows() && im(k++, cc) > 0.8 * val_max[cc])
            ;

        double weighed_sum = 0., val_sum = 0.;

        for (int rr = j + 1; rr < k; rr++)
        {
            weighed_sum += im(rr, cc) * (rr - j);
            val_sum += im(rr, cc);
        }

        cv::Point2f laser_pt(cc + laser_ROI_.x,
                             int(weighed_sum / val_sum + j) + laser_ROI_.y);
        CoM_pts.push_back(laser_pt);
    }

    return !CoM_pts.empty();
}

bool LaserDetector::findCoMRow(Eigen::MatrixXd &im,
                                     std::vector<cv::Point2f> &CoM_pts)
{
    Eigen::VectorXi row_max(im.rows());
    Eigen::VectorXi val_max(im.rows());

    Eigen::MatrixXf::Index max_index;
    for (int rr = 0; rr < im.rows(); rr++)
    {
        val_max[rr] = (int)im.row(rr).maxCoeff(&max_index);
        row_max[rr] = (int)max_index;
        if (val_max[rr] < val_min_) // set a low threshold on dark columns
            continue;

        int j = row_max[rr] - 1, k = row_max[rr] + 1;
        while (j >= 0 && im(rr, j--) > 0.8 * val_max[rr])
            ;
        while (k < im.cols() && im(rr, k++) > 0.8 * val_max[rr])
            ;

        double weighed_sum = 0., val_sum = 0.;

        for (int cc = j + 1; cc < k; cc++)
        {
            weighed_sum += im(rr, cc) * (cc - j);
            val_sum += im(rr, cc);
        }

        cv::Point2f laser_pt(float(weighed_sum / val_sum + j + laser_ROI_.x),
                             float(rr + laser_ROI_.y));
        CoM_pts.push_back(laser_pt);
    }

    return !CoM_pts.empty();
}

void LaserDetector::rejectOutliers(const std::vector<cv::Point2f> &pts_in,
                                         std::vector<cv::Point2f> &pts_out)
{
    pts_out.clear();
    pts_out.reserve(pts_in.size());

    int seg_cnt = 0;
    for (size_t i = 0; i < pts_in.size(); i++)
    {
        size_t j = i;
        while (i != pts_in.size() && fabs(pts_in[i + 1].x - pts_in[i].x) < 3 && fabs(pts_in[i + 1].y - pts_in[i].y) < 3)
            i++;

        if (i - j + 1 >= 40) // minimum number of points in a segment
        {
            seg_cnt++;
            pts_out.insert(pts_out.end(), pts_in.begin() + j, pts_in.begin() + i + 1);
        }
    }
}

void LaserDetector::generateHSVMasks(const cv::Mat &im_hsv, cv::Mat &hsv_mask) const
{
    // get max value (intensity) of image
    cv::Mat v_channel;
    cv::extractChannel(im_hsv, v_channel, 2);
    double val_max, tmp;
    cv::minMaxIdx(v_channel, &tmp, &val_max);

    // compute hsv mask
    double val_min = std::max(val_min_, val_max * val_ratio_);
    if (hue_min_ > hue_max_) // red color that covers hue value = 180/0
    {
        cv::Mat hsv_mask_1, hsv_mask_2;
        cv::inRange(im_hsv, cv::Scalar(0, sat_min_, val_min),
                    cv::Scalar(hue_max_, 255, 255), hsv_mask_1);
        cv::inRange(im_hsv, cv::Scalar(hue_min_, sat_min_, val_min),
                    cv::Scalar(180, 255, 255), hsv_mask_2);
        cv::bitwise_or(hsv_mask_1, hsv_mask_2, hsv_mask);
    }
    else
    {
        cv::inRange(im_hsv, cv::Scalar(hue_min_, sat_min_, val_min),
                    cv::Scalar(hue_max_, 255, 255), hsv_mask);
    }
}

void LaserDetector::visualize(const cv::Mat &im_ori, const cv::Mat &hsv_mask,
                                    const cv::Mat &im_v,
                                    const std::vector<cv::Point2f> &laser_pixels) const
{
    // get hsv mask and im_v in original sized image
    cv::Mat hsv_mask_large(img_height_, img_width_, CV_8UC1, 0.0);
    cv::Mat im_v_large(img_height_, img_width_, CV_8UC1, 0.0);
    hsv_mask.copyTo(hsv_mask_large(laser_ROI_));
    im_v.copyTo(im_v_large(laser_ROI_));

    // compose original image with colored masks
    cv::Mat im_black(img_height_, img_width_, CV_8UC1, 0.0);

    std::vector<cv::Mat> v_hsv_mask_green, v_fisheye_mask_blue;
    cv::Mat hsv_mask_green, mask_color, im_mask;

    v_hsv_mask_green.push_back(im_black);
    v_hsv_mask_green.push_back(hsv_mask_large);
    v_hsv_mask_green.push_back(im_black);
    cv::merge(v_hsv_mask_green, hsv_mask_green);

    // generate blue ROI mask
    cv::Mat roi_mask_blue(img_height_, img_width_, CV_8UC3, cv::Scalar(0, 0, 0));
    roi_mask_blue(laser_ROI_).setTo(cv::Scalar(255, 0, 0));
    mask_color = hsv_mask_green + roi_mask_blue;
    /*
  v_fisheye_mask_blue.push_back(fisheye_mask_);
  v_fisheye_mask_blue.push_back(im_black);
  v_fisheye_mask_blue.push_back(im_black);
  cv::merge(v_fisheye_mask_blue, fisheye_mask_blue);
  mask_color = hsv_mask_green + fisheye_mask_blue;
  */
    mask_color = hsv_mask_green;
    cv::addWeighted(im_ori, 0.8, mask_color, 0.2, 0.0, im_mask);

    // draw image with laser points
    cv::Mat im_laser;
    im_ori.copyTo(im_laser);
    for (const auto &laser_pixel : laser_pixels)
        cv::circle(im_laser, laser_pixel, 0, cv::Scalar(255, 0, 0), 10);

    // concat four visualization images
    cv::Mat im_hconcat1, im_hconcat2, im_concat;
    cv::Mat im_v_C3;
    std::vector<cv::Mat> v_im_v_C3{im_v_large, im_v_large, im_v_large};
    cv::merge(v_im_v_C3, im_v_C3);
    cv::hconcat(im_ori, im_mask, im_hconcat1);
    cv::hconcat(im_v_C3, im_laser, im_hconcat2);
    cv::vconcat(im_hconcat1, im_hconcat2, im_concat);

    // put text on image
    cv::putText(im_concat, "(a) Raw image",
                cv::Point2i(10, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(im_concat, "(b) HSV & ROI masks",
                cv::Point2i(img_width_ + 10, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(im_concat, "(c) Masked-out intensity image",
                cv::Point2i(10, img_height_ + 40),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(im_concat, "(d) Laser detection result",
                cv::Point2i(img_width_ + 10, img_height_ + 40),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);

    cv::namedWindow("laser detection visualization", cv::WINDOW_NORMAL);
    cv::resizeWindow("laser detection visualization", 800, 400);
    cv::imshow("laser detection visualization", im_concat);
    
    cv::waitKey(30);
    cv::imwrite("/home/wu/Documents/laser_ring_detect_vis.png", im_concat);
}