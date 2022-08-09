#include "mei_camera.hpp"


MeiCamera::MeiCamera(){}

MeiCamera::MeiCamera(std::string fileName)
{   
    cv::FileStorage fs;
    fs.open(fileName, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::cout<<"Cannot open camera calibration file"<<std::endl;
        exit (1);
    }

    if (!fs["model_type"].isNone())
    {
        std::string sModelType;
        fs["model_type"] >> sModelType;

        if (sModelType.compare("MEI") != 0)
        {
            std::cout<<"Wrong camera model"<<std::endl;
            exit (1);
        }
    }

    cv::FileNode n = fs["mirror_parameters"];
    this->xi = static_cast<double>(n["xi"]);

    n = fs["distortion_parameters"];
    this->k1 = static_cast<double>(n["k1"]);
    this->k2 = static_cast<double>(n["k2"]);
    this->p1 = static_cast<double>(n["p1"]);
    this->p2 = static_cast<double>(n["p2"]);

    n = fs["projection_parameters"];
    this->gamma1 = static_cast<double>(n["gamma1"]);
    this->gamma2 = static_cast<double>(n["gamma2"]);
    this->u0 = static_cast<double>(n["u0"]);
    this->v0 = static_cast<double>(n["v0"]);
}

/** 
 * From CataCamera.cc - CameraModel
 * \brief Lifts a point from the image plane to its projective ray
 *
 * \param p image coordinates
 * \param P coordinates of the projective ray
 */
void MeiCamera::liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P) const
{
    double mx_d, my_d, mx2_d, mxy_d, my2_d, mx_u, my_u;
    double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
    //double lambda;

    // Lift points to normalised plane
    double m_inv_K11 = 1.0 / this->gamma1;
    double m_inv_K13 = -this->u0 / this->gamma1;
    double m_inv_K22 = 1.0 / this->gamma2;
    double m_inv_K23 = -this->v0 / this->gamma2;

    mx_d = m_inv_K11 * p(0) + m_inv_K13;
    my_d = m_inv_K22 * p(1) + m_inv_K23;


    // Recursive distortion model
    int n = 8;
    Eigen::Vector2d d_u;
    distortion(Eigen::Vector2d(mx_d, my_d), d_u);
    // Approximate value
    mx_u = mx_d - d_u(0);
    my_u = my_d - d_u(1);

    for (int i = 1; i < n; ++i)
    {
        distortion(Eigen::Vector2d(mx_u, my_u), d_u);
        mx_u = mx_d - d_u(0);
        my_u = my_d - d_u(1);
    }

    // Obtain a projective ray
    double xi = this->xi;
    if (xi == 1.0)
    {
        P << mx_u, my_u, (1.0 - mx_u * mx_u - my_u * my_u) / 2.0;
    }
    else
    {
        // Reuse variable
        rho2_d = mx_u * mx_u + my_u * my_u;
        P << mx_u, my_u, 1.0 - xi * (rho2_d + 1.0) / (xi + sqrt(1.0 + (1.0 - xi * xi) * rho2_d));
    }
}

/** 
 * \brief Apply distortion to input point (from the normalised plane)
 *  
 * \param p_u undistorted coordinates of point on the normalised plane
 * \return to obtain the distorted point: p_d = p_u + d_u
 */
void MeiCamera::distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u) const
{
    double k1 = this->k1;
    double k2 = this->k2;
    double p1 = this->p1;
    double p2 = this->p2;

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
        p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}