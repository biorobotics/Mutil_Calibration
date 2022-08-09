#ifndef MEI_H
#define MEI_H

#include <Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp>

class MeiCamera
{
public:
    MeiCamera();
    MeiCamera(std::string fileName);
    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
    void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const;

private:
    double xi;
    double k1, k2, p1, p2;
    double gamma1, gamma2, u0, v0;

};

#endif