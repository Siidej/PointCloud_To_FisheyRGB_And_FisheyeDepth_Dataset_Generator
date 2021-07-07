# pragma once
# include <Eigen/Dense>
# include <iostream>
# include <stdio.h>
# include <string>
# include <fstream>
# include <cmath>
# include <opencv2/core/core.hpp>
# include <opencv2/opencv.hpp>
# include <opencv2/highgui/highgui.hpp>
#include <map> 
# include "utils.hpp"

struct CamModel
{
    float pol[5];
    float xc, yc, c, d, e, fov;
    std::vector<float> invPol;
};
struct PointF
{
    float u, v, r, g, b, x, y, z;
    bool operator < (const PointF& a)
    {
        return ((this->x < a.x) || (this->x == a.x && this->y < a.y) || (this->x == a.x && this->y == a.y && this->z < a.z));
    }
};
struct Point3F
{
    float x, y, z, r, g, b;
};
class CameraLoader
{
private:
    /////Extrinsics/////
    Eigen::Affine3d cam_T_rig; // Transformation Matrix
    ////////////////////
    CamModel model;
    std::string camId;
    std::ifstream filestream;

public:
    float width, height;
    CameraLoader(std::string inputFile);
    ~CameraLoader();
    void word2cam(Eigen::Matrix<float, Eigen::Dynamic, 6> xyzrgbMat, Eigen::Affine3d world_T_rig,std::vector<PointF>& point2_, std::string extensionRemoved);
};