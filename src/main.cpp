# include <iostream>
# include "PTXReader.hpp"
# include "CameraLoader.hpp"
# include <fstream>
# include <filesystem>
# include <Eigen/Dense>
#include <Eigen/Geometry> 


int main(int argn, char *argv[])
{
    std::string ptxFolderPath  = "";
    if (argn < 2)
    {
        std::cout << "Missing arguments. Default path is :\n" 
        "../ptxFolder/ \n" << std::endl;
        ptxFolderPath  = "../ptxFolder/";
    }
    else{ptxFolderPath  = argv[1];}

    Eigen::Affine3d world_T_rig(Eigen::AngleAxis(-M_PI/6,Eigen::Vector3d::UnitY()));
    //Eigen::Vector3f translation_rigInworld (0.0, 0.0, 0.0);
    Eigen::Vector3d test(0.0, 0.5, 0.5);
    world_T_rig.rotate(Eigen::AngleAxis(-M_PI/6,test));


    //Point3F rig = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::filesystem::path ocamCalibFileName_left ("calib_results_left.txt");
    std::filesystem::path ocamCalibFilePath_left = std::filesystem::current_path().parent_path() / ocamCalibFileName_left; 
    CameraLoader oCam_left(ocamCalibFilePath_left);

    std::filesystem::path ocamCalibFileName_right ("calib_results_right.txt");
    std::filesystem::path ocamCalibFilePath_right = std::filesystem::current_path().parent_path() / ocamCalibFileName_right; 
    CameraLoader oCam_right(ocamCalibFilePath_right);

    std::vector<PointF> l_n_r, l_points, r_points;



    std::string l_resFolder = "../res/0/";
    std::string r_resFolder = "../res/1/";
    std::string gt_resFolder = "../res/gt/";


    for (const auto & entry : std::filesystem::directory_iterator(ptxFolderPath))
    {
    // Eache file correspond to one room 
        for (const auto & file: std::filesystem::directory_iterator(entry.path()))
        {
            Eigen::Matrix<float, Eigen::Dynamic, 6> xyzrgbMat;
            PTXReader ptx(xyzrgbMat, file.path());
            cv::Mat imageMat = cv::Mat::zeros(oCam_left.height, oCam_left.width, CV_16UC1);
            cv::Mat gtimageMat; 
            cv::Mat l_imageMat = cv::Mat::zeros(oCam_left.height, oCam_left.width, CV_8UC3);
            cv::Mat r_imageMat = cv::Mat::zeros(oCam_left.height, oCam_left.width, CV_8UC3);


            oCam_left.word2cam(xyzrgbMat, world_T_rig, l_points, file.path().filename().stem());
            oCam_right.word2cam(xyzrgbMat, world_T_rig, r_points, file.path().filename().stem());


            std::sort(l_points.begin(), l_points.end());
            std::sort(r_points.begin(), r_points.end());

            std::set_intersection(l_points.begin(),l_points.end(),
                          r_points.begin(),r_points.end(),
                          std::back_inserter(l_n_r));

            for(std::size_t i = 0; i < l_n_r.size(); ++i) 
            {
                int col = (int)(l_n_r[i].u);
                int row = (int)(l_n_r[i].v);

                imageMat.at<uchar>(row,col) = sqrt(l_n_r[i].x*l_n_r[i].x + l_n_r[i].y*l_n_r[i].y + l_n_r[i].z*l_n_r[i].z);
            }

            for(std::size_t i = 0; i < r_points.size(); ++i) 
            {
                int col = (int)(r_points[i].u);
                int row = (int)(r_points[i].v);

                r_imageMat.at<cv::Vec3b>(row,col)[0] = r_points[i].b;
                r_imageMat.at<cv::Vec3b>(row,col)[1] = r_points[i].g;
                r_imageMat.at<cv::Vec3b>(row,col)[2] = r_points[i].r;
            }

            for(std::size_t i = 0; i < l_points.size(); ++i) 
            {
                int col = (int)(l_points[i].u);
                int row = (int)(l_points[i].v);

                l_imageMat.at<cv::Vec3b>(row,col)[0] = l_points[i].b;
                l_imageMat.at<cv::Vec3b>(row,col)[1] = l_points[i].g;
                l_imageMat.at<cv::Vec3b>(row,col)[2] = l_points[i].r;
            }
            cv::normalize(imageMat, gtimageMat, 0, 255, cv::NormTypes::NORM_MINMAX, CV_16UC1);
            cv::imshow( "Display window", gtimageMat );
            cv::imshow( "l", l_imageMat );
            cv::imshow( "r", r_imageMat );
            cv::waitKey(0);
            cv::destroyAllWindows();

            cv::imwrite(l_resFolder.append(file.path().filename().stem())+".png", l_imageMat);
            cv::imwrite(r_resFolder.append(file.path().filename().stem())+".png", r_imageMat);
            cv::imwrite(gt_resFolder.append(file.path().filename().stem())+".png", imageMat);

            l_imageMat.release();
            r_imageMat.release();
            imageMat.release();
            l_n_r.clear();
            l_points.clear();
            r_points.clear();

        }

    }

    
    
    return 0;
}
