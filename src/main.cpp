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
    std::string resultFolderPath  = "";
    if (argn < 2)
    {
        std::cout << "Missing arguments. Default path is :\n" 
        "../ptxFolder/ \n" << std::endl;
        ptxFolderPath  = "../ptxFolder/";
        resultFolderPath = "../res/";
    }
    else{ptxFolderPath  = argv[1]; resultFolderPath = argv[2];}

    float angle = 18;// Pitch 18° up, for not capturing the big hole
    Eigen::Affine3d world_T_rig(Eigen::AngleAxis(-angle*M_PI/180, Eigen::Vector3d::UnitY()));  

    //Point3F rig = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::filesystem::path ocamCalibFileName_left ("calib_results_left.txt");
    std::filesystem::path ocamCalibFilePath_left = std::filesystem::current_path().parent_path() / ocamCalibFileName_left; 
    CameraLoader oCam_left(ocamCalibFilePath_left);

    std::filesystem::path ocamCalibFileName_right ("calib_results_right.txt");
    std::filesystem::path ocamCalibFilePath_right = std::filesystem::current_path().parent_path() / ocamCalibFileName_right; 
    CameraLoader oCam_right(ocamCalibFilePath_right);

    std::vector<PointF> l_points, r_points, l_n_r;


    std::string l_resFolder = resultFolderPath+"0/";
    std::string r_resFolder = resultFolderPath+"1/";
    std::string gt_resFolder = resultFolderPath+"gt/";


    for (const auto & entry : std::filesystem::directory_iterator(ptxFolderPath))
    {
    // Eache file correspond to one room 
        for (const auto & file: std::filesystem::directory_iterator(entry.path()))
        {
            Eigen::Matrix<float, Eigen::Dynamic, 6> xyzrgbMat;
            PTXReader ptx(xyzrgbMat, file.path());
            for(size_t angleIdx = 0; angleIdx < 5; ++angleIdx) // 5 pairs for each point of view
            {
                //std::cout << ptx.nCols << "\t" << ptx.nRows << std::endl;
                //cv::Mat gt_imageMat = cv::Mat::ones(oCam_left.height, oCam_left.width, CV_16U);
                cv::Mat gt_imageMat = cv::Mat::ones(ptx.nRows+1, ptx.nCols, CV_16U);
                //cv::Mat gt_imageMat; 
                cv::Mat l_imageMat = cv::Mat::zeros(oCam_left.height, oCam_left.width, CV_8UC3);
                cv::Mat r_imageMat = cv::Mat::zeros(oCam_right.height, oCam_right.width, CV_8UC3);

                oCam_left.word2cam(xyzrgbMat, world_T_rig, l_points);
                oCam_right.word2cam(xyzrgbMat, world_T_rig, r_points);

                std::sort(l_points.begin(), l_points.end());
                std::sort(r_points.begin(), r_points.end());

                //std::set_intersection(l_points.begin(),l_points.end(),
                            //r_points.begin(),r_points.end(),
                            //std::back_inserter(l_n_r));
                std::set_intersection(r_points.begin(),r_points.end(),
                            l_points.begin(),l_points.end(),
                            std::back_inserter(l_n_r));

                for(std::size_t i = 0; i < l_n_r.size(); ++i) 
                {
                    int col = (int)(l_n_r[i].u);
                    int row = (int)(l_n_r[i].v);
                    float x = xyzrgbMat(l_n_r[i].idx, 0);
                    float y = xyzrgbMat(l_n_r[i].idx, 1);
                    float z = xyzrgbMat(l_n_r[i].idx, 2);

                    gt_imageMat.at<uchar>(row,col) = sqrt(x*x+y*y+z*z);
                }

/*
                size_t ix = 0;
                size_t xx = 0;
                for (size_t col = 0; col < ptx.nCols; ++col)
                {
                    for(size_t row = ptx.nRows-1; row > 0; --row)
                    {
                        if (ix == l_n_r[xx].idx)
                        {
                            gt_imageMat.at<cv::Vec3d>(row,col)[0] = xyzrgbMat(l_n_r[xx].idx, 5); 
                            gt_imageMat.at<cv::Vec3d>(row,col)[1] = xyzrgbMat(l_n_r[xx].idx, 4); 
                            gt_imageMat.at<cv::Vec3d>(row,col)[2] = xyzrgbMat(l_n_r[xx].idx, 3); 
                            xx++;
                        }
                        else
                        {
                            gt_imageMat.at<cv::Vec3d>(row,col) = (0,0,0); 
                        }
                        ix++;
                    }
                }
                */

                for(std::size_t i = 0; i < r_points.size(); ++i) 
                {
                    int col = (int)(r_points[i].u);
                    int row = (int)(r_points[i].v);

                    r_imageMat.at<cv::Vec3b>(row,col)[0] = xyzrgbMat(r_points[i].idx, 5);
                    r_imageMat.at<cv::Vec3b>(row,col)[1] = xyzrgbMat(r_points[i].idx, 4);
                    r_imageMat.at<cv::Vec3b>(row,col)[2] = xyzrgbMat(r_points[i].idx, 3);
                }

                for(std::size_t i = 0; i < l_points.size(); ++i) 
                {
                    int col = (int)(l_points[i].u);
                    int row = (int)(l_points[i].v);

                    l_imageMat.at<cv::Vec3b>(row,col)[0] = xyzrgbMat(l_points[i].idx, 5);
                    l_imageMat.at<cv::Vec3b>(row,col)[1] = xyzrgbMat(l_points[i].idx, 4);
                    l_imageMat.at<cv::Vec3b>(row,col)[2] = xyzrgbMat(l_points[i].idx, 3);
                }
                //cv::normalize(gt_imageMat, norm_gt_imageMat, 0, 255, cv::NormTypes::NORM_MINMAX, CV_16UC1);
                //cv::imshow( "Display window", gtimageMat );
                //cv::imshow( "l", l_imageMat );
                //cv::imshow( "r", r_imageMat );
                //cv::waitKey(0);
                //cv::destroyAllWindows();

                cv::imwrite(l_resFolder+file.path().filename().stem().string()+std::to_string(angleIdx)+".png", l_imageMat);
                cv::imwrite(r_resFolder+file.path().filename().stem().string()+std::to_string(angleIdx)+".png", r_imageMat);
                cv::imwrite(gt_resFolder+file.path().filename().stem().string()+std::to_string(angleIdx)+".png", gt_imageMat);

                l_imageMat.release();
                r_imageMat.release();
                gt_imageMat.release();
                l_n_r.clear();
                l_points.clear();
                r_points.clear();

                if (angleIdx == 3)
                {
                    world_T_rig.setIdentity();
                    world_T_rig.rotate(Eigen::AngleAxis(-(angle+90)*M_PI/180,Eigen::Vector3d::UnitY())); 
                }
                else if (angleIdx == 4)
                {
                    world_T_rig.setIdentity();
                    world_T_rig.rotate(Eigen::AngleAxis(-M_PI/6,Eigen::Vector3d::UnitY())); 

                }
                else
                {
                    Eigen::Vector3d v(sin(angle*M_PI/180),0,cos(angle*M_PI/180));
                    world_T_rig.rotate(Eigen::AngleAxis(-M_PI/2,v));
                }
            }
            //xyzrgbMat.resize(0,0);
        }

    }

    
    
    return 0;
}
