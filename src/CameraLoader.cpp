# include "CameraLoader.hpp"

CameraLoader::CameraLoader(std::string inputFile)
{
    filestream.open(inputFile,std::ios::in);
    std::string line;

    safeGetline(filestream, line); //Line 1
    safeGetline(filestream, line); //Line 2

    // Polynomial coefficients for the DIRECT mapping function (ocam_model.ss in MATLAB). These are used by cam2world
    safeGetline(filestream, line); //Line 3
    std::vector<std::string> parsed = split(line, ' ');
    for (std::size_t i = 0; i < std::stod(parsed[0]); ++i)
    {
        model.pol[i] = std::stod(parsed[i+1]);
        //std::cout << model.pol[i] << std::endl;
    }

    safeGetline(filestream, line); //Line 4
    safeGetline(filestream, line); //Line 5
    safeGetline(filestream, line); //Line 6

    // Polynomial coefficients for the inverse mapping function (ocam_model.invpol in MATLAB). These are used by world2cam
    safeGetline(filestream, line); //Line 7
    parsed = split(line, ' ');
    for (int i = 0; i < std::stod(parsed[0]); ++i)
    {
        model.invPol.emplace_back(std::stod(parsed[i+1]));
    }

    safeGetline(filestream, line); //Line 8
    safeGetline(filestream, line); //Line 9
    safeGetline(filestream, line); //Line 10

    // Center: "row" and "column", starting from 0 (C convention)
    safeGetline(filestream, line); //Line 11
    parsed = split(line, ' ');
    model.xc = std::stod(parsed[0]);
    model.yc = std::stod(parsed[1]);

    safeGetline(filestream, line); //Line 12
    safeGetline(filestream, line); //Line 13
    safeGetline(filestream, line); //Line 14

    // Affine parameters "c", "d", "e"
    safeGetline(filestream, line); //Line 15
    parsed = split(line, ' ');
    model.c = std::stod(parsed[0]);
    model.d = std::stod(parsed[1]);
    model.e = std::stod(parsed[2]);

    safeGetline(filestream, line); //Line 16
    safeGetline(filestream, line); //Line 17
    safeGetline(filestream, line); //Line 18

    // Image size: "height" and "width"
    safeGetline(filestream, line); //Line 19
    parsed = split(line, ' ');
    height = std::stod(parsed[0]);
    width = std::stod(parsed[1]);

    safeGetline(filestream, line); //Line 20 
    safeGetline(filestream, line); //Line 21
    parsed = split(line, ' ');
    model.fov = std::stod(parsed[0]);

    safeGetline(filestream, line); //Line 22
    safeGetline(filestream, line); //Line 23
    safeGetline(filestream, line); //Line 24
    parsed = split(line, ' ');
    cam_T_rig.matrix().row(0) << stod(parsed[0]), stod(parsed[1]), stod(parsed[2]), stod(parsed[3]);
    safeGetline(filestream, line); //Line 25
    parsed = split(line, ' ');
    cam_T_rig.matrix().row(1) << stod(parsed[0]), stod(parsed[1]), stod(parsed[2]), stod(parsed[3]);
    safeGetline(filestream, line); //Line 26
    parsed = split(line, ' ');
    cam_T_rig.matrix().row(2) << stod(parsed[0]), stod(parsed[1]), stod(parsed[2]), stod(parsed[3]);
    safeGetline(filestream, line); //Line 27
    parsed = split(line, ' ');
    cam_T_rig.matrix().row(3) << stod(parsed[0]), stod(parsed[1]), stod(parsed[2]), stod(parsed[3]);


    safeGetline(filestream, line); //Line 28
    safeGetline(filestream, line); //Line 29
    parsed = split(line, ' ');
    camId = parsed[0];

}

void CameraLoader::word2cam(Eigen::Matrix<float, Eigen::Dynamic, 6> xyzrgbMat, 
                                            Eigen::Affine3d world_T_rig,
                                            std::vector<PointF>& point2_, 
                                            std::string extensionRemoved)
{   
    size_t nPoints = xyzrgbMat.rows();
    float x, y, z, norm, angle;
    double theta, u, v;
    PointF point2;


    //std::cout << cam_T_rig.matrix() << std::endl;

    for (size_t idx = 0; idx < nPoints; ++idx)
    {
        //each point//
        Eigen::Vector3d posInWorld(xyzrgbMat(idx, 0), xyzrgbMat(idx,1), xyzrgbMat(idx,2));
        Eigen::Vector3d posInCam = cam_T_rig*(world_T_rig.inverse()*posInWorld);
        x = posInCam(0);
        y = posInCam(1);
        z = posInCam(2);

        if (x > 0)
        {
            angle = atan2(sqrt(y*y + z*z), x);
            if (angle < model.fov/2)         // When the point is inside the filed of view
            {
                norm = sqrt(z*z + y*y);
                if (norm == 0.0) {norm = FLT_EPSILON;}
                theta = atan2(-x, norm);
                double rho = 0;
                
                for (std::size_t i = 0; i < model.invPol.size(); ++i)
                {
                    rho += model.invPol[i]*pow(theta, i);
                }
                v = -z/norm*rho;
                u = -y/norm*rho;
                // Add center
                point2.v = v*model.c + u*model.d + model.xc;
                point2.u = v*model.e + u + model.yc;
                point2.x = xyzrgbMat(idx, 0);
                point2.y = xyzrgbMat(idx, 1);
                point2.z = xyzrgbMat(idx, 2);
                point2.r = xyzrgbMat(idx, 3);
                point2.g = xyzrgbMat(idx, 4);
                point2.b = xyzrgbMat(idx, 5);
                //point2_.insert({point2, pColor});
                point2_.push_back(point2);
            }
        }
    }
    /*
    for(std::size_t i = 0; i < point2_.size(); ++i) 
    {
        int col = (int)(point2_[i].u);
        int row = (int)(point2_[i].v);

        imageMat.at<cv::Vec3b>(row,col)[0] = point2_[i].b;
        imageMat.at<cv::Vec3b>(row,col)[1] = point2_[i].g;
        imageMat.at<cv::Vec3b>(row,col)[2] = point2_[i].r;

        //gtMat.at<float>(row,col) = point2_[i].dis;
    }
    //cv::Mat normGtMat;
    //gtMat.convertTo(normGtMat, CV_32F, 0.1, 0);
    //cv::bilateralFilter(imageMat, imageMat, );

    //cv::imshow( "Display window", imageMat );
    cv::imwrite(savePath, imageMat);
    

    //cv::imshow("GT window", normGtMat);
    //int k = cv::waitKey(0); // Wait for a keystroke in the window
    //if(k == 'q')
    //{
        //cv::destroyAllWindows();
    //}
    */
}

CameraLoader::~CameraLoader() 
{
    filestream.close();
}

