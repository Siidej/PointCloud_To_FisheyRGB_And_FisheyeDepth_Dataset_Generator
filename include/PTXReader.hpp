
/**
 * @brief file.ptx -> Matrix 
 */

# pragma once
//#include <Eigen/Dense>
// For debug only
# include <iostream>

// Essentials
# include <stdio.h>
# include <string>
# include <fstream>
# include <iomanip>
# include <sstream>
# include <vector>
# include <Eigen/Dense>
# include "utils.hpp"

class PTXReader {
private:
    // Variables
    std::string filename = "";
    Eigen::Matrix<float, Eigen::Dynamic, 6> &xyzrgbMat;
    std::ifstream filestream;
    
public:
    PTXReader(Eigen::Matrix<float, Eigen::Dynamic, 6> &xyzrgbMat, std::string inputFile);
    ~PTXReader();
};
