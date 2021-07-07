# include "PTXReader.hpp"


PTXReader::PTXReader(Eigen::Matrix<float, Eigen::Dynamic, 6> &xyzrgbMat, std::string inputFile):
xyzrgbMat(xyzrgbMat), filename(inputFile)
//data(outputData), colors(outputColor), filename(inputFile)
{
    // Get data from header of file
    filestream.open(filename,std::ios::in);
    std::string line;
    safeGetline(filestream, line);
    int nCols = stoi(line);
    safeGetline(filestream, line);
    int nRows = stoi(line);
    unsigned int nElems = nCols*nRows;

    xyzrgbMat.resize(nElems, 6);
    
    // Jump over the extrinsics block
    for (uint i = 0; i < 8; ++i)
    {
        safeGetline(filestream, line);
    }
   
    for (uint i = 0; i < nElems; ++i)
    {
        safeGetline(filestream, line);
        std::vector<std::string> parsed = split(line, ' ');
        xyzrgbMat.row(i) << stof(parsed[0]), stof(parsed[1]), stof(parsed[2]), stof(parsed[4]), stof(parsed[5]), stof(parsed[6]);
    }

}
PTXReader::~PTXReader() 
{
    filestream.close();
}