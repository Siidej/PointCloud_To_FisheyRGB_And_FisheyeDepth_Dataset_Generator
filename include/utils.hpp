# pragma once
# include <stdio.h>
# include <string>
# include <fstream>
# include <iomanip>
# include <sstream>
# include <vector>

const std::istream& safeGetline(std::istream& is, std::string& t);
const std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
const std::vector<std::string> split(const std::string &s, char delim);
