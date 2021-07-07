# pragma once
# include <stdio.h>
# include <string>
# include <fstream>
# include <iomanip>
# include <sstream>
# include <vector>

extern const std::istream& safeGetline(std::istream& is, std::string& t);
extern const std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
extern const std::vector<std::string> split(const std::string &s, char delim);