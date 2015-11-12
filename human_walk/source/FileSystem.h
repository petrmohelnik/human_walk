#ifndef FILE_SYSTEM_H
#define FILE_SYSTEM_H

#include <string>
#include <fstream>
#include <iostream>
#include <glm/glm.hpp>
#include <string>
#include <sstream>
#include <vector>
#include <stdlib.h>  
#include "Model.h"
#include "Skeleton.h"
#include "lodepng.h"

class FileSystem
{
private:
	//http://stackoverflow.com/a/236803
	std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
	std::vector<std::string> split(const std::string &s, char delim);
	int findLastOpen(std::vector<bool> &closed);
public:
	bool loadFile(const char *path, std::string &buffer);
	bool loadTexture(const char *path, std::vector<unsigned char> img);
	bool parseObj(const char *path, Model &m);
	bool loadModelAndSkeletonDae(const char *path, Model &m, Skeleton &s);
};

#endif //FILE_SYSTEM_H