#ifndef TERRAIN_H
#define TERRAIN_H

#include <vector>
#include <stdlib.h> 
#include "Model.h"

class Terrain
{
private:
	float z1, z2;
	float maxHeight;
	float resolution;
	float width;
	std::vector<float> y;

	void midPointStep(int i, int j, float range, float H);
public:
	Terrain(float _z1, float _z2, float maxH, float w, float res);
	void midPoint(float range, float H);
	void fillModel(Model &m, Texture &t);
	float getHeight(glm::vec3 pos);
};

#endif //TERRAIN_H