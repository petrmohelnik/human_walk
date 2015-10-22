#ifndef SKELETON_H
#define SKELETON_H

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

struct Bone
{
	glm::mat4 localMat;
	glm::mat4 globalMat;
	int parent;
	std::vector<int> childs;
	std::string name;
	float scale;
	Bone(glm::mat4 mat, int parent, const char *name) : localMat(mat), parent(parent), name(name), scale(1.0f) {}
};

class Skeleton
{
private:
	std::vector<Bone> bones;
public:
	void addBone(glm::mat4 m, int p, const char *n);
	std::vector<glm::mat4> getScaledGlobalMatrices();
	std::vector<Bone> getBones();
	void countGlobalMatrices();
	void fixScale();
};

#endif //SKELETON_H