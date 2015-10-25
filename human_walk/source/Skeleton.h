#ifndef SKELETON_H
#define SKELETON_H

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

struct Bone
{
	glm::mat4 localMat;
	glm::mat4 globalMat;
	glm::mat4 inverseBindMatrix;
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
	int getBoneByName(const char *n);
	Bone* getBone(int i);
	std::vector<glm::mat4> getInverseMatrices();
	std::vector<glm::mat4> getGlobalMatrices();
};

#endif //SKELETON_H