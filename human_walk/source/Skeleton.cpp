#include "Skeleton.h"

void Skeleton::addBone(glm::mat4 m, int p, const char *n)
{
	bones.push_back(Bone(m, p, n));
	if (p >= 0)
		bones[p].childs.push_back(bones.size() - 1);
}

std::vector<glm::mat4> Skeleton::getScaledGlobalMatrices()
{
	std::vector<glm::mat4> vec;

	for (unsigned int i = 0; i < bones.size(); i++)	{
		vec.push_back(glm::scale(bones[i].globalMat, glm::vec3(bones[i].scale)));
	}

	return vec;
}

std::vector<Bone> Skeleton::getBones()
{
	return bones;
}

void Skeleton::countGlobalMatrices()
{
	for (unsigned int i = 0; i < bones.size(); i++) {
		if (bones[i].parent < 0)
			bones[i].globalMat = bones[i].localMat;
		else
			bones[i].globalMat = bones[bones[i].parent].globalMat * bones[i].localMat;
	}
}

void Skeleton::fixScale()
{
	countGlobalMatrices();

	for (unsigned int i = 0; i < bones.size(); i++) {
		if (bones[i].childs.size() != 0)
			bones[i].scale = glm::length(glm::vec3((bones[bones[i].childs[0]].globalMat[3] - bones[i].globalMat[3])));
		else
			bones[i].scale = bones[bones[i].parent].scale * 0.5;
	}

	for (unsigned int i = 0; i < bones.size(); i++) {
		if (bones[i].name.find("empty") != std::string::npos) {
			bones[bones[i].parent].childs.pop_back();
			for (unsigned int j = 0; j < bones.size(); j++) {
				if (bones[j].parent > i)
					bones[j].parent--;
				for (unsigned int k = 0; k < bones[j].childs.size(); k++) {
					if (bones[j].childs[k] > i)
						bones[j].childs[k]--;
				}
			}
			bones.erase(bones.begin() + i);
			i--;
		}
	}

	bones[0].localMat = glm::rotate(bones[0].localMat, -1.0f, glm::vec3(1.0, 0.0, 0.0));
	bones[4].localMat = glm::rotate(bones[4].localMat, 0.4f, glm::vec3(0.0, 1.0, 0.0));
	bones[13].localMat = glm::rotate(bones[13].localMat, -0.6f, glm::vec3(1.0, 0.0, 0.0));
	bones[getBoneByName("Bone_007")].localMat = glm::rotate(bones[getBoneByName("Bone_007")].localMat, 0.5f, glm::vec3(1.0, 0.0, 0.0));
	bones[getBoneByName("Bone_019")].localMat = glm::rotate(bones[getBoneByName("Bone_019")].localMat, -0.7f, glm::vec3(1.0, 0.0, 0.0));
	bones[getBoneByName("Bone_023")].localMat = glm::rotate(bones[getBoneByName("Bone_023")].localMat, -0.7f, glm::vec3(1.0, 0.0, 0.0));
	bones[getBoneByName("Bone_023")].localMat = glm::rotate(bones[getBoneByName("Bone_023")].localMat, -0.7f, glm::vec3(0.0, 0.0, 1.0));
}

int Skeleton::getBoneByName(const char *n) {
	for (unsigned int i = 0; i < bones.size(); i++) {
		if (bones[i].name.compare(n) == 0)
			return i;
	}

	return -1;
}

Bone* Skeleton::getBone(int i) 
{
	return &bones[i];
}

std::vector<glm::mat4> Skeleton::getInverseMatrices()
{
	std::vector<glm::mat4> vec;

	for (unsigned int i = 0; i < bones.size(); i++)	{
		vec.push_back(bones[i].inverseBindMatrix);
	}

	return vec;
}

std::vector<glm::mat4> Skeleton::getGlobalMatrices()
{
	std::vector<glm::mat4> vec;

	for (unsigned int i = 0; i < bones.size(); i++)	{
		vec.push_back(bones[i].globalMat);
	}

	return vec;
}

std::vector<glm::mat4> Skeleton::getSkinningMatrices()
{
	std::vector<glm::mat4> vec;

	for (unsigned int i = 0; i < bones.size(); i++)	{
		vec.push_back(bones[i].globalMat * bones[i].inverseBindMatrix);
	}

	return vec;
}

void Skeleton::setRootTransformMatrix(glm::mat4 m)
{
	rootTransform = m;
}

glm::mat4 Skeleton::getRootTransformMatrix()
{
	return  rootTransform;
}
