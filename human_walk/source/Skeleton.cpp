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

	for (unsigned int i = 0; i < bones.size(); i++)
	{
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

	bones[0].localMat = glm::rotate(bones[0].localMat, 0.4f, glm::vec3(1.0, 0.0, 0.0));
	bones[13].localMat = glm::rotate(bones[13].localMat, -0.6f, glm::vec3(1.0, 0.0, 0.0));
}
