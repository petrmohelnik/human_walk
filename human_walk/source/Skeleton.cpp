#include "Skeleton.h"

void Skeleton::init()
{
	countGlobalMatrices();

	rightLeg = Leg(&bones[HIPS], &bones[THIGH_R], &bones[SHIN_R], &bones[FOOT_R], &bones[TOE_R], 2.0f);
	leftLeg = Leg(&bones[HIPS], &bones[THIGH_L], &bones[SHIN_L], &bones[FOOT_L], &bones[TOE_L], 2.0f);

	pelvisVerticalControlPointsVec.resize(4);
	for (auto i = 0; i < pelvisVerticalControlPointsVec.size(); i++)
		pelvisVerticalControlPointsVec[i] = pelvisVerticalControlPoints[i] + /*(bones[FOOT_R].globalMat[3].y + rightLeg.getLegLength() - bones[THIGH_R].globalMat[3].y)*/ + bones[HIPS].globalMat[3].y; // leftLeg.getLegLength();

	pelvisLateralControlPointsVec.resize(4);
	for (auto i = 0; i < pelvisLateralControlPointsVec.size(); i++)
		pelvisLateralControlPointsVec[i] = pelvisLateralControlPoints[i] + bones[HIPS].globalMat[3].x; // leftLeg.getLegLength();

	glm::vec3 hipsCenter = glm::vec3(bones[THIGH_R].globalMat[3] + bones[THIGH_L].globalMat[3]);

	bones[HIPS].localMat[3].x = bezierCurve(pelvisLateralControlPointsVec, 2.0 * MID_STANCE);
	bones[HIPS].localMat[3].y = bezierCurve(pelvisVerticalControlPointsVec, 2.0 * LOADING_RESPONSE);

	countGlobalMatrices();

	rightLeg.init(INIT_HEEL_STRIKE, hipsCenter);
	leftLeg.init(INIT_TERMINAL_STANCE, hipsCenter);
}

void Skeleton::onUpdate(float dt)
{
	dt *= 0.5f;
	t += dt;

	if (t > TERMINAL_SWING)
		t -= TERMINAL_SWING;

	//leftLeg.savePos();
	//rightLeg.savePos();

	//setLeftAnklePos(glm::vec3(cos(t) *0.4+ width_L, cos(t)*0.3 + 0.5, sin(t) *0.3));
	//setRightAnklePos(glm::vec3(width_R, sin(t)*0.4 + 0.49, -0.084));
	float pelvicVerticalT = t - LOADING_RESPONSE * 0.5 + 1.0;
	//pelvicT = pelvicT - (long)pelvicT;
	pelvicVerticalT -= static_cast<int>(pelvicVerticalT / 0.5) * 0.5;
	pelvicVerticalT = pelvicVerticalT > 0.25 ? 1.0 - 4.0 * (pelvicVerticalT - 0.25) : 4.0 * pelvicVerticalT;

	float pelvicLateralT = t - MID_STANCE + 1.0;
	//pelvicT = pelvicT - (long)pelvicT;
	pelvicLateralT -= static_cast<int>(pelvicLateralT);
	pelvicLateralT = pelvicLateralT > 0.5 ? 1.0 - 2.0 * (pelvicLateralT - 0.5) : 2.0 * pelvicLateralT;

	bones[HIPS].localMat[3].x = bezierCurve(pelvisLateralControlPointsVec, pelvicLateralT);
	bones[HIPS].localMat[3].y = bezierCurve(pelvisVerticalControlPointsVec, pelvicVerticalT);
	bones[HIPS].localMat[3].z += dt * 2.0;

	countGlobalMatrices();
	leftLeg.update(dt);
	rightLeg.update(dt);
	
	countGlobalMatrices();
}

int Skeleton::addBone(glm::mat4 m, int p)
{
	bones.push_back(Bone(m, p >= 0 ? &bones[p] : nullptr));
	if (p >= 0)
		bones[p].childs.push_back(&bones.back());

	return bones.size() - 1;
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
		if (bones[i].parent == nullptr)
			bones[i].globalMat = bones[i].localMat;
		else
			bones[i].globalMat = bones[i].parent->globalMat * bones[i].localMat;
	}
}

void Skeleton::fixScale()
{
	countGlobalMatrices();

	for (unsigned int i = 0; i < bones.size(); i++) {
		if (bones[i].childs.size() != 0)
			bones[i].scale = glm::length(glm::vec3(bones[i].childs[0]->globalMat[3] - bones[i].globalMat[3]));
		else
			bones[i].scale = bones[i].parent->scale * 0.5f;
	}

	/*for (unsigned int i = 0; i < bones.size(); i++) {
		if (bones[i].name.find("empty") != std::string::npos) {
			bones[bones[i].parent].childs.pop_back();
			for (unsigned int j = 0; j < bones.size(); j++) {
				if (bones[j].parent > (int)i)
					bones[j].parent--;
				for (unsigned int k = 0; k < bones[j].childs.size(); k++) {
					if (bones[j].childs[k] > (int)i)
						bones[j].childs[k]--;
				}
			}
			bones.erase(bones.begin() + i);
			i--;
		}
	}*/

	/*bones[0].localMat = glm::rotate(bones[0].localMat, -0.7f, glm::vec3(1.0, 0.0, 0.0));
	bones[1].localMat = glm::rotate(bones[1].localMat, 0.7f, glm::vec3(1.0, 0.0, 0.0));
	bones[2].localMat = glm::rotate(bones[2].localMat, -0.7f, glm::vec3(1.0, 0.0, 0.0));
	bones[2].localMat = glm::rotate(bones[2].localMat, -0.3f, glm::vec3(0.0, 0.0, 1.0));
	bones[4].localMat = glm::rotate(bones[4].localMat, 0.4f, glm::vec3(0.0, 1.0, 0.0));
	bones[13].localMat = glm::rotate(bones[13].localMat, -0.6f, glm::vec3(1.0, 0.0, 0.0));
	bones[getBoneByName("Bone_007")].localMat = glm::rotate(bones[getBoneByName("Bone_007")].localMat, 0.5f, glm::vec3(1.0, 0.0, 0.0));
	bones[getBoneByName("Bone_009")].localMat = glm::rotate(bones[getBoneByName("Bone_009")].localMat, 1.5f, glm::vec3(0.0, 0.0, 1.0));
	bones[getBoneByName("Bone_019")].localMat = glm::rotate(bones[getBoneByName("Bone_019")].localMat, -0.7f, glm::vec3(1.0, 0.0, 0.0));
	bones[getBoneByName("Bone_023")].localMat = glm::rotate(bones[getBoneByName("Bone_023")].localMat, -0.3f, glm::vec3(1.0, 0.0, 0.0));
	bones[getBoneByName("Bone_023")].localMat = glm::rotate(bones[getBoneByName("Bone_023")].localMat, -0.5f, glm::vec3(0.0, 0.0, 1.0));*/
	//bones[THIGH_L].localMat = glm::rotate(bones[THIGH_L].localMat, -1.0f, glm::vec3(1.0, 0.0, 0.0));
	//bones[SHIN_L].localMat = glm::rotate(bones[SHIN_L].localMat, 1.0f, glm::vec3(1.0, 0.0, 0.0));
}

/*int Skeleton::getBoneByName(const char *n) {
	for (unsigned int i = 0; i < bones.size(); i++) {
		if (bones[i].name.compare(n) == 0)
			return i;
	}

	return -1;
}*/

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

std::vector<glm::mat3> Skeleton::getTISkinningMatrices()
{
	std::vector<glm::mat3> vec;

	for (unsigned int i = 0; i < bones.size(); i++)	{
		vec.push_back(glm::transpose(glm::inverse(glm::mat3(bones[i].globalMat) * glm::mat3(bones[i].inverseBindMatrix))));
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