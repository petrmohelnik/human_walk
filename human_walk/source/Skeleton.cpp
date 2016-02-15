#include "Skeleton.h"

void Skeleton::init()
{
	countGlobalMatrices();

	//count rotation in the knee at the beggining
	//neccessary because i get rotation from origin, but i dont get transformation matrix at origin
	float L = glm::length(glm::vec3(globalMat[THIGH_L][3] - globalMat[FOOT_L][3]));
	float L_1 = glm::length(glm::vec3(globalMat[THIGH_L][3] - globalMat[SHIN_L][3]));
	float L_2 = glm::length(glm::vec3(globalMat[SHIN_L][3] - globalMat[FOOT_L][3]));
	float phi = PI - acos((L_1*L_1 + L_2*L_2 - L*L) / (2 * L_1*L_2));
	kneeRot_L = phi;

	L = glm::length(glm::vec3(globalMat[THIGH_R][3] - globalMat[FOOT_R][3]));
	L_1 = glm::length(glm::vec3(globalMat[THIGH_R][3] - globalMat[SHIN_R][3]));
	L_2 = glm::length(glm::vec3(globalMat[SHIN_R][3] - globalMat[FOOT_R][3]));
	phi = PI - acos((L_1*L_1 + L_2*L_2 - L*L) / (2 * L_1*L_2));
	kneeRot_R = phi;

	width_L = globalMat[FOOT_L][3].x;
	width_R = globalMat[FOOT_R][3].x;

	thighBindMat_L = globalMat[THIGH_L];
	thighBindMat_R = globalMat[THIGH_R];
}

void Skeleton::onUpdate(float dt)
{
	t += dt;
	setLeftAnklePos(glm::vec3(cos(t) *0.4+ width_L, cos(t)*0.3 + 0.5, sin(t) *0.3));
	setRightAnklePos(glm::vec3(width_R, sin(t)*0.4 + 0.49, -0.084));
	
	countGlobalMatrices();
}

int Skeleton::addBone(glm::mat4 m, int p)
{
	bones.push_back(Bone(m, p));
	globalMat.push_back(glm::mat4());
	if (p >= 0)
		bones[p].childs.push_back(bones.size() - 1);

	return bones.size() - 1;
}

std::vector<glm::mat4> Skeleton::getScaledGlobalMatrices()
{
	std::vector<glm::mat4> vec;

	for (unsigned int i = 0; i < bones.size(); i++)	{
		vec.push_back(glm::scale(globalMat[i], glm::vec3(bones[i].scale)));
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
			globalMat[i] = bones[i].localMat;
		else
			globalMat[i] = globalMat[bones[i].parent] * bones[i].localMat;
	}
}

void Skeleton::fixScale()
{
	countGlobalMatrices();

	for (unsigned int i = 0; i < bones.size(); i++) {
		if (bones[i].childs.size() != 0)
			bones[i].scale = glm::length(glm::vec3((globalMat[bones[i].childs[0]][3] - globalMat[i][3])));
		else
			bones[i].scale = bones[bones[i].parent].scale * 0.5f;
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
		vec.push_back(globalMat[i]);
	}

	return vec;
}

std::vector<glm::mat4> Skeleton::getSkinningMatrices()
{
	std::vector<glm::mat4> vec;

	for (unsigned int i = 0; i < bones.size(); i++)	{
		vec.push_back(globalMat[i] * bones[i].inverseBindMatrix);
	}

	return vec;
}

std::vector<glm::mat3> Skeleton::getTISkinningMatrices()
{
	std::vector<glm::mat3> vec;

	for (unsigned int i = 0; i < bones.size(); i++)	{
		vec.push_back(glm::transpose(glm::inverse(glm::mat3(globalMat[i]) * glm::mat3(bones[i].inverseBindMatrix))));
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

void Skeleton::setLeftAnklePos(glm::vec3 desiredPos)
{
	solveLegIK(desiredPos, THIGH_L, SHIN_L, FOOT_L, kneeRot_L, thighBindMat_L);
}

void Skeleton::setRightAnklePos(glm::vec3 desiredPos)
{
	solveLegIK(desiredPos, THIGH_R, SHIN_R, FOOT_R, kneeRot_R, thighBindMat_R);
}

void Skeleton::solveLegIK(glm::vec3 desiredPos, int sphericalJoint, int hingeJoint, int endEffector, float &actPhi, glm::mat4 thighBindMat)
{
	//transformation of spherical joint so it points at desired position
	glm::vec3 direction(glm::normalize(glm::vec3(glm::inverse(globalMat[sphericalJoint]) * glm::vec4(desiredPos, 1.0))));
	glm::vec3 up = glm::vec3(0.0, 0.0, -1.0);
	glm::vec3 right = glm::normalize(glm::cross(up, direction));
	up = glm::normalize(glm::cross(right, direction));
	glm::mat4 mat(right.x, right.y, right.z, 0.0f,
		direction.x, direction.y, direction.z, 0.0f,
		up.x, up.y, up.z, 0.0f,
		0.0, 0.0, 0.0, 1.0f);

	bones[sphericalJoint].localMat = bones[sphericalJoint].localMat * mat;

	//angles in triangle made of joints and line between spherical joint and desired pos
	float y = globalMat[sphericalJoint][3].y - desiredPos.y, z = desiredPos.z - globalMat[sphericalJoint][3].z;
	float L = glm::length(glm::vec3(globalMat[sphericalJoint][3] - glm::vec4(desiredPos, 1.0)));
	float L_1 = glm::length(glm::vec3(globalMat[sphericalJoint][3] - globalMat[hingeJoint][3]));
	float L_2 = glm::length(glm::vec3(globalMat[hingeJoint][3] - globalMat[endEffector][3]));
	float phi_t = atan(z/y);
	float phi_1 = acos((L_1*L_1 + L*L - L_2*L_2) / (2*L_1*L));
	float phi_2 = PI - acos((L_1*L_1 + L_2*L_2 - L*L) / (2*L_1*L_2));

	//transformation of joints
	bones[sphericalJoint].localMat = glm::rotate(bones[sphericalJoint].localMat, -phi_1, glm::vec3(1.0, 0.0, 0.0));
	bones[hingeJoint].localMat = glm::rotate(bones[hingeJoint].localMat, phi_2 - actPhi, glm::vec3(1.0, 0.0, 0.0));
	actPhi = phi_2;

	//count angle delta to rotate around vector from spherical joint and desired pos, so thigh is as close as possible to reference position
	//reference position is thigh in 2D (rotated only around x) rotated by the same angle plus a bit more to keep leg point forward
	glm::vec3 a = glm::normalize(glm::vec3(glm::vec4(desiredPos, 1.0) - globalMat[sphericalJoint][3]));
	glm::vec3 v = glm::normalize(glm::vec3(bones[HIPS].localMat * bones[sphericalJoint].localMat * glm::vec4(0.0, 1.0, 0.0, 1.0) - globalMat[sphericalJoint][3]));
	glm::vec3 k = glm::normalize(glm::vec3(glm::rotate(thighBindMat, -(phi_1 + phi_t + 0.6f), glm::vec3(1.0, 0.0, 0.0)) * glm::vec4(0.0, 1.0, 0.0, 1.0) - globalMat[sphericalJoint][3]));
	float A = glm::dot(glm::dot(a, v) * a, k) - glm::dot(glm::cross(glm::cross(a, v), a), k);
	float B = glm::dot(glm::cross(a, v), k);
	float C = glm::dot(v, k);

	float delta = atan2(2 * B, C - A);
	if ((((A - C) / 2.0) * cos(delta) - B * sin(delta)) > 0.0)
		delta += PI;

	//rotate by delta
	bones[sphericalJoint].localMat = glm::rotate(bones[sphericalJoint].localMat, phi_1, glm::vec3(1.0, 0.0, 0.0));
	bones[sphericalJoint].localMat = glm::rotate(bones[sphericalJoint].localMat, delta, glm::vec3(0.0, 1.0, 0.0));
	bones[sphericalJoint].localMat = glm::rotate(bones[sphericalJoint].localMat, -phi_1, glm::vec3(1.0, 0.0, 0.0));
}
