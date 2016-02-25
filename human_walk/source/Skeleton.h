#ifndef SKELETON_H
#define SKELETON_H

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "Update.h"
#include "Leg.h"

#define HIPS 0
#define SPINE 1
#define CHEST 2
#define NECK 3
#define HEAD 4
#define SHOULDER_L 5
#define UPPER_ARM_L 6
#define FOREARM_L 7
#define HAND_L 8
#define SHOULDER_R 9
#define UPPER_ARM_R 10
#define FOREARM_R 11
#define HAND_R 12
#define THIGH_L 13
#define SHIN_L 14
#define FOOT_L 15
#define TOE_L 16
#define THIGH_R 17
#define SHIN_R 18
#define FOOT_R 19
#define TOE_R 20

const float pelvisVerticalControlPoints[4]{-0.1, -0.1, 0.0, 0.0};
const float pelvisLateralControlPoints[4]{-0.06, -0.06, 0.06, 0.06};

#define PI 3.14159265f

class Skeleton : public Update
{
private:
	std::vector<Bone> bones;
	glm::mat4 rootTransform;
//	std::vector<glm::mat4> globalMat; //globalni transformace
	//float kneeRot_L = 0.0, kneeRot_R = 0.0; //actual rotation in knee joint
	float t = 0.0; //time passed
	//float width_L = 0.0, width_R = 0.0; //distance of fot from the center
	//glm::mat4 thighBindMat_L, thighBindMat_R; //original transformation of thigh
	//void solveLegIK(glm::vec3 desiredPos, int sphericalJoint, int hingeJoint, int endEffector, float &actPhi, glm::mat4 thighBindMat);
	Leg leftLeg, rightLeg;
	std::vector<float> pelvisVerticalControlPointsVec;
	std::vector<float> pelvisLateralControlPointsVec;
public:
	void init();
	void onUpdate(float dt);
	int addBone(glm::mat4 m, int p);
	std::vector<glm::mat4> getScaledGlobalMatrices();
	std::vector<Bone> getBones();
	void countGlobalMatrices();
	void fixScale();
	//int getBoneByName(const char *n);
	Bone* getBone(int i);
	std::vector<glm::mat4> getInverseMatrices();
	std::vector<glm::mat4> getGlobalMatrices();
	std::vector<glm::mat4> getSkinningMatrices();
	std::vector<glm::mat3> getTISkinningMatrices();
	void setRootTransformMatrix(glm::mat4 m);
	glm::mat4 getRootTransformMatrix();
	void setBonesNumber(int n) { bones.reserve(n); };
	//void setLeftAnklePos(glm::vec3 desiredPos);
	//void setRightAnklePos(glm::vec3 desiredPos);
};

#endif //SKELETON_H