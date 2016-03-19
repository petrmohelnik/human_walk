#ifndef ARM_H
#define ARM_H

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include "Leg.h"

#define PI 3.14159265f

#define INIT_HEEL_STRIKE 0
#define INIT_TERMINAL_STANCE 1

#define ELBOW_MINIMUM_ROT 0.523599f

const float zControlPointsVecPoints[4]{-0.2, -0.02, 0.12, 0.3};
const float yControlPointsVecPoints[4]{0.03, -0.07, -0.003, 0.1};
const float xControlPointsVecPoints[4]{1.0, 0.96, 0.90, 0.80};
const float smoothCurveControlPointsVecPoints[4]{0.0, 0.0, 1.0, 1.0};
const float elbowControlPoints[2][12]{{0.0, 0.266, 0.3634, 0.4027, 0.4027, 0.6130, 0.6294, 0.7276, 0.7276, 0.7734, 0.9143, 1.0},
{0.0205, -0.1129, 0.4264, 0.4027, 0.4027, 0.3863, 0.2774, 0.1465, 0.1465, 0.0843, 0.0555, 0.0205}};
const float shoulderControlPoints[2][8]{{0.0, 0.185, 0.3806, 0.5058, 0.5058, 0.5909, 0.8091, 1.0},
{-0.4378, -0.4534, 0.1858, 0.117, 0.117, 0.0728, -0.4217, -0.4378}};

class Arm
{
private:
	Bone *root, *upperArm, *forearm, *hand;
	float shoulderRot = 0.0, elbowRot = 0.0; //actual rotation in knee joint
	float t = 0.0; //time passed
	float width = 0.0; //distance of fot from the center
	float swingLength = 0.0;
	float armLength;
	float prevTiltSave = 0.0, prevRotSave = 0.0;
	glm::mat4 upperArmBindMat; //original transformation of thigh
	glm::mat4 rootStaticMat; //matrix of root without tilting/rotating of pelvis
	/*std::vector<float> zControlPointsVec;
	std::vector<float> yControlPointsVec;
	std::vector<float> xControlPointsVec;*/
	//std::vector<float> smoothCurveControlPointsVec;
	BezierCurve elbowCurve;
	BezierCurve shoulderCurve;

	void solveIK(glm::vec3 desiredPos);
	void move(float prevRot, float actRot, float prevTilt, float actTilt);
public:
	Arm() = default;
	Arm(Bone *neck, Bone *sphericalJoint, Bone *hingeJoint, Bone *endEffector, float length);
	void init(float time, float actRot, float actTilt);
	void update(float dt, float prevRot, float actRot, float prevTilt, float actTilt);
	void translateStaticRoot(glm::vec3 t);
	//void fixShoulderRotationAndTilt(float prevRot, float actRot, float prevTilt, float actTilt);
	void setShoulderCoeff(float c) { shoulderCurve.setCoeff(c); }
	float getShoulderCoeff() { return shoulderCurve.getCoeff(); }
	void setElbowCoeff(float c) { elbowCurve.setCoeff(c); }
	float getElbowCoeff() { return elbowCurve.getCoeff(); }
	void incrementWidth(float i);
};

#endif //ARM_H