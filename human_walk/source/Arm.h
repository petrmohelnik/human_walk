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

const float zControlPointsVecPoints[4]{-0.2f, -0.02f, 0.12f, 0.3f};
const float yControlPointsVecPoints[4]{0.03f, -0.07f, -0.003f, 0.1f};
const float xControlPointsVecPoints[4]{1.0f, 0.96f, 0.90f, 0.80f};
const float smoothCurveControlPointsVecPoints[4]{0.0f, 0.0f, 1.0f, 1.0f};
const float elbowControlPoints[2][12]{{0.0f, 0.266f, 0.3634f, 0.4027f, 0.4027f, 0.6130f, 0.6294f, 0.7276f, 0.7276f, 0.7734f, 0.9143f, 1.0f},
{0.0205f, -0.1129f, 0.4264f, 0.4027f, 0.4027f, 0.3863f, 0.2774f, 0.1465f, 0.1465f, 0.0843f, 0.0555f, 0.0205f}};
const float shoulderControlPoints[2][8]{{0.0f, 0.185f, 0.3806f, 0.5058f, 0.5058f, 0.5909f, 0.8091f, 1.0f},
{-0.4378f, -0.4534f, 0.1858f, 0.117f, 0.117f, 0.0728f, -0.4217f, -0.4378f}};

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
	//glm::mat4 rootStaticMat; //matrix of root without tilting/rotating of pelvis
	/*std::vector<float> zControlPointsVec;
	std::vector<float> yControlPointsVec;
	std::vector<float> xControlPointsVec;*/
	//std::vector<float> smoothCurveControlPointsVec;
	BezierCurve elbowCurve;
	BezierCurve shoulderCurve;
	std::shared_ptr<const glm::mat4> staticRootMat;

	void solveIK(glm::vec3 desiredPos);
	void move(float prevRot, float actRot, float prevTilt, float actTilt);
public:
	Arm() = default;
	Arm(Bone *neck, Bone *sphericalJoint, Bone *hingeJoint, Bone *endEffector, float length, std::shared_ptr<const glm::mat4> staticRootM);
	void init(float time, float actRot, float actTilt);
	void update(float dt, float prevRot, float actRot, float prevTilt, float actTilt);
//	void translateStaticRoot(glm::vec3 t);
	//void fixShoulderRotationAndTilt(float prevRot, float actRot, float prevTilt, float actTilt);
	void setShoulderCoeff(float c) { shoulderCurve.setCoeff(c); }
	float getShoulderCoeff() { return shoulderCurve.getCoeff(); }
	void setElbowCoeff(float c) { elbowCurve.setCoeff(c); }
	float getElbowCoeff() { return elbowCurve.getCoeff(); }
	void incrementWidth(float i);
};

#endif //ARM_H