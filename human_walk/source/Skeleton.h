#ifndef SKELETON_H
#define SKELETON_H

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "Update.h"
#include "Leg.h"
#include "Arm.h"

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

const float pelvisVerticalControlPoints[][8]{{0.0f, 0.075f, 0.175f, 0.25f, 0.25f, 0.325f, 0.425f, 0.5f},
{-0.05f, -0.05f, 0.0f, 0.0f, 0.0f, 0.0f, -0.05f, -0.05f}};
const float pelvisLateralControlPoints[][8]{{0.0f, 0.0622f, 0.175f, 0.2504f, 0.2504f, 0.3258f, 0.4378f, 0.5f},
{0.0f, -0.00902f, -0.02084f, -0.02022f, -0.02022f, -0.01975f, -0.00902f, 0.0f}};
const float pelvicRotationControlPoints[][8]{{0.0f, 0.0565f, 0.1029f, 0.1818f, 0.1818f, 0.2601f, 0.3580f, 0.5f},
{0.10442f, 0.10056f, 0.04532f, -0.00088f, -0.00088f, -0.04686f, -0.11416f, -0.10442f}};
const float shoulderRotationControlPoints[][8]{{ 0.0f, 0.1338f, 0.1617f, 0.2447f, 0.2447f, 0.2687f, 0.4602f, 0.5f },
{0.10472f, 0.107f, 0.0889f, 0.0405f, 0.0405f, 0.0274f, -0.1054f, -0.10472f }};
const float pelvicTiltControlPoints[][16]{{ 0.0f, 0.0575f, 0.0644f, 0.1042f, 0.1042f, 0.17f, 0.2664f, 0.3365f, 0.3365f, 0.4009f, 0.4312f, 0.4596f, 0.4596f, 0.4798f, 0.486f, 0.5f },
{ 0.0f, -0.022253f, -0.07505f, -0.069412f, -0.069412f, -0.060388f, 0.010908f, 0.006493f, 0.006493f, 0.002321f, -0.011135f, -0.010681f, -0.010681f, -0.01035f, -0.00541f, 0.0f}};
const float pelvicTiltForwardControlPoints[][8]{{0.0f, 0.0865f, 0.2077f, 0.2639f, 0.2639f, 0.3687f, 0.4266f, 0.5f},
{ 0.0f, 0.00423f, 0.03163f, 0.03049f, 0.03049f, 0.02859f, 0.00359f, 0.0f }};
//integrate((1-t)^3*5*^-1+3*(1-t)^2*t*5*^-1+3*(1-t)* t^2*(-19.0 / 62.0)+t^3*(-19.0 / 62.0))*derivative((1-t)^3*0+3*(1-t)^2*t*2*^-1+3*(1-t)*t^2*4*^-1+t^3*1) from t=0 to 1
//http://stackoverflow.com/a/24725575
//const float pelvisSpeedControlPoints[][8]{{ 0.0f, 0.05f, 0.1f, 0.25f, 0.25f, 0.4f, 0.45f, 0.5}f,
//{ 0.5f, 0.5f, -19.0 / 62.0f, -19.0 / 62.0f, -19.0 / 62.0f, -19.0 / 62.0f, 0.5f, 0.5 }}; //-0.31434047 -0.314332247557 -- -0.314581f, -0.3145900
const float pelvisSpeedControlPoints[][4]{{ 0.0f, 0.3f, 0.7f, 1.0f },
{ 0.0f, 0.4f, 0.6f, 1.0f }};

#define PI 3.14159265f

class Skeleton : public Update
{
private:
	std::shared_ptr<Terrain> terrain;
	std::vector<Bone> bones;
	glm::mat4 rootTransform;
	float t = 0.0;
	Leg leftLeg, rightLeg;
	Arm leftArm, rightArm;
	float pelvicRotation = 0.0, shoulderRotation = 0.0, pelvicTilt = 0.0, pelvicTiltForward = 0.0, spineRot = 0.0;// , headRot = 0.0;
	std::shared_ptr<float> maxPelvisHeight;// = -0.01;
	float stepLength = 1.4f;
	float stepSpeedAccuracyCheck = 0.0;
	std::shared_ptr<glm::mat4> staticRootMat;// , staticRootPosSpeed;
	std::shared_ptr<glm::vec3> prevRootPos, nextRootPos;
	std::shared_ptr<BezierCurve> pelvisVerticalCurve;
	BezierCurve pelvisLateralCurve;
	BezierCurve pelvicRotationCurve;
	BezierCurve shoulderRotationCurve;
	BezierCurve pelvicTiltCurve;
	BezierCurve pelvicTiltForwardCurve;
	std::shared_ptr<BezierCurve> pelvisSpeedCurve;

	float heightTestHeight = 0.0f;
	bool heightTest = false;

	float spineEquation(float T, float hipAngle, float x);
	float solveSpine(float dist, float hipAngle);
public:
	Skeleton(std::shared_ptr<Terrain> t);
	void init();
	void onUpdate(float dt);
	int addBone(glm::mat4 m, int p);
	void getScaledGlobalMatrices(std::vector<glm::mat4> &vec);
	std::vector<Bone> getBones();
	void countGlobalMatrices();
	void fixScale();
	Bone* getBone(int i);
	void getInverseMatrices(std::vector<glm::mat4> &vec);
	void getGlobalMatrices(std::vector<glm::mat4> &vec);
	void getSkinningMatrices(std::vector<glm::mat4> &vec);
	void getTISkinningMatrices(std::vector<glm::mat3> &vec);
	void setRootTransformMatrix(glm::mat4 m);
	glm::mat4 getRootTransformMatrix();
	void setBonesNumber(int n) { bones.reserve(n); };

	void setPelvicTiltCoeff(float c) { pelvicTiltCurve.setCoeff(c); }
	float getPelvicTiltCoeff() { return pelvicTiltCurve.getCoeff(); }
	void setPelvicTiltForwardCoeff(float c) { pelvicTiltForwardCurve.setCoeff(c); }
	float getPelvicTiltForwardCoeff() { return pelvicTiltForwardCurve.getCoeff(); }
	void setPelvicRotationCoeff(float c) { pelvicRotationCurve.setCoeff(c); shoulderRotationCurve.setCoeff(c); }
	float getPelvicRotationCoeff() { return pelvicRotationCurve.getCoeff(); }
	void setPelvisLateralCoeff(float c) { pelvisLateralCurve.setCoeff(c); }
	float getPelvisLateralCoeff() { return pelvisLateralCurve.getCoeff(); }
	void setPelvisVerticalCoeff(float c) { pelvisVerticalCurve->setCoeff(c); }
	float getPelvisVerticalCoeff() { return pelvisVerticalCurve->getCoeff(); }
	void increaseMaxPelvisHeight(float i) { *maxPelvisHeight += i; }
	void increaseStepWidth(float i) { leftLeg.increaseStepWidth(i); rightLeg.increaseStepWidth(i); }
	void setShoulderCoeff(float c) { leftArm.setShoulderCoeff(c); rightArm.setShoulderCoeff(c); }
	float getShoulderCoeff() { return leftArm.getShoulderCoeff(); }
	void setElbowCoeff(float c) { leftArm.setElbowCoeff(c); rightArm.setElbowCoeff(c); }
	float getElbowCoeff() { return leftArm.getElbowCoeff(); }
	void increaseArmWidth(float a) { leftArm.incrementWidth(a); rightArm.incrementWidth(a); }
	glm::vec3 getStaticRootPos() { return glm::vec3((*staticRootMat)[3]); }
	void testHeight(bool b) { heightTest = b; }
};

#endif //SKELETON_H