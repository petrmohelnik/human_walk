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

const float pelvisVerticalControlPoints[][8]{{0.0, 0.075, 0.175, 0.25, 0.25, 0.325, 0.425, 0.5},
{-0.05, -0.05, 0.0, 0.0, 0.0, 0.0, -0.05, -0.05}};
const float pelvisLateralControlPoints[][8]{{0.0, 0.0622, 0.175, 0.2504, 0.2504, 0.3258, 0.4378, 0.5},
{0.0, -0.00902, -0.02084, -0.02022, -0.02022, -0.01975, -0.00902, 0.0}};
const float pelvicRotationControlPoints[][8]{{0.0, 0.0565, 0.1029, 0.1818, 0.1818, 0.2601, 0.3580, 0.5},
{0.10442, 0.10056, 0.04532, -0.00088, -0.00088, -0.04686, -0.11416, -0.10442}};
const float shoulderRotationControlPoints[][8]{{ 0.0, 0.1338, 0.1617, 0.2447, 0.2447, 0.2687, 0.4602, 0.5 },
{0.10472, 0.107, 0.0889, 0.0405, 0.0405, 0.0274, -0.1054, -0.10472 }};
const float pelvicTiltControlPoints[][16]{{ 0.0, 0.0575, 0.0644, 0.1042, 0.1042, 0.17, 0.2664, 0.3365, 0.3365, 0.4009, 0.4312, 0.4596, 0.4596, 0.4798, 0.486, 0.5 },
{ 0.0, -0.022253, -0.07505, -0.069412, -0.069412, -0.060388, 0.010908, 0.006493, 0.006493, 0.002321, -0.011135, -0.010681, -0.010681, -0.01035, -0.00541, 0.0}};
const float pelvicTiltForwardControlPoints[][8]{{0.0, 0.0865, 0.2077, 0.2639, 0.2639, 0.3687, 0.4266, 0.5},
{ 0.0, 0.00423, 0.03163, 0.03049, 0.03049, 0.02859, 0.00359, 0.0 }};
//integrate((1-t)^3*5*^-1+3*(1-t)^2*t*5*^-1+3*(1-t)* t^2*(-19.0 / 62.0)+t^3*(-19.0 / 62.0))*derivative((1-t)^3*0+3*(1-t)^2*t*2*^-1+3*(1-t)*t^2*4*^-1+t^3*1) from t=0 to 1
//http://stackoverflow.com/a/24725575
const float pelvisSpeedControlPoints[][8]{{ 0.0, 0.05, 0.1, 0.25, 0.25, 0.4, 0.45, 0.5},
{ 0.5, 0.5, -19.0 / 62.0, -19.0 / 62.0, -19.0 / 62.0, -19.0 / 62.0, 0.5, 0.5 }}; //-0.31434047 -0.314332247557 -- -0.314581, -0.3145900
const float pelvisSpeedIntegralControlPoints[][4]{{ 0.0, 0.2, 0.8, 1.0},
{ 1.0, 1.0, 0.0, 0.0 }};
const float uniformX[4]{0.0, 0.333334, 0.666667, 1.0};

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
	float maxPelvisHeight = -0.01;
	float stepLength = 1.4f;
	float stepSpeedAccuracyCheck = 0.0;
	glm::vec3 staticRootPos, staticRootPosSpeed;
	glm::vec3 prevRootPos, nextRootPos;
	BezierCurve pelvisVerticalCurve;
	BezierCurve pelvisLateralCurve;
	BezierCurve pelvicRotationCurve;
	BezierCurve shoulderRotationCurve;
	BezierCurve pelvicTiltCurve;
	BezierCurve pelvicTiltForwardCurve;
	BezierCurve pelvisSpeedCurve;
	std::vector<float> uniformXVec;

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
	void setPelvisVerticalCoeff(float c) { pelvisVerticalCurve.setCoeff(c); }
	float getPelvisVerticalCoeff() { return pelvisVerticalCurve.getCoeff(); }
	void increaseMaxPelvisHeight(float i) { maxPelvisHeight += i; }
	void increaseStepWidth(float i) { leftLeg.increaseStepWidth(i); rightLeg.increaseStepWidth(i); }
	void setShoulderCoeff(float c) { leftArm.setShoulderCoeff(c); rightArm.setShoulderCoeff(c); }
	float getShoulderCoeff() { return leftArm.getShoulderCoeff(); }
	void setElbowCoeff(float c) { leftArm.setElbowCoeff(c); rightArm.setElbowCoeff(c); }
	float getElbowCoeff() { return leftArm.getElbowCoeff(); }
	void increaseArmWidth(float a) { leftArm.incrementWidth(a); rightArm.incrementWidth(a); }
	glm::vec3 getStaticRootPos() { return staticRootPos; }
	void testHeight(bool b) { heightTest = b; }
};

#endif //SKELETON_H