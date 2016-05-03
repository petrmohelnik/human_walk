#ifndef SKELETON_H
#define SKELETON_H

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "Update.h"
#include "Leg.h"
#include "Arm.h"

#define PELVIS_CONFIGURATION_STEPS 25

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

const float pelvisVerticalControlPoints[][8]{{0.0f, 0.15f, 0.35f, 0.5f, 0.5f, 0.65f, 0.85f, 1.0f},
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
/*const float pelvisSpeed1ControlPoints[][4]{{ 0.0f, 0.4f, 0.7f, 1.0f },
{ 0.0f, 0.4f, 0.6f, 1.0f }};*/
const float pelvisSpeed1ControlPoints[][4]{{ 0.0f, 0.23f, 0.57f, 1.0f },
{ 0.0f, 0.33f, 0.73f, 1.0f }};
const float pelvisSpeed2ControlPoints[][4]{{ 0.0f, 0.43f, 0.77f, 1.0f },
{ 0.0f, 0.27f, 0.67f, 1.0f }};

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
	float defaultPelvisHeight;
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
	std::shared_ptr<BezierCurve> pelvisSpeed1Curve;
	std::shared_ptr<BezierCurve> pelvisSpeed2Curve;
	glm::vec3 cameraPos;
	glm::vec3 jointWeights; //hip, knee, ankle
	float timeSpeedCoeff = 1.0f;
	float rotationForward = 0.0f;

	float spineEquation(float T, float hipAngle, float x);
	float solveSpine(float dist, float hipAngle);
	void configurePelvis(Leg &stanceLeg, Leg &swingLeg);
	float pelvisSpeedCurve(float _t);
	float configurePelvisFindBestScore(Leg &stanceLeg, Leg &swingLeg, int iter, float lowRange, float highRange, float actVerticalCoeff,
		float lastVerticaCoeff);
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
	float getPelvicTiltCoeff() { return pelvicTiltCurve.getNextCoeff(); }
	void setPelvicTiltForwardCoeff(float c) { pelvicTiltForwardCurve.setCoeff(c); }
	float getPelvicTiltForwardCoeff() { return pelvicTiltForwardCurve.getNextCoeff(); }
	void setPelvicRotationCoeff(float c) { pelvicRotationCurve.setCoeff(c); shoulderRotationCurve.setCoeff(c); }
	float getPelvicRotationCoeff() { return pelvicRotationCurve.getNextCoeff(); }
	void setPelvisLateralCoeff(float c) { pelvisLateralCurve.setCoeff(c); }
	float getPelvisLateralCoeff() { return pelvisLateralCurve.getNextCoeff(); }
	void setPelvisVerticalCoeff(float c) { pelvisVerticalCurve->setCoeff(c); }
	float getPelvisVerticalCoeff() { return pelvisVerticalCurve->getNextCoeff(); }
	void setMaxPelvisHeight(float h) { *maxPelvisHeight = defaultPelvisHeight + h; }
	void increaseStepWidth(float i) { leftLeg.increaseStepWidth(i); rightLeg.increaseStepWidth(i); }
	void setStepWidth(float w) { leftLeg.setStepWidth(w); rightLeg.setStepWidth(w); }
	void setShoulderCoeff(float c) { leftArm.setShoulderCoeff(c); rightArm.setShoulderCoeff(c); }
	float getShoulderCoeff() { return leftArm.getShoulderCoeff(); }
	void setElbowCoeff(float c) { leftArm.setElbowCoeff(c); rightArm.setElbowCoeff(c); }
	float getElbowCoeff() { return leftArm.getElbowCoeff(); }
	void setArmWidth(float a) { leftArm.setWidth(a); rightArm.setWidth(a); }
	glm::vec3 getCameraPos() { return cameraPos; }
	void setTerrain(std::shared_ptr<Terrain> t) { terrain = t; leftLeg.setTerrain(t); rightLeg.setTerrain(t); }
	void getHeelSwingCurvePoints(std::vector<float> &vec);
	void getPelvisVerticalCurvePoints(std::vector<float> &vec);
	void getPelvisSpeed1CurvePoints(std::vector<float> &vec);
	void getPelvisSpeed2CurvePoints(std::vector<float> &vec);
	void getPelvisSpeedCurvePoints(std::vector<float> &vec);
	void setToeOutAngle(float a) { leftLeg.setToeOutAngle(a); rightLeg.setToeOutAngle(a); }
	void setStepLength(float l) { stepLength = l; /*leftLeg.setStepLength(l); rightLeg.setStepLength(l);*/ }
	void setPelvisMidStanceOffset(float o) { leftLeg.setPelvisMidStanceOffset(o); rightLeg.setPelvisMidStanceOffset(o); }
	void setJointWeights(glm::vec3 &w) { jointWeights = w; }
	void setTimeSpeedCoeff(float c) { timeSpeedCoeff = c; }
	void setWalkingSpeed(float speed);
	void setRotationForward(float r) { bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, r - rotationForward, glm::vec3(1.0, 0.0, 0.0)); rotationForward = r; }
	void setMaxElbowExtension(float e) { leftArm.setMaxElbowExtension(e); rightArm.setMaxElbowExtension(e); }
	void setFootUpCoeff(float c) { leftLeg.setFootUpCoeff(c); rightLeg.setFootUpCoeff(c); }
};

#endif //SKELETON_H