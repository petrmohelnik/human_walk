#ifndef LEG_H
#define LEG_H

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include "BezierCurve.h"
#include "Terrain.h"

#define PI 3.14159265f

#define LOADING_RESPONSE 0.12f
#define MID_STANCE 0.31f
#define TERMINAL_STANCE 0.5f
#define PRE_SWING 0.62f
#define INITIAL_SWING 0.75f
#define MID_SWING 0.87f
#define TERMINAL_SWING 1.0f

#define INIT_HEEL_STRIKE 0
#define INIT_TERMINAL_STANCE 1

#define TERMINAL_STANCE_HEEL_RISE_ANGLE 0.25f
#define PRE_SWING_HEEL_RISE_ANGLE 0.6f

const float swingControlPoints[][12]{{ 0.0, 0.0690, 0.1340, 0.1976, 0.1976, 0.4371, 0.6320, 0.8174, 0.8174, 0.8810, 0.927, 1.0 },
{ 0.2, 0.2382, 0.2531, 0.2531, 0.2531, 0.2531, 0.0365, 0.0189, 0.0189, 0.0135, 0.0271, 0.0 }};
const float swingForwardControlPoints[][4]{{ 0.0, 0.4731, 0.6857, 1.0 },
{ 0.0, 0.1161, 0.9679, 1.0 }};
const float swingRotControlPoints[][4]{{0.0, 0.2172, 0.2357, 1.0},
{ -0.2618, -0.28344, 0.1611, -0.044174 }};
const float heelRiseControlPoints[][4]{{ 0.0, 0.3, 0.83, 1.0 },
{ 0.0, 0.0, 0.45, 0.85 }};
const float swingWidthFixControlPoints[][8]{{ 0.0, 0.15, 0.25, 0.5, 0.5, 0.75, 0.85, 1.0 },
{ 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0}};

struct Bone
{
	glm::mat4 localMat; //lokalni transformace
	glm::mat4 inverseBindMatrix; //inverzni bind pose matice
	glm::mat4 globalMat; //globalni transformace
	Bone* parent; //odkaz na rodicovskou kost
	std::vector<Bone *> childs; //odkaz na potomky
	float scale; //velikost kosti, slouzi pro zobrazeni kosti
	Bone(glm::mat4 mat, Bone *parent) : localMat(mat), parent(parent), scale(1.0f) {}
};

float lerp(float start, float end, float t);
float getAngle(const glm::vec3 &vec1, const glm::vec3 &vec2, const glm::mat4 &ref, const glm::vec3 &refDir);
glm::vec3 twoJointsIK(glm::vec3 desiredPos, Bone *sphericalJoint, Bone *hingeJoint, float &hingeRot, glm::vec3 referenceVec);

class Leg
{
private:
	std::shared_ptr<Terrain> terrain;
	Bone *root, *thigh, *shin, *foot, *toe;
	float thighLength = 0.0, shinLength = 0.0;
	float thighWidth, footWidth; //distance of foot from the center
	float kneeRot = 0.0; //actual rotation in knee joint
	float t = 0.0; //time passed
	float footRot = 0.0, toeRot = 0.0;
	float initialLoadingResponseRot = 0.0;
	glm::vec3 toeOffPos;
	float stepLength = 0.0;
	float swingDist = 0.0;
	float terminalSwingInitToeRot = 0.0;
	float stepLengthSum = 0.0;
	bool startNewStep = false;
	bool IKFalse = false;
	glm::vec3 prevAnklePos, prevHeelPos, prevToePos, toePosInit, nextHeelPos;
	glm::vec3 heelPos; //relative position of heel from ankle
	glm::vec3 toePos; //relative position of foot under toe form toe
	glm::mat4 thighBindMat; //original transformation of thigh
	glm::mat4 rootStaticMat; //matrix of root without tilting/rotating of pelvis
	BezierCurve swingCurve;
	BezierCurve swingForwardCurve;
	BezierCurve swingRotationCurve;
	BezierCurve heelRiseCurve;
	BezierCurve swingWidthFixCurve;
	glm::vec3 prevPrevAnklePos;

	glm::vec3 getToePos();
	glm::vec3 getHeelPos(glm::vec3 pos);
	glm::vec3 getAnklePosFromHeel(glm::vec3 pos);
	void solveIK(glm::vec3 desiredPos);
	void riseHeel(glm::vec3 &baseVec, float startAngle, float endAngle, float t);
	void updateFootGlobal();
	void updateToeGlobal();
public:
	Leg() = default;
	Leg(std::shared_ptr<Terrain> ter, Bone *hips, Bone *sphericalJoint, Bone *hingeJoint, Bone *endEffector, Bone *fingers, float length);
	void init(int init, glm::vec3 center);
	void update(float dt);
	void translateStaticRoot(glm::vec3 t);
	float getLegLength();
	void increaseStepWidth(float i) { thighWidth > 0.0 ? footWidth += i : footWidth -= i; nextHeelPos.x = footWidth; }
	bool getIKFalse();
	glm::vec3 setNextPosition(float step);
	glm::vec3 getNextPosition() { return nextHeelPos; }
	glm::vec3 getPrevPosition() { return prevHeelPos; }
	float getFootWidth() { return footWidth; }
	void swingCurveIncrease(float inc) { swingCurve.swingIncrease(inc); }
	float getStepLengthSum() { return stepLengthSum; }
};

#endif //LEG_H