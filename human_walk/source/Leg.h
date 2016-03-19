#ifndef LEG_H
#define LEG_H

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include "BezierCurve.h"

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

#define TERMINAL_STANCE_HEEL_RISE_ANGLE 0.4f
#define PRE_SWING_HEEL_RISE_ANGLE 0.6f

const float swingControlPoints[][8]{{ 0.0, 0.178, 0.3611, 0.7046, 0.7046, 0.7955, 0.9366, 1.0 },
{ 0.2, 0.2791, 0.2740, 0.0568, 0.0568, 0.0025, 0.0384, 0.0 }};
const float swingForwardControlPoints[][8]{{ 0.0, 0.20, 0.5196, 0.8018, 0.8018, 0.8536, 0.9, 1.0 },
{ 0.0, 0.05, 0.6393, 0.9214, 0.9214, 0.9714, 1.0, 1.0 }};
const float swingRotControlPoints[][8]{{0.0, 0.2172, 0.2357, 1.0},
{ -0.2618, -0.28344, 0.1611, -0.044174 }};

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

glm::vec3 bezierCurve(std::vector<glm::vec3> &P, float t);
float solveBezierForT(std::vector<float> &x, float X);
float bezierCurve(std::vector<float> &y, std::vector<float> &x, float X);
float bezierCurve(std::vector<float> &P, float t);
float lerp(float start, float end, float t);
float getAngle(const glm::vec3 &vec1, const glm::vec3 &vec2, const glm::mat4 &ref, const glm::vec3 &refDir);
glm::vec3 twoJointsIK(glm::vec3 desiredPos, Bone *sphericalJoint, Bone *hingeJoint, float &hingeRot, glm::vec3 referenceVec);

class Leg
{
private:
	Bone *root, *thigh, *shin, *foot, *toe;
	float thighLength = 0.0, shinLength = 0.0;
	float thighWidth, footWidth; //distance of foot from the center
	float kneeRot = 0.0; //actual rotation in knee joint
	float t = 0.0; //time passed
	float footRot = 0.0, toeRot = 0.0;
	float initialLoadingResponseRot = 0.0;
	float toeOffPos = -1.0;
	float stepLength = 0.0;
	float swingDist = 0.0;
	float terminalSwingInitToeRot = 0.0;
	bool startNewStep = false;
	bool IKFalse = false;
	glm::vec3 prevAnklePos, prevHeelPos, prevToePos, toePosInit, nextHeelPos;
	glm::vec3 heelPos; //relative position of heel from ankle
	glm::vec3 toePos; //relative position of foot under toe form toe
	glm::mat4 thighBindMat; //original transformation of thigh
	glm::mat4 rootStaticMat; //matrix of root without tilting/rotating of pelvis
	//std::vector<float> swingControlPointsVec;
	BezierCurve swingCurve;
	BezierCurve swingForwardCurve;
	//std::vector<float> swingRotControlPointsVec;
	BezierCurve swingRotationCurve;

	glm::vec3 getToePos();
	glm::vec3 getHeelPos(glm::vec3 pos);
	glm::vec3 getAnklePosFromHeel(glm::vec3 pos);
	void solveIK(glm::vec3 desiredPos);
	void riseHeel(glm::vec3 &baseVec, float startAngle, float endAngle, float t);
	void updateFootGlobal();
	void updateToeGlobal();
public:
	Leg() = default;
	Leg(Bone *hips, Bone *sphericalJoint, Bone *hingeJoint, Bone *endEffector, Bone *fingers, float length);
	void init(int init, glm::vec3 center);
	void update(float dt);
	void translateStaticRoot(glm::vec3 t);
	float getLegLength();
	void increaseStepWidth(float i) { thighWidth > 0.0 ? footWidth += i : footWidth -= i; }
	bool getIKFalse();
};

#endif //LEG_H