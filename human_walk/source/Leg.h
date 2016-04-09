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

const float swingControlPoints[][12]{{ 0.0f, 0.0690f, 0.1462f, 0.1976f, 0.1976f, 0.3684f, 0.5383f, 0.6888f, 0.6888f, 0.8353f, 0.8931f, 1.0 },
{ 0.2f, 0.2382f, 0.2531f, 0.2531f, 0.2531f, 0.2531f, 0.1465f, 0.065f, 0.065f, -0.0145f, 0.0521f, 0.0f }};
const float swingForwardControlPoints[][4]{{ 0.0f, 0.4731f, 0.6857f, 1.0f },
{ 0.0f, 0.1161f, 0.9679f, 1.0f }};
const float swingRotControlPoints[][4]{{0.0f, 0.2172f, 0.2357f, 1.0f},
{ -0.2618f, -0.28344f, 0.1611f, -0.044174f }};
const float heelRiseControlPoints[][4]{{ 0.0f, 0.5f, 0.8915f, 1.0f },
{ 0.0f, 0.0f, 0.8207f, 1.2f }};
const float swingWidthFixControlPoints[][8]{{ 0.0f, 0.15f, 0.25f, 0.5f, 0.5f, 0.75f, 0.85f, 1.0f },
{ 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f}};

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

struct AngleDifferences
{
	float hip;
	float knee;
	float ankle;
	AngleDifferences() = default;
	AngleDifferences(float h, float k, float a) { hip = h; knee = k; ankle = a; };
};

float lerp(float start, float end, float t);
float getAngle(const glm::vec3 &vec1, const glm::vec3 &vec2, const glm::mat4 &ref, const glm::vec3 &refDir);
glm::vec3 twoJointsIK(glm::vec3 desiredPos, glm::mat4 &sphericalJointGlobal, glm::mat4 &sphericalJointLocal, float sphericalJointScale,
	glm::mat4 &hingeJointLocal, float hingeJointScale, float &hingeRot, glm::vec3 referenceVec, glm::mat4 &rootGlobal);

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
	float pelvisMidStanceOffset = 0.0f;
	float toeOutAngle = 0.0f;
	float swingDist = 0.0;
	float terminalSwingInitToeRot = 0.0;
	float stepLengthSum = 0.0;
	float footUpCoeff = 0.0f; //parameter for walk - raising knees
	bool startNewStep = false;
	bool IKFalse = false;
	glm::vec3 prevAnklePos, prevHeelPos, toePosInit, nextHeelPos;
	glm::vec3 heelPos; //relative position of heel from ankle
	glm::vec3 toePos; //relative position of foot under toe form toe
	glm::vec3 pelvisMidStancePos; //position of pelvis from ankle in mid stance
	glm::mat4 thighBindMat; //original transformation of thigh
	//glm::mat4 rootStaticMat; //matrix of root without tilting/rotating of pelvis
	BezierCurve swingCurve;
	BezierCurve swingForwardCurve;
	BezierCurve swingRotationCurve;
	BezierCurve heelRiseCurve;
	BezierCurve swingWidthFixCurve;
	glm::vec3 prevPrevAnklePos;
	std::shared_ptr<const BezierCurve> pelvisVerticalCurve, pelvisSpeed1Curve, pelvisSpeed2Curve;
	std::shared_ptr<const glm::vec3> prevRootPos, nextRootPos;
	std::shared_ptr<const float> maxPelvisHeight;
	std::shared_ptr<const glm::mat4> staticRootMat;

	glm::vec3 getToePos(glm::mat4 &footGlobal);
	glm::vec3 getHeelPos(glm::vec3 pos, glm::mat4 &footGlobal);
	glm::vec3 getAnklePosFromHeel(glm::vec3 pos, glm::mat4 &footGlobal);
	void solveIK(glm::vec3 desiredPos);
	void riseHeel(float angle);
	void updateFootGlobal();
	void updateToeGlobal();
	float getFootAngleFromTerrain(glm::mat4 &footGlobal);
	void configureSwing();
	void configureSwingRotation();
	float pelvisSpeedCurve(std::shared_ptr<const BezierCurve> pSpeed1, std::shared_ptr<const BezierCurve> pSpeed2, float t);
public:
	Leg() = default;
	Leg(std::shared_ptr<Terrain> ter, Bone *hips, Bone *sphericalJoint, Bone *hingeJoint, Bone *endEffector, Bone *fingers, float length);
	void init(int init, glm::vec3 center, std::shared_ptr<const BezierCurve> pVert, std::shared_ptr<const BezierCurve> pSpeed1,
		std::shared_ptr<const BezierCurve> pSpeed2,	std::shared_ptr<const float> maxPelvisH, std::shared_ptr<const glm::vec3> _prevRootPos, 
		std::shared_ptr<const glm::vec3> _nextRootPos, std::shared_ptr<const glm::mat4> staticRootM);
	void update(float dt);
//	void translateStaticRoot(glm::vec3 t);
	float getLegLength();
	void increaseStepWidth(float i) { thighWidth > 0.0 ? footWidth += i : footWidth -= i; nextHeelPos.x = footWidth; }
	void setStepWidth(float w) { thighWidth > 0.0 ? footWidth = w : footWidth = -w; nextHeelPos.x = footWidth; }
	bool getIKFalse();
	glm::vec3 setNextPosition(float step);
	glm::vec3 getNextPosition() { return nextHeelPos; }
	glm::vec3 getPrevPosition() { return prevHeelPos; }
	float getFootWidth() { return footWidth; }
	void swingCurveIncrease(float inc) { swingCurve.swingIncrease(inc); }
	float getStepLengthSum() { return stepLengthSum; }
	void setTerrain(std::shared_ptr<Terrain> t) { terrain = t; }
	void getHeelSwingCurvePoints(std::vector<float> &vec);
	Bone *getThigh() { return thigh; }
	Bone *getShin() { return shin; }
	Bone *getFoot() { return foot; }
	float getKneeRot() { return kneeRot; }
	bool pelvisConfigurationInitMidStance(glm::mat4 &thighGlobal, glm::mat4 &thighLocal, glm::mat4 &shinLocal,
		float &testKneeRot, glm::mat4 &footLocal, glm::mat4 &rootLocal, glm::mat4 &rootLocalHeelStrike);
	void pelvisConfigurationInitMidSwing(glm::mat4 &thighGlobal, glm::mat4 &thighLocal, glm::mat4 &shinLocal,
		float &testKneeRot, glm::mat4 &footLocal, glm::mat4 &rootLocal);
	float solveIKStanceLegPelvisConfiguration(float time, glm::mat4 &thighGlobal, glm::mat4 &thighLocal, glm::mat4 &shinLocal,
		float &testKneeRot, glm::mat4 &footLocal, glm::mat4 &rootLocal, std::vector<AngleDifferences> &vec);
	glm::vec3 getPelvisMidStancePos() { return nextHeelPos + pelvisMidStancePos + glm::vec3(0.0, 0.0, pelvisMidStanceOffset); }
	void setToeOutAngle(float a) { toeOutAngle = a; }
	void setStepLength(float l) { stepLength = l; }
	void setPelvisMidStanceOffset(float o) { pelvisMidStanceOffset = o; }
	void setFootUpCoeff(float c) { footUpCoeff = c; }
};

#endif //LEG_H