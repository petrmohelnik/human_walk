#ifndef LEG_H
#define LEG_H

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

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

#define TERMINAL_STANCE_HEEL_RISE_ANGLE 0.6f
#define PRE_SWING_HEEL_RISE_ANGLE 0.6f

const float swingControlPoints[4]{0.0, 0.13, -0.19, -0.14};
const float swingRotControlPoints[4]{0.0, 0.052, 0.48, 0.28};

glm::vec3 bezierCurve(std::vector<glm::vec3> &P, float t);
float bezierCurve(std::vector<float> &P, float t);
float lerp(float start, float end, float t);

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

class Leg
{
private:
	Bone *root, *thigh, *shin, *foot, *toe;
	float thighLength = 0.0, shinLength = 0.0;
	float kneeRot = 0.0; //actual rotation in knee joint
	float t = 0.0; //time passed
	float width = 0.0; //distance of fot from the center
	float footRot = 0.0, toeRot = 0.0;
	float initialLoadingResponseRot = 0.0;
	float toeOffPos = -1.0;
	float stepLength = 0.0;
	float swingDist = 0.0;
	bool startNewStep = false;
	glm::vec3 prevAnklePos, prevHeelPos, prevToePos;
	glm::vec3 heelPos; //relative position of heel from ankle
	glm::vec3 toePos; //relative position of foot under toe form toe
	glm::mat4 thighBindMat; //original transformation of thigh
	std::vector<float> swingControlPointsVec;
	std::vector<float> swingRotControlPointsVec;

	glm::vec3 getToePos();
	glm::vec3 getHeelPos(glm::vec3 pos);
	glm::vec3 getAnklePosFromHeel(glm::vec3 pos);
	void solveIK(glm::vec3 desiredPos);
	void raiseHeel(glm::vec3 &baseVec, float startAngle, float endAngle, float t);
	void updateFootGlobal();
	void updateToeGlobal();
	float getAngle(const glm::vec3 &vec1, const glm::vec3 &vec2, const glm::mat4 &ref, const glm::vec3 &refDir);
public:
	Leg() = default;
	Leg(Bone *hips, Bone *sphericalJoint, Bone *hingeJoint, Bone *endEffector, Bone *fingers, float length);
	void init(int init, glm::vec3 center);
	void update(float dt);
	float getLegLength();
};

#endif //LEG_H