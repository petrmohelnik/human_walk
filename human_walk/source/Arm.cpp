#include "Arm.h"

Arm::Arm(Bone *neck, Bone *sphericalJoint, Bone *hingeJoint, Bone *endEffector, float length)
{
	root = neck;
	upperArm = sphericalJoint;
	forearm = hingeJoint;
	hand = endEffector;
	swingLength = length;

	width = upperArm->globalMat[3].x;
	glm::mat4 newUpperArm(1.0);
	newUpperArm = glm::rotate(newUpperArm, PI, glm::vec3(1.0, 0.0, 0.0));
	newUpperArm[3] = upperArm->localMat[3];
	upperArmBindMat = newUpperArm;
	rootStaticMat = root->globalMat;
	armLength = upperArm->scale + forearm->scale;
		
	glm::mat4 newForearm(1.0);
	newForearm = glm::translate(newForearm, glm::vec3(forearm->localMat[3]));
	forearm->localMat = newForearm;

	elbowCurve = BezierCurve(&elbowControlPoints[0][0], &elbowControlPoints[1][0], 12, 1, false);
	shoulderCurve = BezierCurve(&shoulderControlPoints[0][0], &shoulderControlPoints[1][0], 8, 1, false);

	/*zControlPointsVec.resize(4);
	for (auto i = 0; i < zControlPointsVec.size(); i++)
		zControlPointsVec[i] = zControlPointsVecPoints[i];

	yControlPointsVec.resize(4);
	for (auto i = 0; i < yControlPointsVec.size(); i++)
		yControlPointsVec[i] = yControlPointsVecPoints[i];

	xControlPointsVec.resize(4);
	for (auto i = 0; i < xControlPointsVec.size(); i++)
		xControlPointsVec[i] = xControlPointsVecPoints[i];

	smoothCurveControlPointsVec.resize(4);
	for (auto i = 0; i < smoothCurveControlPointsVec.size(); i++)
		smoothCurveControlPointsVec[i] = smoothCurveControlPointsVecPoints[i];*/

	solveIK(glm::vec3(upperArm->globalMat[3].x * 1.4,
		upperArm->globalMat[3].y - armLength,
		upperArm->globalMat[3].z));
}

void Arm::translateStaticRoot(glm::vec3 t)
{
	rootStaticMat[3].x += t.x;
	rootStaticMat[3].y += t.y;
	rootStaticMat[3].z += t.z;
}

void Arm::init(float time, float actRot, float actTilt)
{
	t = time;

	move(0.0, actRot, 0.0, actTilt);
}

void Arm::move(float prevRot, float actRot, float prevTilt, float actTilt)
{
	float armT = t;/* +TERMINAL_SWING - LOADING_RESPONSE;
	armT -= static_cast<int>(armT);
	armT = armT > 0.5 ? 1.0 - 2.0 * (armT - 0.5) : 2.0 * armT;
	armT = bezierCurve(smoothCurveControlPointsVec, armT);*/

	
	forearm->localMat = glm::rotate(forearm->localMat, -elbowRot, glm::vec3(1.0, 0.0, 0.0));
	upperArm->localMat = glm::rotate(upperArm->localMat, -shoulderRot, glm::vec3(1.0, 0.0, 0.0));
	upperArm->localMat = glm::rotate(upperArm->localMat, prevTilt, glm::vec3(0.0, 0.0, 1.0));
	upperArm->localMat = glm::rotate(upperArm->localMat, prevRot, glm::vec3(0.0, 1.0, 0.0));
	prevRotSave = prevRot;
	prevTiltSave = prevTilt;

	elbowRot = elbowCurve.YfromX(armT) + ELBOW_MINIMUM_ROT;// *(elbowCurve.getCoeff() > 1.0f ? 1.0f / elbowCurve.getCoeff() : 1.0f);
	shoulderRot = shoulderCurve.YfromX(armT);
	upperArm->localMat = glm::rotate(upperArm->localMat, -actRot, glm::vec3(0.0, 1.0, 0.0));
	upperArm->localMat = glm::rotate(upperArm->localMat, -actTilt, glm::vec3(0.0, 0.0, 1.0));
	forearm->localMat = glm::rotate(forearm->localMat, elbowRot, glm::vec3(1.0, 0.0, 0.0));
	upperArm->localMat = glm::rotate(upperArm->localMat, shoulderRot, glm::vec3(1.0, 0.0, 0.0));

	/*solveIK(glm::vec3(bezierCurve(xControlPointsVec, armT) * 1.6 * upperArm->globalMat[3].x,
		bezierCurve(yControlPointsVec, armT) - armLength + upperArm->globalMat[3].y,
		bezierCurve(zControlPointsVec, armT) + upperArm->globalMat[3].z));*/
}

void Arm::update(float dt, float prevRot, float actRot, float prevTilt, float actTilt)
{
	t += dt;

	if (t > TERMINAL_SWING)
		t -= TERMINAL_SWING;

	move(prevRot, actRot, prevTilt, actTilt);
}

void Arm::solveIK(glm::vec3 desiredPos)
{
	glm::mat4 referenceGlobal = rootStaticMat * upperArmBindMat;
	glm::vec3 referenceVec = glm::normalize(glm::vec3(glm::rotate(glm::rotate(referenceGlobal, (PI * 0.3f), glm::vec3(1.0, 0.0, 0.0)), width <= 0.0 ? 0.1f : -0.1f, glm::vec3(0.0, 0.0, 1.0)) * glm::vec4(0.0, 1.0, 0.0, 1.0) - rootStaticMat[3]));

	twoJointsIK(desiredPos, upperArm, forearm, elbowRot, referenceVec);
}
/*
void Arm::fixShoulderRotationAndTilt(float prevRot, float actRot, float prevTilt, float actTilt)
{
	upperArm->localMat = glm::rotate(upperArm->localMat, -prevTilt, glm::vec3(0.0, 0.0, 1.0));
	upperArm->localMat = glm::rotate(upperArm->localMat, -prevRot, glm::vec3(0.0, 1.0, 0.0));

	upperArm->localMat = glm::rotate(upperArm->localMat, actRot, glm::vec3(0.0, 1.0, 0.0));
	upperArm->localMat = glm::rotate(upperArm->localMat, actTilt, glm::vec3(0.0, 0.0, 1.0));
}*/

void Arm::incrementWidth(float a)
{
	forearm->localMat = glm::rotate(forearm->localMat, -elbowRot, glm::vec3(1.0, 0.0, 0.0));
	upperArm->localMat = glm::rotate(upperArm->localMat, -shoulderRot, glm::vec3(1.0, 0.0, 0.0));
	upperArm->localMat = glm::rotate(upperArm->localMat, prevTiltSave, glm::vec3(0.0, 0.0, 1.0));
	upperArm->localMat = glm::rotate(upperArm->localMat, prevRotSave, glm::vec3(0.0, 1.0, 0.0));

	upperArm->localMat = glm::rotate(upperArm->localMat, width > 0.0 ? a : -a , glm::vec3(0.0, 0.0, 1.0));

	upperArm->localMat = glm::rotate(upperArm->localMat, -prevTiltSave, glm::vec3(0.0, 1.0, 0.0));
	upperArm->localMat = glm::rotate(upperArm->localMat, -prevTiltSave, glm::vec3(0.0, 0.0, 1.0));
	upperArm->localMat = glm::rotate(upperArm->localMat, shoulderRot, glm::vec3(1.0, 0.0, 0.0));
	forearm->localMat = glm::rotate(forearm->localMat, elbowRot, glm::vec3(1.0, 0.0, 0.0));

}
