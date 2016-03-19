#include "Skeleton.h"

void Skeleton::init()
{
	countGlobalMatrices();

	rightLeg = Leg(&bones[HIPS], &bones[THIGH_R], &bones[SHIN_R], &bones[FOOT_R], &bones[TOE_R], stepLength);
	leftLeg = Leg(&bones[HIPS], &bones[THIGH_L], &bones[SHIN_L], &bones[FOOT_L], &bones[TOE_L], stepLength);
	rightArm = Arm(&bones[NECK], &bones[UPPER_ARM_R], &bones[FOREARM_R], &bones[HAND_R], 0.5);
	leftArm = Arm(&bones[NECK], &bones[UPPER_ARM_L], &bones[FOREARM_L], &bones[HAND_L], 0.5);

	staticRootPos = glm::vec3(bones[HIPS].globalMat[3]);

	/*pelvisVerticalControlPointsVec.resize(4);
	for (auto i = 0; i < pelvisVerticalControlPointsVec.size(); i++)*/
	//	pelvisVerticalControlPointsVec[i] = pelvisVerticalControlPoints[i] - 0.01 /*(bones[FOOT_R].globalMat[3].y + rightLeg.getLegLength() - bones[THIGH_R].globalMat[3].y)*/ + bones[HIPS].globalMat[3].y; // leftLeg.getLegLength();
	maxPelvisHeight += bones[HIPS].globalMat[3].y;
	pelvisVerticalCurve = BezierCurve(&pelvisVerticalControlPoints[0][0], &pelvisVerticalControlPoints[1][0], 8, 2, false);

	/*pelvisLateralControlPointsVec.resize(4);
	for (auto i = 0; i < pelvisLateralControlPointsVec.size(); i++)
		pelvisLateralControlPointsVec[i] = pelvisLateralControlPoints[i] + bones[HIPS].globalMat[3].x; // leftLeg.getLegLength();*/
	pelvisLateralCurve = BezierCurve(&pelvisLateralControlPoints[0][0], &pelvisLateralControlPoints[1][0], 8, 2, true);

	/*pelvicRotationControlPointsVec.resize(4);
	for (auto i = 0; i < pelvicRotationControlPointsVec.size(); i++)
		pelvicRotationControlPointsVec[i] = pelvicRotationControlPoints[i];*/
	pelvicRotationCurve = BezierCurve(&pelvicRotationControlPoints[0][0], &pelvicRotationControlPoints[1][0], 8, 2, true);
	shoulderRotationCurve = BezierCurve(&shoulderRotationControlPoints[0][0], &shoulderRotationControlPoints[1][0], 8, 2, true);

	/*pelvicTiltControlPointsVec.resize(4);
	for (auto i = 0; i < pelvicTiltControlPointsVec.size(); i++)
		pelvicTiltControlPointsVec[i] = pelvicTiltControlPoints[i];*/

	pelvicTiltCurve = BezierCurve(&pelvicTiltControlPoints[0][0], &pelvicTiltControlPoints[1][0], 16, 2, true);

	/*pelvicTiltForwardControlPointsVec.resize(4);
	for (auto i = 0; i < pelvicTiltForwardControlPointsVec.size(); i++)
		pelvicTiltForwardControlPointsVec[i] = pelvicTiltForwardControlPoints[i];*/

	pelvicTiltForwardCurve = BezierCurve(&pelvicTiltForwardControlPoints[0][0], &pelvicTiltForwardControlPoints[1][0], 8, 2, false);

	/*pelvisSpeedControlPointsVec.resize(4);
	for (auto i = 0; i < pelvisSpeedControlPointsVec.size(); i++)
		pelvisSpeedControlPointsVec[i] = pelvisSpeedControlPoints[i];*/
	pelvisSpeedCurve = BezierCurve(&pelvisSpeedControlPoints[0][0], &pelvisSpeedControlPoints[1][0], 8, 2, false);
	pelvisSpeedIntegralCurve = BezierCurve(&pelvisSpeedIntegralControlPoints[0][0], &pelvisSpeedIntegralControlPoints[1][0], 4, 1, false);

	uniformXVec.resize(4);
	for (auto i = 0; i < uniformXVec.size(); i++)
		uniformXVec[i] = uniformX[i];

	glm::vec3 hipsCenter = glm::vec3(bones[THIGH_R].globalMat[3] + bones[THIGH_L].globalMat[3]);

	bones[HIPS].localMat[3].x = pelvisLateralCurve.YfromX(1.0 - LOADING_RESPONSE * 0.5);//bezierCurve(pelvisLateralControlPointsVec, 2.0 * MID_STANCE);
	bones[HIPS].localMat[3].y = pelvisVerticalCurve.YfromX(1.0 - LOADING_RESPONSE * 0.5) + maxPelvisHeight;//bezierCurve(pelvisVerticalControlPointsVec, uniformXVec, 2.0 * LOADING_RESPONSE);
	pelvicRotation = pelvicRotationCurve.YfromX(0.0);//bezierCurve(pelvicRotationControlPointsVec, 0.0);
	shoulderRotation = shoulderRotationCurve.YfromX(0.0);
	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, pelvicRotation, glm::vec3(0.0, 1.0, 0.0));
	pelvicTilt = pelvicTiltCurve.YfromX(0.96); // bezierCurve(pelvicTiltControlPointsVec, 0.0);
	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, pelvicTilt, glm::vec3(0.0, 0.0, 1.0));
	pelvicTiltForward = pelvicTiltForwardCurve.YfromX(1.0 - LOADING_RESPONSE);//bezierCurve(pelvicTiltForwardControlPointsVec, 4.0 * LOADING_RESPONSE);
	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, pelvicTiltForward, glm::vec3(1.0, 0.0, 0.0));
	countGlobalMatrices();

	bones[SHOULDER_L].localMat = glm::rotate(bones[SHOULDER_L].localMat, (2.0f * shoulderRotation / 3.0f), glm::vec3(1.0, 0.0, 0.0));
	//bones[SHOULDER_L].localMat = glm::rotate(bones[SHOULDER_L].localMat, -0.5f*pelvicTilt, glm::vec3(0.0, 0.0, 1.0));
	bones[SHOULDER_R].localMat = glm::rotate(bones[SHOULDER_R].localMat, -(2.0f * shoulderRotation / 3.0f), glm::vec3(1.0, 0.0, 0.0));
	//bones[SHOULDER_R].localMat = glm::rotate(bones[SHOULDER_R].localMat, -0.5f*pelvicTilt, glm::vec3(0.0, 0.0, 1.0));
	spineRot = solveSpine(0.0, pelvicTilt);
	bones[SPINE].localMat = glm::rotate(bones[SPINE].localMat, spineRot, glm::vec3(0.0, 0.0, 1.0));
	bones[SPINE].localMat = glm::rotate(bones[SPINE].localMat, -(2.0f * shoulderRotation / 3.0f), glm::vec3(0.0, 1.0, 0.0));
	bones[CHEST].localMat = glm::rotate(bones[CHEST].localMat, spineRot, glm::vec3(0.0, 0.0, 1.0));
	bones[CHEST].localMat = glm::rotate(bones[CHEST].localMat, -(2.0f * shoulderRotation / 3.0f), glm::vec3(0.0, 1.0, 0.0));
	float shoulderRotFix = ((4.0f / 3.0f) * shoulderRotation - pelvicRotation);
	float shoulderTiltFix = pelvicTilt + 2.0f * spineRot;
	bones[NECK].localMat = glm::rotate(bones[NECK].localMat, -(shoulderTiltFix)* 0.5f, glm::vec3(0.0, 0.0, 1.0));
	bones[NECK].localMat = glm::rotate(bones[NECK].localMat, (shoulderRotFix / 2.0f), glm::vec3(0.0, 1.0, 0.0));
	bones[HEAD].localMat = glm::rotate(bones[HEAD].localMat, -(shoulderTiltFix)* 0.5f, glm::vec3(0.0, 0.0, 1.0));
	bones[HEAD].localMat = glm::rotate(bones[HEAD].localMat, (shoulderRotFix / 2.0f), glm::vec3(0.0, 1.0, 0.0));
	//rightArm.fixShoulderRotationAndTilt(0.0, shoulderRotFix, 0.0, -shoulderTiltFix);
	//leftArm.fixShoulderRotationAndTilt(0.0, shoulderRotFix, 0.0, -shoulderTiltFix);
	//headRot = pelvicTiltForward;
	//bones[HEAD].localMat = glm::rotate(bones[HEAD].localMat, headRot, glm::vec3(1.0, 0.0, 0.0));

	rightLeg.init(INIT_HEEL_STRIKE, hipsCenter);
	leftLeg.init(INIT_TERMINAL_STANCE, hipsCenter);
	rightArm.init(0.0, shoulderRotFix, shoulderTiltFix);
	leftArm.init(0.5, shoulderRotFix, shoulderTiltFix);
}

float Skeleton::spineEquation(float T, float hipAngle, float x)
{
	return T - sin(x + hipAngle) * bones[SPINE].scale - sin(2 * x + hipAngle) * bones[CHEST].scale;
}

float Skeleton::solveSpine(float dist, float hipAngle)
{
	float fa, fb, a = -1.0, b = 1.0;
	float fc, prevC, c;
	float T = dist - sin(hipAngle) * bones[HIPS].scale;

	if ((fa = spineEquation(T, hipAngle, a)) > (fb = spineEquation(T, hipAngle, b))) {
		float temp = a;	a = b; b = temp;
		temp = fa; fa = fb;	fb = temp;
	}

	c = b - ((b - a) / (fb - fa)) * fb;
	if ((fc = spineEquation(T, hipAngle, c)) > 0.0){
		b = c;
		fb = fc;
	}
	else {
		a = c;
		fa = fc;
	}
	prevC = c;

	do {
		prevC = c;
		c = b - ((b - a) / (fb - fa)) * fb;
		if ((fc = spineEquation(T, hipAngle, c)) > 0.0){
			b = c;
			fb = fc;
		}
		else {
			a = c;
			fa = fc;
		}
	} while (abs(prevC - c) > 0.00001);

	return c;
}

void Skeleton::onUpdate(float dt)
{
	float speedDispFix = 0.0;
	dt = dt > 0.1 ? 0.1 : dt;
	//dt *= 0.1f;
	t += dt;

	if (t > TERMINAL_SWING) {
		t -= TERMINAL_SWING;
		stepSpeedAccuracyCheck += (dt - t) * stepLength * (pelvisSpeedCurve.YfromX(1.0) + 1.0);
		speedDispFix = stepLength - stepSpeedAccuracyCheck;
		stepSpeedAccuracyCheck = 0.0;
		std::cout << "speedDispFix: " << speedDispFix << std::endl;
		std::cout << "bones[HIPS].localMat[3].z: " << bones[HIPS].localMat[3].z << std::endl;

		if (leftLeg.getIKFalse() || rightLeg.getIKFalse())
			increaseMaxPelvisHeight(-0.01);
	}

	float pelvicVerticalT = t - LOADING_RESPONSE * 0.5 + 1.0;
	pelvicVerticalT -= static_cast<int>(pelvicVerticalT);
	//pelvicVerticalT = pelvicVerticalT > 0.25 ? 1.0 - 4.0 * (pelvicVerticalT - 0.25) : 4.0 * pelvicVerticalT;

	float pelvicLateralT = t - LOADING_RESPONSE * 0.5 + 1.0;
	pelvicLateralT -= static_cast<int>(pelvicLateralT);
	//pelvicLateralT = pelvicLateralT > 0.5 ? 1.0 - 2.0 * (pelvicLateralT - 0.5) : 2.0 * pelvicLateralT;

	float pelvicRotationT = t;// +1.0;
	//pelvicRotationT -= static_cast<int>(pelvicRotationT);
	//pelvicRotationT = pelvicRotationT > 0.5 ? 1.0 - 2.0 * (pelvicRotationT - 0.5) : 2.0 * pelvicRotationT;

	/*float pelvicTiltT = t;// +1.0;
	//pelvicTiltT -= static_cast<int>(pelvicTiltT);
	pelvicTiltT = pelvicTiltT > 0.5 ? 1.0 - 2.0 * (pelvicTiltT - 0.5) :
		(pelvicTiltT < LOADING_RESPONSE ? pelvicTiltT * (1.0 / LOADING_RESPONSE) : (pelvicTiltT - 0.5) * (1.0 / (0.5 - LOADING_RESPONSE)));*/

	float pelvicTiltForwardT = t - LOADING_RESPONSE + 1.0;
	/*pelvicTiltForwardT -= static_cast<int>(pelvicTiltForwardT / 0.5) * 0.5;
	pelvicTiltForwardT = pelvicTiltForwardT > 0.25 ? 1.0 - 4.0 * (pelvicTiltForwardT - 0.25) : 4.0 * pelvicTiltForwardT;*/
	pelvicTiltForwardT -= static_cast<int>(pelvicTiltForwardT);

	bones[HIPS].localMat[3].x = pelvisLateralCurve.YfromX(pelvicLateralT); // bezierCurve(pelvisLateralControlPointsVec, pelvicLateralT);
	bones[HIPS].localMat[3].y = pelvisVerticalCurve.YfromX(pelvicVerticalT) + maxPelvisHeight;//bezierCurve(pelvisVerticalControlPointsVec, uniformXVec, pelvicVerticalT);
	float forwardDisp = dt * stepLength;
	//float pelvicVerticalTOld = t - LOADING_RESPONSE * 0.5 + 1.0;
	//pelvicVerticalTOld -= static_cast<int>(pelvicVerticalTOld / 0.5) * 0.5;;
	//pelvicVerticalTOld = pelvicVerticalTOld > 0.25 ? 1.0 - 4.0 * (pelvicVerticalTOld - 0.25) : 4.0 * pelvicVerticalTOld;
	//pelvisSpeedCurve.setCoeffImmediately(0.8);
	pelvisSpeedCurve.setCoeffImmediately(0.7);
	float forwardDispSpeed = forwardDisp * (pelvisSpeedCurve.YfromX(pelvicVerticalT) + 1.0);
	stepSpeedAccuracyCheck += forwardDispSpeed;
	bones[HIPS].localMat[3].z += forwardDispSpeed;// *bezierCurve(pelvisSpeedControlPointsVec, pelvicVerticalTOld);
	leftLeg.translateStaticRoot(glm::vec3(0.0, 0.0, forwardDisp));
	rightLeg.translateStaticRoot(glm::vec3(0.0, 0.0, forwardDisp));
	leftArm.translateStaticRoot(glm::vec3(0.0, 0.0, forwardDisp));
	rightArm.translateStaticRoot(glm::vec3(0.0, 0.0, forwardDisp));
	staticRootPos += glm::vec3(0.0, 0.0, forwardDisp);

	/*double integral = 0.0;
	for (int i = 0; i <= 10000; i++)
	{
		integral += pelvisSpeedCurve.YfromX(0.0001 * i);
	}
	std::cout << "integral: " << integral << std::endl;*/

	/*for (int i = 0; i <= 1000000; i++) {
		float x = pelvisSpeedIntegralCurve.solveX(0.000001 * i);
		float y = pelvisSpeedIntegralCurve.solveY(0.000001 * i);
		if (abs(x - y) < 0.000001)
			std::cout << "y: " << y << std::endl;
	}*/

	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, -pelvicTiltForward, glm::vec3(1.0, 0.0, 0.0));
	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, -pelvicTilt, glm::vec3(0.0, 0.0, 1.0));
	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, -pelvicRotation, glm::vec3(0.0, 1.0, 0.0));
	//bones[SHOULDER_L].localMat = glm::rotate(bones[SHOULDER_L].localMat, 0.5f*pelvicTilt, glm::vec3(0.0, 0.0, 1.0));
	bones[SHOULDER_L].localMat = glm::rotate(bones[SHOULDER_L].localMat, -(2.0f * shoulderRotation / 3.0f), glm::vec3(1.0, 0.0, 0.0));
	//bones[SHOULDER_R].localMat = glm::rotate(bones[SHOULDER_R].localMat, 0.5f*pelvicTilt, glm::vec3(0.0, 0.0, 1.0));
	bones[SHOULDER_R].localMat = glm::rotate(bones[SHOULDER_R].localMat, (2.0f * shoulderRotation / 3.0f), glm::vec3(1.0, 0.0, 0.0));
	/*bones[SPINE].localMat = glm::rotate(bones[SPINE].localMat, 0.8f*pelvicTilt, glm::vec3(0.0, 0.0, 1.0));
	bones[CHEST].localMat = glm::rotate(bones[CHEST].localMat, 0.8f*pelvicTilt, glm::vec3(0.0, 0.0, 1.0));*/
	bones[SPINE].localMat = glm::rotate(bones[SPINE].localMat, (2.0f * shoulderRotation / 3.0f), glm::vec3(0.0, 1.0, 0.0));
	bones[SPINE].localMat = glm::rotate(bones[SPINE].localMat, -spineRot, glm::vec3(0.0, 0.0, 1.0));
	bones[CHEST].localMat = glm::rotate(bones[CHEST].localMat, (2.0f * shoulderRotation / 3.0f), glm::vec3(0.0, 1.0, 0.0));
	bones[CHEST].localMat = glm::rotate(bones[CHEST].localMat, -spineRot, glm::vec3(0.0, 0.0, 1.0));
	float shoulderRotFix = ((4.0f / 3.0f) * shoulderRotation - pelvicRotation);
	float shoulderTiltFix = pelvicTilt + 2.0f * spineRot;
	bones[NECK].localMat = glm::rotate(bones[NECK].localMat, -(shoulderRotFix / 2.0f), glm::vec3(0.0, 1.0, 0.0));
	bones[NECK].localMat = glm::rotate(bones[NECK].localMat, shoulderTiltFix * 0.5f, glm::vec3(0.0, 0.0, 1.0));
	//bones[HEAD].localMat = glm::rotate(bones[HEAD].localMat, -headRot, glm::vec3(1.0, 0.0, 0.0));
	bones[HEAD].localMat = glm::rotate(bones[HEAD].localMat, -(shoulderRotFix / 2.0f), glm::vec3(0.0, 1.0, 0.0));
	bones[HEAD].localMat = glm::rotate(bones[HEAD].localMat, shoulderTiltFix * 0.5f, glm::vec3(0.0, 0.0, 1.0));

	pelvicRotation = pelvicRotationCurve.YfromX(pelvicRotationT);//bezierCurve(pelvicRotationControlPointsVec, pelvicRotationT);
	shoulderRotation = shoulderRotationCurve.YfromX(pelvicRotationT);
	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, pelvicRotation, glm::vec3(0.0, 1.0, 0.0));
	//float pelvicTiltT = t;// +1.0;
	//pelvicTiltT -= static_cast<int>(pelvicTiltT);
	/*if (pelvicTiltT > 0.5) {
		pelvicTiltT -= 0.5;
		pelvicTiltT = pelvicTiltT < LOADING_RESPONSE ? pelvicTiltT * (1.0 / LOADING_RESPONSE) : (0.5 - pelvicTiltT) * (1.0 / (0.5 - LOADING_RESPONSE));
		pelvicTilt = -bezierCurve(pelvicTiltControlPointsVec, pelvicTiltT);
	}
	else {
		pelvicTiltT = pelvicTiltT < LOADING_RESPONSE ? pelvicTiltT * (1.0 / LOADING_RESPONSE) : (0.5 - pelvicTiltT) * (1.0 / (0.5 - LOADING_RESPONSE));
		pelvicTilt = bezierCurve(pelvicTiltControlPointsVec, pelvicTiltT);
	}*/
	float pelvicTiltT = t - 0.04 + 1.0;
	pelvicTiltT -= static_cast<int>(pelvicTiltT);
	pelvicTilt = pelvicTiltCurve.YfromX(pelvicTiltT);
	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, pelvicTilt, glm::vec3(0.0, 0.0, 1.0));
	pelvicTiltForward = pelvicTiltForwardCurve.YfromX(pelvicTiltForwardT);//bezierCurve(pelvicTiltForwardControlPointsVec, pelvicTiltForwardT);
	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, pelvicTiltForward, glm::vec3(1.0, 0.0, 0.0));

	bones[SHOULDER_L].localMat = glm::rotate(bones[SHOULDER_L].localMat, (2.0f * shoulderRotation / 3.0f), glm::vec3(1.0, 0.0, 0.0));
	//bones[SHOULDER_L].localMat = glm::rotate(bones[SHOULDER_L].localMat, -0.5f*pelvicTilt, glm::vec3(0.0, 0.0, 1.0));
	bones[SHOULDER_R].localMat = glm::rotate(bones[SHOULDER_R].localMat, -(2.0f * shoulderRotation / 3.0f), glm::vec3(1.0, 0.0, 0.0));
	//bones[SHOULDER_R].localMat = glm::rotate(bones[SHOULDER_R].localMat, -0.5f*pelvicTilt, glm::vec3(0.0, 0.0, 1.0));
	spineRot = solveSpine(0.0, pelvicTilt);
	/*bones[SPINE].localMat = glm::rotate(bones[SPINE].localMat, -0.8f*pelvicTilt, glm::vec3(0.0, 0.0, 1.0));
	bones[CHEST].localMat = glm::rotate(bones[CHEST].localMat, -0.8f*pelvicTilt, glm::vec3(0.0, 0.0, 1.0));*/
	bones[SPINE].localMat = glm::rotate(bones[SPINE].localMat, spineRot, glm::vec3(0.0, 0.0, 1.0));
	bones[SPINE].localMat = glm::rotate(bones[SPINE].localMat, -(2.0f * shoulderRotation / 3.0f), glm::vec3(0.0, 1.0, 0.0));
	bones[CHEST].localMat = glm::rotate(bones[CHEST].localMat, spineRot, glm::vec3(0.0, 0.0, 1.0));
	bones[CHEST].localMat = glm::rotate(bones[CHEST].localMat, -(2.0f * shoulderRotation / 3.0f), glm::vec3(0.0, 1.0, 0.0));
	float shoulderRotFixUpdated = ((4.0f / 3.0f) * shoulderRotation - pelvicRotation);
	float shoulderTiltFixUpdated = pelvicTilt + 2.0f * spineRot;
	bones[NECK].localMat = glm::rotate(bones[NECK].localMat, -shoulderTiltFixUpdated * 0.5f, glm::vec3(0.0, 0.0, 1.0));
	bones[NECK].localMat = glm::rotate(bones[NECK].localMat, (shoulderRotFixUpdated / 2.0f), glm::vec3(0.0, 1.0, 0.0));
	bones[HEAD].localMat = glm::rotate(bones[HEAD].localMat, -shoulderTiltFixUpdated * 0.5f, glm::vec3(0.0, 0.0, 1.0));
	bones[HEAD].localMat = glm::rotate(bones[HEAD].localMat, (shoulderRotFixUpdated / 2.0f), glm::vec3(0.0, 1.0, 0.0));
	//rightArm.fixShoulderRotationAndTilt(shoulderRotFix, shoulderRotFixUpdated, -shoulderTiltFix, -shoulderTiltFixUpdated);
	//leftArm.fixShoulderRotationAndTilt(shoulderRotFix, shoulderRotFixUpdated, -shoulderTiltFix, -shoulderTiltFixUpdated);
	//headRot = pelvicTiltForward;
	//bones[HEAD].localMat = glm::rotate(bones[HEAD].localMat, headRot, glm::vec3(1.0, 0.0, 0.0));
	countGlobalMatrices();
	leftLeg.update(dt);
	rightLeg.update(dt);
	leftArm.update(dt, shoulderRotFix, shoulderRotFixUpdated, shoulderTiltFix, shoulderTiltFixUpdated);
	rightArm.update(dt, shoulderRotFix, shoulderRotFixUpdated, shoulderTiltFix, shoulderTiltFixUpdated);
	
	countGlobalMatrices();
}

int Skeleton::addBone(glm::mat4 m, int p)
{
	bones.push_back(Bone(m, p >= 0 ? &bones[p] : nullptr));
	if (p >= 0)
		bones[p].childs.push_back(&bones.back());

	return bones.size() - 1;
}

void Skeleton::getScaledGlobalMatrices(std::vector<glm::mat4> &vec)
{
	vec.resize(0);

	for (unsigned int i = 0; i < bones.size(); i++)	{
		vec.push_back(glm::scale(bones[i].globalMat, glm::vec3(bones[i].scale)));
	}
}

std::vector<Bone> Skeleton::getBones()
{
	return bones;
}

void Skeleton::countGlobalMatrices()
{
	for (unsigned int i = 0; i < bones.size(); i++) {
		if (bones[i].parent == nullptr)
			bones[i].globalMat = bones[i].localMat;
		else
			bones[i].globalMat = bones[i].parent->globalMat * bones[i].localMat;
	}
}

void Skeleton::fixScale()
{
	countGlobalMatrices();

	for (unsigned int i = 0; i < bones.size(); i++) {
		if (bones[i].childs.size() != 0)
			bones[i].scale = glm::length(glm::vec3(bones[i].childs[0]->globalMat[3] - bones[i].globalMat[3]));
		else
			bones[i].scale = bones[i].parent->scale * 0.5f;
	}

	/*for (unsigned int i = 0; i < bones.size(); i++) {
		if (bones[i].name.find("empty") != std::string::npos) {
			bones[bones[i].parent].childs.pop_back();
			for (unsigned int j = 0; j < bones.size(); j++) {
				if (bones[j].parent > (int)i)
					bones[j].parent--;
				for (unsigned int k = 0; k < bones[j].childs.size(); k++) {
					if (bones[j].childs[k] > (int)i)
						bones[j].childs[k]--;
				}
			}
			bones.erase(bones.begin() + i);
			i--;
		}
	}*/

	/*bones[0].localMat = glm::rotate(bones[0].localMat, -0.7f, glm::vec3(1.0, 0.0, 0.0));
	bones[1].localMat = glm::rotate(bones[1].localMat, 0.7f, glm::vec3(1.0, 0.0, 0.0));
	bones[2].localMat = glm::rotate(bones[2].localMat, -0.7f, glm::vec3(1.0, 0.0, 0.0));
	bones[2].localMat = glm::rotate(bones[2].localMat, -0.3f, glm::vec3(0.0, 0.0, 1.0));
	bones[4].localMat = glm::rotate(bones[4].localMat, 0.4f, glm::vec3(0.0, 1.0, 0.0));
	bones[13].localMat = glm::rotate(bones[13].localMat, -0.6f, glm::vec3(1.0, 0.0, 0.0));
	bones[getBoneByName("Bone_007")].localMat = glm::rotate(bones[getBoneByName("Bone_007")].localMat, 0.5f, glm::vec3(1.0, 0.0, 0.0));
	bones[getBoneByName("Bone_009")].localMat = glm::rotate(bones[getBoneByName("Bone_009")].localMat, 1.5f, glm::vec3(0.0, 0.0, 1.0));
	bones[getBoneByName("Bone_019")].localMat = glm::rotate(bones[getBoneByName("Bone_019")].localMat, -0.7f, glm::vec3(1.0, 0.0, 0.0));
	bones[getBoneByName("Bone_023")].localMat = glm::rotate(bones[getBoneByName("Bone_023")].localMat, -0.3f, glm::vec3(1.0, 0.0, 0.0));
	bones[getBoneByName("Bone_023")].localMat = glm::rotate(bones[getBoneByName("Bone_023")].localMat, -0.5f, glm::vec3(0.0, 0.0, 1.0));*/
	//bones[THIGH_L].localMat = glm::rotate(bones[THIGH_L].localMat, -1.0f, glm::vec3(1.0, 0.0, 0.0));
	//bones[SHIN_L].localMat = glm::rotate(bones[SHIN_L].localMat, 1.0f, glm::vec3(1.0, 0.0, 0.0));
}

/*int Skeleton::getBoneByName(const char *n) {
	for (unsigned int i = 0; i < bones.size(); i++) {
		if (bones[i].name.compare(n) == 0)
			return i;
	}

	return -1;
}*/

Bone* Skeleton::getBone(int i) 
{
	return &bones[i];
}

void Skeleton::getInverseMatrices(std::vector<glm::mat4> &vec)
{
	vec.resize(0);

	for (unsigned int i = 0; i < bones.size(); i++)	{
		vec.push_back(bones[i].inverseBindMatrix);
	}
}

void Skeleton::getGlobalMatrices(std::vector<glm::mat4> &vec)
{
	vec.resize(0);

	for (unsigned int i = 0; i < bones.size(); i++)	{
		vec.push_back(bones[i].globalMat);
	}
}

void Skeleton::getSkinningMatrices(std::vector<glm::mat4> &vec)
{
	vec.resize(0);

	for (unsigned int i = 0; i < bones.size(); i++)	{
		vec.push_back(bones[i].globalMat * bones[i].inverseBindMatrix);
	}
}

void Skeleton::getTISkinningMatrices(std::vector<glm::mat3> &vec)
{
	vec.resize(0);

	for (unsigned int i = 0; i < bones.size(); i++)	{
		vec.push_back(glm::transpose(glm::inverse(glm::mat3(bones[i].globalMat) * glm::mat3(bones[i].inverseBindMatrix))));
	}
}

void Skeleton::setRootTransformMatrix(glm::mat4 m)
{
	rootTransform = m;
}

glm::mat4 Skeleton::getRootTransformMatrix()
{
	return  rootTransform;
}