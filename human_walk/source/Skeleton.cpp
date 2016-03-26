#include "Skeleton.h"

Skeleton::Skeleton(std::shared_ptr<Terrain> t)
{
	terrain = t;
}

void Skeleton::init()
{
	countGlobalMatrices();

	rightLeg = Leg(terrain, &bones[HIPS], &bones[THIGH_R], &bones[SHIN_R], &bones[FOOT_R], &bones[TOE_R], stepLength);
	leftLeg = Leg(terrain, &bones[HIPS], &bones[THIGH_L], &bones[SHIN_L], &bones[FOOT_L], &bones[TOE_L], stepLength);
	rightArm = Arm(&bones[NECK], &bones[UPPER_ARM_R], &bones[FOREARM_R], &bones[HAND_R], 0.5);
	leftArm = Arm(&bones[NECK], &bones[UPPER_ARM_L], &bones[FOREARM_L], &bones[HAND_L], 0.5);

	staticRootPos = glm::vec3(bones[HIPS].globalMat[3]);
	prevRootPos = staticRootPos;
	staticRootPosSpeed = staticRootPos;
	nextRootPos = prevRootPos + glm::vec3(0.0, 0.0, stepLength * 0.5f);

	maxPelvisHeight += bones[HIPS].globalMat[3].y;
	pelvisVerticalCurve = BezierCurve(&pelvisVerticalControlPoints[0][0], &pelvisVerticalControlPoints[1][0], 8, 2, false);
	pelvisLateralCurve = BezierCurve(&pelvisLateralControlPoints[0][0], &pelvisLateralControlPoints[1][0], 8, 2, true);
	pelvicRotationCurve = BezierCurve(&pelvicRotationControlPoints[0][0], &pelvicRotationControlPoints[1][0], 8, 2, true);
	shoulderRotationCurve = BezierCurve(&shoulderRotationControlPoints[0][0], &shoulderRotationControlPoints[1][0], 8, 2, true);
	pelvicTiltCurve = BezierCurve(&pelvicTiltControlPoints[0][0], &pelvicTiltControlPoints[1][0], 16, 2, true);
	pelvicTiltForwardCurve = BezierCurve(&pelvicTiltForwardControlPoints[0][0], &pelvicTiltForwardControlPoints[1][0], 8, 2, false);
	pelvisSpeedCurve = BezierCurve(&pelvisSpeedControlPoints[0][0], &pelvisSpeedControlPoints[1][0], 8, 2, false);

	uniformXVec.resize(4);
	for (auto i = 0; i < uniformXVec.size(); i++)
		uniformXVec[i] = uniformX[i];

	glm::vec3 hipsCenter = glm::vec3(bones[THIGH_R].globalMat[3] + bones[THIGH_L].globalMat[3]) / 2.0f;

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
	leftLeg.setNextPosition(stepLength);
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
	//dt *= 0.7f;
	t += dt;

	if (t > TERMINAL_SWING) {
		t -= TERMINAL_SWING;
		stepSpeedAccuracyCheck += (dt - t) * stepLength * (pelvisSpeedCurve.YfromX(1.0) + 1.0);
		speedDispFix = stepLength - stepSpeedAccuracyCheck;
		stepSpeedAccuracyCheck = 0.0;
		std::cout << "speedDispFix: " << speedDispFix << std::endl;
		std::cout << "bones[HIPS].localMat[3].z: " << bones[HIPS].localMat[3].z << std::endl;

		//if (leftLeg.getIKFalse() || rightLeg.getIKFalse())
			//increaseMaxPelvisHeight(-0.01);

		if (!heightTest)
			heightTestHeight = 0.0f;
		float fixedStepLength = stepLength;// rightLeg.getStepLengthSum() - leftLeg.getStepLengthSum() + stepLength * 0.5f - abs(leftLeg.getPrevPosition().y - heightTestHeight);
		nextRootPos += glm::vec3(0.0, 0.0, fixedStepLength * 0.5f);
		prevRootPos = staticRootPosSpeed;

		//pelvisVerticalCurve.incrementAddCoeff(0.05);
		leftLeg.setNextPosition(fixedStepLength); // , heightTestHeight);
		heightTestHeight += heightTestHeight < 0.15 ? 0.05f : -0.08;
		float heightDiff = rightLeg.getNextPosition().y - leftLeg.getPrevPosition().y;
		if (heightDiff > 0.0) {
			pelvisVerticalCurve.incrementAddCoeff(heightDiff);
			leftLeg.swingCurveIncrease(heightDiff);
		}
	}

	if (t >= 0.25 && t - dt < 0.25) {
		float heightDiff = leftLeg.getNextPosition().y - rightLeg.getPrevPosition().y;
		if (heightDiff < 0.0)
			pelvisVerticalCurve.incrementAddCoeff(heightDiff);
	}

	if (t >= 0.5 && t - dt < 0.5) {
		if (!heightTest)
			heightTestHeight = 0.0f;
		float fixedStepLength = stepLength;// leftLeg.getStepLengthSum() - rightLeg.getStepLengthSum() + stepLength * 0.5f - abs(rightLeg.getPrevPosition().y - heightTestHeight);
		nextRootPos += glm::vec3(0.0, 0.0, fixedStepLength * 0.5f);
		prevRootPos = staticRootPosSpeed;
//		pelvisVerticalCurve.incrementAddCoeff(0.2);
		rightLeg.setNextPosition(fixedStepLength);// , heightTestHeight);
		heightTestHeight += heightTestHeight < 0.15 ? 0.05f : -0.08;
		float heightDiff = leftLeg.getNextPosition().y - rightLeg.getPrevPosition().y;
		if (heightDiff > 0.0) {
			pelvisVerticalCurve.incrementAddCoeff(heightDiff);
			rightLeg.swingCurveIncrease(heightDiff);
		}
	}

	if (t >= 0.75 && t - dt < 0.75) {
		float heightDiff = rightLeg.getNextPosition().y - leftLeg.getPrevPosition().y;
		if (heightDiff < 0.0)
			pelvisVerticalCurve.incrementAddCoeff(heightDiff);
	}

	float pelvicVerticalT = t - LOADING_RESPONSE * 0.5 + 1.0;
	pelvicVerticalT -= static_cast<int>(pelvicVerticalT);

	float pelvicLateralT = t - LOADING_RESPONSE * 0.5 + 1.0;
	pelvicLateralT -= static_cast<int>(pelvicLateralT);

	float pelvicRotationT = t;// +1.0;

	float pelvicTiltForwardT = t - LOADING_RESPONSE + 1.0;
	pelvicTiltForwardT -= static_cast<int>(pelvicTiltForwardT);

	bones[HIPS].localMat[3].x = pelvisLateralCurve.YfromX(pelvicLateralT); // bezierCurve(pelvisLateralControlPointsVec, pelvicLateralT);
	bones[HIPS].localMat[3].y = pelvisVerticalCurve.YfromX(pelvicVerticalT) + maxPelvisHeight;//bezierCurve(pelvisVerticalControlPointsVec, uniformXVec, pelvicVerticalT);
	glm::vec3 forwardDisp = 2*dt * (nextRootPos - prevRootPos);
	//pelvisSpeedCurve.setCoeffImmediately(0.8);
	pelvisSpeedCurve.setCoeffImmediately(0.2);
	glm::vec3 forwardDispSpeed = 2*dt * (nextRootPos - prevRootPos) * (pelvisSpeedCurve.YfromX(pelvicVerticalT) + 1.0f);
	//stepSpeedAccuracyCheck += forwardDispSpeed;
	bones[HIPS].localMat[3] += glm::vec4(forwardDispSpeed, 0.0f);// *bezierCurve(pelvisSpeedControlPointsVec, pelvicVerticalTOld);
	leftLeg.translateStaticRoot(forwardDisp);
	rightLeg.translateStaticRoot(forwardDisp);
	leftArm.translateStaticRoot(forwardDisp);
	rightArm.translateStaticRoot(forwardDisp);
	staticRootPos += forwardDisp;
//	staticRootPos.y = bones[HIPS].localMat[3].y;
	staticRootPosSpeed += forwardDispSpeed;

	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, -pelvicTiltForward, glm::vec3(1.0, 0.0, 0.0));
	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, -pelvicTilt, glm::vec3(0.0, 0.0, 1.0));
	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, -pelvicRotation, glm::vec3(0.0, 1.0, 0.0));
	bones[SHOULDER_L].localMat = glm::rotate(bones[SHOULDER_L].localMat, -(2.0f * shoulderRotation / 3.0f), glm::vec3(1.0, 0.0, 0.0));
	bones[SHOULDER_R].localMat = glm::rotate(bones[SHOULDER_R].localMat, (2.0f * shoulderRotation / 3.0f), glm::vec3(1.0, 0.0, 0.0));
	bones[SPINE].localMat = glm::rotate(bones[SPINE].localMat, (2.0f * shoulderRotation / 3.0f), glm::vec3(0.0, 1.0, 0.0));
	bones[SPINE].localMat = glm::rotate(bones[SPINE].localMat, -spineRot, glm::vec3(0.0, 0.0, 1.0));
	bones[CHEST].localMat = glm::rotate(bones[CHEST].localMat, (2.0f * shoulderRotation / 3.0f), glm::vec3(0.0, 1.0, 0.0));
	bones[CHEST].localMat = glm::rotate(bones[CHEST].localMat, -spineRot, glm::vec3(0.0, 0.0, 1.0));
	float shoulderRotFix = ((4.0f / 3.0f) * shoulderRotation - pelvicRotation);
	float shoulderTiltFix = pelvicTilt + 2.0f * spineRot;
	bones[NECK].localMat = glm::rotate(bones[NECK].localMat, -(shoulderRotFix / 2.0f), glm::vec3(0.0, 1.0, 0.0));
	bones[NECK].localMat = glm::rotate(bones[NECK].localMat, shoulderTiltFix * 0.5f, glm::vec3(0.0, 0.0, 1.0));
	bones[HEAD].localMat = glm::rotate(bones[HEAD].localMat, -(shoulderRotFix / 2.0f), glm::vec3(0.0, 1.0, 0.0));
	bones[HEAD].localMat = glm::rotate(bones[HEAD].localMat, shoulderTiltFix * 0.5f, glm::vec3(0.0, 0.0, 1.0));

	pelvicRotation = pelvicRotationCurve.YfromX(pelvicRotationT);//bezierCurve(pelvicRotationControlPointsVec, pelvicRotationT);
	shoulderRotation = shoulderRotationCurve.YfromX(pelvicRotationT);
	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, pelvicRotation, glm::vec3(0.0, 1.0, 0.0));

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
}

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