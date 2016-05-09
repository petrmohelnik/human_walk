#include "Skeleton.h"

Skeleton::Skeleton(std::shared_ptr<Terrain> t)
{
	terrain = t;
}

float Skeleton::pelvisSpeedCurve(float _t)
{
	if (_t > 1.0f)
		_t = 1.0f;
	if (_t <= 0.5f)
		return pelvisSpeed1Curve->YfromX(_t * 2.0f);
	else 
		return pelvisSpeed1Curve->getNextCoeff() + pelvisSpeed1Curve->getLastAddCoeff() + pelvisSpeed2Curve->YfromX((_t - 0.5f) * 2.0f);
}

void Skeleton::init()
{
	countGlobalMatrices();

	staticRootMat = std::make_shared<glm::mat4>(bones[HIPS].globalMat);
	cameraPos = glm::vec3(bones[HIPS].globalMat[3]);
	prevRootPos = std::make_shared<glm::vec3>();
	nextRootPos = std::make_shared<glm::vec3>();
	maxPelvisHeight = std::make_shared<float>(0.0f);
	jointWeights = glm::vec3(1.0f);

	*maxPelvisHeight += bones[HIPS].globalMat[3].y;
	defaultPelvisHeight = bones[HIPS].globalMat[3].y;
	pelvisVerticalCurve = std::make_shared<BezierCurve>(BezierCurve(&pelvisVerticalControlPoints[0][0], &pelvisVerticalControlPoints[1][0], 8, 1, false));
	pelvisLateralCurve = BezierCurve(&pelvisLateralControlPoints[0][0], &pelvisLateralControlPoints[1][0], 8, 2, true);
	pelvicRotationCurve = BezierCurve(&pelvicRotationControlPoints[0][0], &pelvicRotationControlPoints[1][0], 8, 2, true);
	shoulderRotationCurve = BezierCurve(&shoulderRotationControlPoints[0][0], &shoulderRotationControlPoints[1][0], 8, 2, true);
	pelvicTiltCurve = BezierCurve(&pelvicTiltControlPoints[0][0], &pelvicTiltControlPoints[1][0], 16, 2, true);
	pelvicTiltForwardCurve = BezierCurve(&pelvicTiltForwardControlPoints[0][0], &pelvicTiltForwardControlPoints[1][0], 8, 2, false);
	pelvisSpeed1Curve = std::make_shared<BezierCurve>(BezierCurve(&pelvisSpeed1ControlPoints[0][0], &pelvisSpeed1ControlPoints[1][0], 4, 1, false));
	pelvisSpeed2Curve = std::make_shared<BezierCurve>(BezierCurve(&pelvisSpeed2ControlPoints[0][0], &pelvisSpeed2ControlPoints[1][0], 4, 1, false));

	*nextRootPos = glm::vec3((*staticRootMat)[3]);
	pelvisSpeed1Curve->setCoeffImmediately(stepLength / 4.0f);
	pelvisSpeed2Curve->setCoeffImmediately(stepLength / 4.0f);
	nextRootPos->z += (stepLength / 2.0f) - pelvisSpeedCurve(1.0f - LOADING_RESPONSE * 0.5f);
	*prevRootPos = *nextRootPos;
	prevRootPos->z -= (stepLength / 2.0f);
	*nextRootPos = *prevRootPos + glm::vec3(0.0, 0.0, stepLength * 0.5f);

	rightLeg = Leg(terrain, &bones[HIPS], &bones[THIGH_R], &bones[SHIN_R], &bones[FOOT_R], &bones[TOE_R], stepLength);
	leftLeg = Leg(terrain, &bones[HIPS], &bones[THIGH_L], &bones[SHIN_L], &bones[FOOT_L], &bones[TOE_L], stepLength);
	rightArm = Arm(&bones[NECK], &bones[UPPER_ARM_R], &bones[FOREARM_R], &bones[HAND_R], 0.5, staticRootMat);
	leftArm = Arm(&bones[NECK], &bones[UPPER_ARM_L], &bones[FOREARM_L], &bones[HAND_L], 0.5, staticRootMat);

	glm::vec3 hipsCenter = glm::vec3(bones[THIGH_R].globalMat[3] + bones[THIGH_L].globalMat[3]) / 2.0f;

	bones[HIPS].localMat[3].x = pelvisLateralCurve.YfromX(1.0f - LOADING_RESPONSE * 0.5f);//bezierCurve(pelvisLateralControlPointsVec, 2.0 * MID_STANCE);
	bones[HIPS].localMat[3].y = pelvisVerticalCurve->YfromX(1.0f - LOADING_RESPONSE) + *maxPelvisHeight;//bezierCurve(pelvisVerticalControlPointsVec, uniformXVec, 2.0 * LOADING_RESPONSE);
	pelvicRotation = pelvicRotationCurve.YfromX(0.0f);//bezierCurve(pelvicRotationControlPointsVec, 0.0);
	shoulderRotation = shoulderRotationCurve.YfromX(0.0f);
	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, pelvicRotation, glm::vec3(0.0, 1.0, 0.0));
	pelvicTilt = pelvicTiltCurve.YfromX(0.96f); // bezierCurve(pelvicTiltControlPointsVec, 0.0);
	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, pelvicTilt, glm::vec3(0.0, 0.0, 1.0));
	pelvicTiltForward = pelvicTiltForwardCurve.YfromX(1.0f - LOADING_RESPONSE);//bezierCurve(pelvicTiltForwardControlPointsVec, 4.0 * LOADING_RESPONSE);
	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, pelvicTiltForward, glm::vec3(1.0, 0.0, 0.0));
	countGlobalMatrices();

	bones[SHOULDER_L].localMat = glm::rotate(bones[SHOULDER_L].localMat, (2.0f * shoulderRotation / 3.0f), glm::vec3(1.0, 0.0, 0.0));
	bones[SHOULDER_R].localMat = glm::rotate(bones[SHOULDER_R].localMat, -(2.0f * shoulderRotation / 3.0f), glm::vec3(1.0, 0.0, 0.0));
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

	rightLeg.init(INIT_HEEL_STRIKE, hipsCenter, pelvisVerticalCurve, pelvisSpeed1Curve, pelvisSpeed2Curve,  maxPelvisHeight, prevRootPos, nextRootPos, staticRootMat);
	leftLeg.init(INIT_TERMINAL_STANCE, hipsCenter, pelvisVerticalCurve, pelvisSpeed1Curve, pelvisSpeed2Curve, maxPelvisHeight, prevRootPos, nextRootPos, staticRootMat);
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

float Skeleton::configurePelvisFindBestScore(Leg &stanceLeg, Leg &swingLeg, int iter, float lowRange, float highRange, float actVerticalCoeff,
	float lastVerticaCoeff)
{
	std::vector<AngleDifferences> angleDifferences1, angleDifferences2;
	angleDifferences1.reserve(PELVIS_CONFIGURATION_STEPS + 1); angleDifferences2.reserve(PELVIS_CONFIGURATION_STEPS + 1);
	int lowestI = -1;
	float lowestAcceleration = 100000000.0f;
	int lowestErrorI = -1;
	float lowestError = 100000000.0f;
	for (int i = 0; i < iter; i++)
	{
		angleDifferences1.resize(0); angleDifferences2.resize(0);
		float angleAcceleration = 0.0f;

		glm::mat4 testThighLocal = stanceLeg.getThigh()->localMat;
		glm::mat4 testShinLocal = stanceLeg.getShin()->localMat;
		float testKneeRot = stanceLeg.getKneeRot();
		glm::mat4 testFootLocal = stanceLeg.getFoot()->localMat;

		pelvisSpeed1Curve->setAddCoeff(2, stanceLeg.getPelvisMidStancePos().z - (*nextRootPos).z - pelvisSpeed1Curve->getNextCoeff());
		pelvisSpeed1Curve->setAddCoeff(3, stanceLeg.getPelvisMidStancePos().z - (*nextRootPos).z - pelvisSpeed1Curve->getNextCoeff());
		pelvisSpeed1Curve->recountOnNext();

		float range = swingLeg.getPelvisMidStancePos().z - stanceLeg.getPelvisMidStancePos().z;
		pelvisSpeed2Curve->setAddCoeff(2, lerp(range * lowRange, range * highRange, i * (1.0f / iter)) - pelvisSpeed2Curve->getNextCoeff());
		pelvisSpeed2Curve->setAddCoeff(3, lerp(range * lowRange, range * highRange, i * (1.0f / iter)) - pelvisSpeed2Curve->getNextCoeff());
		pelvisSpeed2Curve->recountOnNext();

		float heightDiff = stanceLeg.getNextPosition().y - swingLeg.getPrevPosition().y;
		pelvisVerticalCurve->setAddCoeff(actVerticalCoeff);
		pelvisVerticalCurve->resetPelvisAddCoeff(lastVerticaCoeff);
		pelvisVerticalCurve->recountOnNext();
		if (heightDiff > 0.0f) {
			pelvisVerticalCurve->pelvisUp(heightDiff);
		}
		heightDiff = swingLeg.getNextPosition().y - stanceLeg.getPrevPosition().y;
		if (heightDiff < 0.0)
			pelvisVerticalCurve->pelvisDown(heightDiff);

		float pelvicVerticalT = 0.5f;
		float error = 0.0f;
		glm::mat4 testRootLocal = *staticRootMat;
		//rotace zatim zanedbam i lateral
		testRootLocal[3].y = pelvisVerticalCurve->YfromX(pelvicVerticalT) + *maxPelvisHeight;
		testRootLocal[3].z = prevRootPos->z + pelvisSpeedCurve(pelvicVerticalT);

		glm::mat4 testThighGlobal = testRootLocal * testThighLocal;

		stanceLeg.pelvisConfigurationInitMidSwing(testThighGlobal, testThighLocal, testShinLocal,
			testKneeRot, testFootLocal, testRootLocal);
		for (; pelvicVerticalT <= 1.0f; pelvicVerticalT += 0.5f / PELVIS_CONFIGURATION_STEPS)
		{
			testRootLocal = *staticRootMat;
			testRootLocal[3].y = pelvisVerticalCurve->YfromX(pelvicVerticalT) + *maxPelvisHeight;
			testRootLocal[3].z = prevRootPos->z + pelvisSpeedCurve(pelvicVerticalT);

			testThighGlobal = testRootLocal * testThighLocal;
			//predam noze chodidlo a ono se tam srovna a urci se uhel o kolik se otocil

			//co tomu budu predavat a co se vlastne pocita

			error += stanceLeg.solveIKStanceLegPelvisConfiguration(pelvicVerticalT * 0.5f, testThighGlobal, testThighLocal, testShinLocal,
				testKneeRot, testFootLocal, testRootLocal, angleDifferences1);

		}

		//pro svihovou nohu

		//nova vertical curve a speed curva1
		//....
		glm::mat4 testRootLocalHeelStrike = *staticRootMat;
		testRootLocalHeelStrike[3].y = pelvisVerticalCurve->YfromX(1.0f - LOADING_RESPONSE) + *maxPelvisHeight;
		testRootLocalHeelStrike[3].z = prevRootPos->z + pelvisSpeedCurve(1.0f - LOADING_RESPONSE);
		testThighGlobal = testRootLocalHeelStrike * testThighLocal;

		testRootLocal = *staticRootMat;
		testRootLocal[3].y = pelvisVerticalCurve->YfromX(1.0f) + *maxPelvisHeight;
		testRootLocal[3].z = prevRootPos->z + pelvisSpeedCurve(1.0f);

		pelvisSpeed1Curve->setAddCoeff(2, swingLeg.getPelvisMidStancePos().z -
			(stanceLeg.getPelvisMidStancePos().z + pelvisSpeed2Curve->getNextCoeff() + pelvisSpeed2Curve->getLastAddCoeff()) - pelvisSpeed1Curve->getNextCoeff());
		pelvisSpeed1Curve->setAddCoeff(3, swingLeg.getPelvisMidStancePos().z -
			(stanceLeg.getPelvisMidStancePos().z + pelvisSpeed2Curve->getNextCoeff() + pelvisSpeed2Curve->getLastAddCoeff()) - pelvisSpeed1Curve->getNextCoeff());
		pelvisSpeed1Curve->recountOnNext();

		pelvisVerticalCurve->resetPelvisAddCoeff();
		pelvisVerticalCurve->recountOnNext();
		heightDiff = swingLeg.getNextPosition().y - stanceLeg.getPrevPosition().y;
		if (heightDiff > 0.0)
			pelvisVerticalCurve->pelvisUp(heightDiff);

		pelvicVerticalT = 0.0f;
		testThighLocal = swingLeg.getThigh()->localMat;
		testShinLocal = swingLeg.getShin()->localMat;
		testKneeRot = swingLeg.getKneeRot();
		testFootLocal = swingLeg.getFoot()->localMat;
		if (!swingLeg.pelvisConfigurationInitMidStance(testThighGlobal, testThighLocal, testShinLocal,
			testKneeRot, testFootLocal, testRootLocal, testRootLocalHeelStrike))
			error += 1000.0f;
		for (; pelvicVerticalT <= 0.5f; pelvicVerticalT += 0.5f / PELVIS_CONFIGURATION_STEPS)
		{
			testRootLocal = *staticRootMat;
			testRootLocal[3].y = pelvisVerticalCurve->YfromX(pelvicVerticalT) + *maxPelvisHeight;
			testRootLocal[3].z = stanceLeg.getPelvisMidStancePos().z + pelvisSpeed2Curve->getNextCoeff() +
				pelvisSpeed2Curve->getLastAddCoeff() + pelvisSpeedCurve(pelvicVerticalT);

			testThighGlobal = testRootLocal * testThighLocal;

			error += swingLeg.solveIKStanceLegPelvisConfiguration(pelvicVerticalT * 0.5f, testThighGlobal, testThighLocal, testShinLocal,
				testKneeRot, testFootLocal, testRootLocal, angleDifferences2);

		}
		//////

		for (unsigned int j = 1; j < angleDifferences1.size(); j++) {
			angleAcceleration += jointWeights.x * abs(angleDifferences1[j].hip - angleDifferences1[j - 1].hip);
		}
		for (unsigned int j = 1; j < angleDifferences1.size(); j++) {
			angleAcceleration += jointWeights.y * abs(angleDifferences1[j].knee - angleDifferences1[j - 1].knee);
		}
		for (unsigned int j = 1; j < angleDifferences1.size(); j++) {
			angleAcceleration += jointWeights.z * abs(angleDifferences1[j].ankle - angleDifferences1[j - 1].ankle);
		}
		for (unsigned int j = 1; j < angleDifferences2.size(); j++) {
			angleAcceleration += jointWeights.x * abs(angleDifferences2[j].hip - angleDifferences2[j - 1].hip);
		}
		for (unsigned int j = 1; j < angleDifferences2.size(); j++) {
			angleAcceleration += jointWeights.y * abs(angleDifferences2[j].knee - angleDifferences2[j - 1].knee);
		}
		for (unsigned int j = 1; j < angleDifferences2.size(); j++) {
			angleAcceleration += jointWeights.z * abs(angleDifferences2[j].ankle - angleDifferences2[j - 1].ankle);
		}

		if (error != 0.0f && error < lowestError) {
			lowestError = error;
			lowestErrorI = i;
		}
		if (error == 0.0f && angleAcceleration < lowestAcceleration) {
			lowestAcceleration = angleAcceleration;
			lowestI = i;
		}
	}

	return lerp(lowRange, highRange, (lowestI < 0 ? lowestErrorI : lowestI) * (1.0f / iter));
}

void Skeleton::configurePelvis(Leg &stanceLeg, Leg &swingLeg)
{
	*prevRootPos = *nextRootPos;
	(*staticRootMat)[3] = glm::vec4(*nextRootPos, 1.0);
	pelvisSpeed2Curve->setCoeffImmediately(stepLength / 4);
	pelvisSpeed1Curve->setCoeffImmediately(stepLength / 4);

	float actVerticalAddCoeff = pelvisVerticalCurve->getNextAddCoeff();
	float lastVerticalAddCoeff = pelvisVerticalCurve->getLastAddCoeff();
	float actVerticalCoeff = pelvisVerticalCurve->getActCoeff();
	float nextVerticalCoeff = pelvisVerticalCurve->getNextCoeff();

	int iter = 10;
	float lowRange = 0.35f;
	float highRange = 0.65f;

	float firstResult = configurePelvisFindBestScore(stanceLeg, swingLeg, iter, lowRange, highRange, actVerticalAddCoeff, lastVerticalAddCoeff);
	float secondResult = configurePelvisFindBestScore(stanceLeg, swingLeg, iter, firstResult - ((highRange - lowRange) / (iter - 1)),
		firstResult + ((highRange - lowRange) / (iter - 1)), actVerticalAddCoeff, lastVerticalAddCoeff);


 	pelvisSpeed1Curve->setAddCoeff(2, stanceLeg.getPelvisMidStancePos().z - (*nextRootPos).z - pelvisSpeed1Curve->getNextCoeff());
	pelvisSpeed1Curve->setAddCoeff(3, stanceLeg.getPelvisMidStancePos().z - (*nextRootPos).z - pelvisSpeed1Curve->getNextCoeff());
	pelvisSpeed1Curve->recountOnNext();
	float range = swingLeg.getPelvisMidStancePos().z - stanceLeg.getPelvisMidStancePos().z;
	pelvisSpeed2Curve->setAddCoeff(2, range * secondResult - pelvisSpeed2Curve->getNextCoeff());
	pelvisSpeed2Curve->setAddCoeff(3, range * secondResult - pelvisSpeed2Curve->getNextCoeff());
	pelvisSpeed2Curve->recountOnNext();

	std::cout << "secondResult: " << range * secondResult << std::endl;

	float heightDiff = stanceLeg.getNextPosition().y - swingLeg.getPrevPosition().y;
	nextRootPos->y += heightDiff;

	pelvisVerticalCurve->setAddCoeff(actVerticalAddCoeff);
	pelvisVerticalCurve->resetPelvisAddCoeff(lastVerticalAddCoeff);
	pelvisVerticalCurve->setCoeffImmediately(actVerticalCoeff);
	pelvisVerticalCurve->setCoeff(nextVerticalCoeff);
	pelvisVerticalCurve->recountOnNext();
	if (heightDiff > 0.0f) {
		pelvisVerticalCurve->pelvisUp(heightDiff);
	}
	heightDiff = swingLeg.getNextPosition().y - stanceLeg.getPrevPosition().y;
	if (heightDiff < 0.0)
		pelvisVerticalCurve->pelvisDown(heightDiff);

	*nextRootPos += glm::vec3(0.0, 0.0, pelvisSpeed1Curve->getNextCoeff() + pelvisSpeed1Curve->getLastAddCoeff() +
		pelvisSpeed2Curve->getNextCoeff() + pelvisSpeed2Curve->getLastAddCoeff());
}

void Skeleton::onUpdate(float dt)
{
	dt *= timeSpeedCoeff;
	dt = dt > 0.04f ? 0.04f : dt;
	t += dt;

	if (t > TERMINAL_SWING)
		t -= TERMINAL_SWING;

	//std::cout << bones[HIPS].localMat[3].z << std::endl;

	if (t >= LOADING_RESPONSE * 0.5f && t - dt < LOADING_RESPONSE * 0.5f) {
		float distToStance = rightLeg.getStepLengthSum() - leftLeg.getStepLengthSum();
		leftLeg.setNextPosition(distToStance + stepLength * 0.5f); // , heightTestHeight);
		rightLeg.setStepLength(stepLength);

		configurePelvis(rightLeg, leftLeg);
	}

	if (t >= 0.5 + LOADING_RESPONSE * 0.5 && t - dt < 0.5 + LOADING_RESPONSE * 0.5) {
		float distToStance = leftLeg.getStepLengthSum() - rightLeg.getStepLengthSum();
		rightLeg.setNextPosition(distToStance + stepLength * 0.5f);// , heightTestHeight);
		leftLeg.setStepLength(stepLength);

		configurePelvis(leftLeg, rightLeg);
	}


	float pelvicVerticalT = t - LOADING_RESPONSE * 0.5f + 1.0f;
	pelvicVerticalT -= static_cast<int>(pelvicVerticalT);
	pelvicVerticalT = pelvicVerticalT > 0.5f ? (pelvicVerticalT - 0.5f) * 2.0f : pelvicVerticalT * 2.0f;

	float pelvicLateralT = t - LOADING_RESPONSE * 0.5f + 1.0f;
	pelvicLateralT -= static_cast<int>(pelvicLateralT);

	float pelvicRotationT = t;// +1.0;

	float pelvicTiltForwardT = t - LOADING_RESPONSE + 1.0f;
	pelvicTiltForwardT -= static_cast<int>(pelvicTiltForwardT);

	bones[HIPS].localMat[3].x = pelvisLateralCurve.YfromX(pelvicLateralT); // bezierCurve(pelvisLateralControlPointsVec, pelvicLateralT);
	bones[HIPS].localMat[3].y = pelvisVerticalCurve->YfromX(pelvicVerticalT) + *maxPelvisHeight;//bezierCurve(pelvisVerticalControlPointsVec, uniformXVec, pelvicVerticalT);
	glm::vec3 forwardDisp = 2*dt * (*nextRootPos - *prevRootPos);
	
	bones[HIPS].localMat[3].z = prevRootPos->z + pelvisSpeedCurve(pelvicVerticalT >= 1.0f ? pelvicVerticalT - 1.0f : pelvicVerticalT); //pelvisSpeedCurve->YfromX(pelvicVerticalT);
	//std::cout << "bones[HIPS].localMat[3].z:" << bones[HIPS].localMat[3].z << std::endl;
	(*staticRootMat)[3] += glm::vec4(forwardDisp, 0.0);
	cameraPos.y += dt * stepLength * ((*nextRootPos).y - cameraPos.y);
	cameraPos.z += dt * stepLength * glm::normalize((*nextRootPos).z - cameraPos.z);


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

	float pelvicTiltT = t - 0.04f + 1.0f;
	pelvicTiltT -= static_cast<int>(pelvicTiltT);
	pelvicTilt = pelvicTiltCurve.YfromX(pelvicTiltT);
	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, pelvicTilt, glm::vec3(0.0, 0.0, 1.0));
	pelvicTiltForward = pelvicTiltForwardCurve.YfromX(pelvicTiltForwardT);//bezierCurve(pelvicTiltForwardControlPointsVec, pelvicTiltForwardT);
	bones[HIPS].localMat = glm::rotate(bones[HIPS].localMat, pelvicTiltForward, glm::vec3(1.0, 0.0, 0.0));

	bones[SHOULDER_L].localMat = glm::rotate(bones[SHOULDER_L].localMat, (2.0f * shoulderRotation / 3.0f), glm::vec3(1.0, 0.0, 0.0));
	bones[SHOULDER_R].localMat = glm::rotate(bones[SHOULDER_R].localMat, -(2.0f * shoulderRotation / 3.0f), glm::vec3(1.0, 0.0, 0.0));
	spineRot = solveSpine(0.0, pelvicTilt);

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

	countGlobalMatrices();
	leftLeg.update(dt);
	rightLeg.update(dt);
	leftArm.update(dt, shoulderRotFix, shoulderRotFixUpdated, shoulderTiltFix, shoulderTiltFixUpdated);
	rightArm.update(dt, shoulderRotFix, shoulderRotFixUpdated, shoulderTiltFix, shoulderTiltFixUpdated);

	countGlobalMatrices();
}

void Skeleton::setWalkingSpeed(float speed)
{
	setStepLength(speed / 3.0f + 0.3666666f);
	setWalkingSpeed(speed / (stepLength * 2.0f));
	setPelvisLateralCoeff((-speed / 50.0f + 0.049f) / 0.02f);
	setPelvisVerticalCoeff((speed / 50.0f + 0.016f) / 0.05f);
	setStepWidth(-speed / 32.0f + 0.135f);
}

void Skeleton::getHeelSwingCurvePoints(std::vector<float> &vec)
{
	if (t > LOADING_RESPONSE && t < LOADING_RESPONSE + 0.5f)
		leftLeg.getHeelSwingCurvePoints(vec);
	else
		rightLeg.getHeelSwingCurvePoints(vec);
}

void Skeleton::getPelvisVerticalCurvePoints(std::vector<float> &vec)
{
	const BezierCurve *curve = pelvisVerticalCurve.get();
	for (unsigned int i = 0; i < vec.size(); i++) {
		vec[i] = curve->YfromX(i * (1.0f / (vec.size() - 1)));
	}
}

void Skeleton::getPelvisSpeed1CurvePoints(std::vector<float> &vec)
{
	const BezierCurve *curve = pelvisSpeed1Curve.get();
	for (unsigned int i = 0; i < vec.size(); i++) {
		vec[i] = curve->YfromX(i * (1.0f / (vec.size() - 1)));
	}
}

void Skeleton::getPelvisSpeedCurvePoints(std::vector<float> &vec)
{
	/*for (unsigned int i = 0; i < vec.size(); i++) {
		vec[i] = pelvisSpeedCurve(i * (1.0f / (vec.size() - 1)));
	}*/

	const BezierCurve *curve1 = pelvisSpeed1Curve.get();
	const BezierCurve *curve2 = pelvisSpeed2Curve.get();

	for (unsigned int i = 0; i < vec.size(); i++) {
		float tt = i * (1.0f / (vec.size() - 1));
		if (tt <= 0.5f)
			vec[i] = curve1->YfromX(tt * 2.0f);
		else
			vec[i] = curve2->getActCoeff() + pelvisSpeed1Curve->getLastAddCoeff() + pelvisSpeed2Curve->YfromX((tt - 0.5f) * 2.0f);
	}
}

void Skeleton::getPelvisSpeed2CurvePoints(std::vector<float> &vec)
{
	const BezierCurve *curve = pelvisSpeed2Curve.get();
	for (unsigned int i = 0; i < vec.size(); i++) {
		vec[i] = curve->YfromX(i * (1.0f / (vec.size() - 1)));
	}
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