#include "Leg.h"

float lerp(float start, float end, float t)
{
	return start + (end - start) * t;
}

float Leg::pelvisSpeedCurve(std::shared_ptr<const BezierCurve> pSpeed1, std::shared_ptr<const BezierCurve> pSpeed2, float t)
{
	if (t <= 0.5f)
		return pSpeed1->YfromX(t * 2.0f);
	else
		//return pSpeed1->getNextCoeff() + pSpeed2->YfromX((t - 0.5f) * 2.0f);
		return pelvisSpeed1Curve->getNextCoeff() + pelvisSpeed1Curve->getLastAddCoeff() + pelvisSpeed2Curve->YfromX((t - 0.5f) * 2.0f);
}

Leg::Leg(std::shared_ptr<Terrain> ter, Bone *hips, Bone *sphericalJoint, Bone *hingeJoint, Bone *endEffector, Bone *fingers, float length)
{
	terrain = ter;
	root = hips;
	thigh = sphericalJoint;
	shin = hingeJoint;
	foot = endEffector;
	toe = fingers;
	stepLength = length;

	thighLength = glm::length(glm::vec3(thigh->globalMat[3] - shin->globalMat[3]));
	shinLength = glm::length(glm::vec3(shin->globalMat[3] - foot->globalMat[3]));

	//count rotation in the knee at the beggining
	//neccessary because i get rotation from origin, but i dont get transformation matrix at origin
	/*float L = glm::length(glm::vec3(thigh->globalMat[3] - foot->globalMat[3]));
	float phi = PI - acos((thighLength*thighLength + shinLength*shinLength - L*L) / (2 * thighLength*shinLength));
	kneeRot = phi;*/
	
	thighWidth = thigh->globalMat[3].x;
	footWidth = thigh->globalMat[3].x;
	glm::mat4 newThigh(1.0);
	newThigh = glm::rotate(newThigh, PI, glm::vec3(1.0, 0.0, 0.0));
	newThigh[3] = thigh->localMat[3];
	thighBindMat = newThigh;// thigh->localMat;
	heelPos = glm::vec3(glm::inverse(foot->globalMat) * glm::vec4(foot->globalMat[3].x, 0.0, 0.0, 1.0));
	toePos = glm::vec3(glm::inverse(foot->globalMat) * glm::vec4(toe->globalMat[3].x, 0.0, toe->globalMat[3].z, 1.0));
	pelvisMidStancePos = glm::vec3(root->localMat[3] - glm::vec4(foot->globalMat[3].x, 0.0, 0.0, 1.0));
	if (t < PRE_SWING)
		toeOffPos = glm::vec3(0.0);
	prevAnklePos = glm::vec3(foot->globalMat[3]);

	glm::mat4 newShin(1.0);
	newShin = glm::translate(newShin, glm::vec3(shin->localMat[3]));
	shin->localMat = newShin;

	//swingControlPointsVec.resize(4);
	swingCurve = BezierCurve(swingControlPoints[0], swingControlPoints[1], 12, 1, false);
	swingForwardCurve = BezierCurve(swingForwardControlPoints[0], swingForwardControlPoints[1], 4, 1, false);
	//swingRotControlPointsVec.resize(4);
	swingRotationCurve = BezierCurve(swingRotControlPoints[0], swingRotControlPoints[1], 4, 1, false);
	heelRiseCurve = BezierCurve(heelRiseControlPoints[0], heelRiseControlPoints[1], 4, 1, false);
	swingWidthFixCurve = BezierCurve(swingWidthFixControlPoints[0], swingWidthFixControlPoints[1], 8, 1, false);
	heelRiseCurve.setCoeffImmediately(stepLength / 1.6f);
}

void Leg::init(int init, glm::vec3 center, std::shared_ptr<const BezierCurve> pVert, std::shared_ptr<const BezierCurve> pSpeed1,
	std::shared_ptr<const BezierCurve> pSpeed2,	std::shared_ptr<const float> maxPelvisH, std::shared_ptr<const glm::vec3> _prevRootPos,
	std::shared_ptr<const glm::vec3> _nextRootPos, std::shared_ptr<const glm::mat4> staticRootM)
{
	pelvisVerticalCurve = pVert;
	pelvisSpeed1Curve = pSpeed1;
	pelvisSpeed2Curve = pSpeed2;
	prevRootPos = _prevRootPos;
	nextRootPos = _nextRootPos;
	maxPelvisHeight = maxPelvisH;
	staticRootMat = staticRootM;

	if (init == INIT_HEEL_STRIKE) {
		prevHeelPos = glm::vec3(center.x + footWidth, 0.0f, center.z + stepLength * 0.19f);
		nextHeelPos = prevHeelPos;
		//nextHeelPos.z += stepLength;
		//rotate foot while being mid air
		float angle = swingRotControlPoints[1][3];
		foot->localMat = glm::rotate(foot->localMat, -angle, glm::vec3(1.0f, 0.0f, 0.0f));
		footRot += angle;

		//set new heel position
		thigh->globalMat = root->localMat * thigh->localMat;
		shin->globalMat = thigh->globalMat * shin->localMat;
		updateFootGlobal();
		solveIK(getAnklePosFromHeel(prevHeelPos, foot->globalMat));

		//updateFootGlobal();
		//prevHeelPos = getHeelPos(prevAnklePos);

		//compute position again using new rotation so heel fits more accurately
		updateFootGlobal();
		thigh->globalMat = root->localMat * thigh->localMat;
		shin->globalMat = thigh->globalMat * shin->localMat;
		solveIK(getAnklePosFromHeel(prevHeelPos, foot->globalMat));

		//updateFootGlobal();
		//prevHeelPos = getHeelPos(prevAnklePos);
		//prevHeelPos.z -= stepLength;
		
		stepLengthSum += stepLength * 0.5f;

		t = 0.0;
	}
	else if (init == INIT_TERMINAL_STANCE) {
		prevHeelPos = glm::vec3(center.x + footWidth, 0.0f, center.z - stepLength * 0.31f);
		nextHeelPos = prevHeelPos;
		//nextHeelPos.z += stepLength;
		solveIK(getAnklePosFromHeel(prevHeelPos, foot->globalMat));
		updateFootGlobal();
		updateToeGlobal();

		//rotate foot so that it alignes with the ground
		float actRot = getFootAngleFromTerrain(foot->globalMat);
		foot->localMat = glm::rotate(foot->localMat, actRot, glm::vec3(1.0f, 0.0f, 0.0f));
		footRot -= actRot;

		//updateFootGlobal();
		//toeOffPos = getHeelPos(prevAnklePos).z; //position from which we start swing phase
		//updateFootGlobal();
		//prevHeelPos = glm::vec3(width, 0.0f, -0.35f);// getHeelPos(prevAnklePos);

		updateFootGlobal();
		updateToeGlobal();
		toePosInit = glm::vec3(toe->globalMat[3]);
		//prevToePos = toePosInit + prevAnklePos - glm::vec3(foot->globalMat[3]); //save toe pos so we know where to put foot while raising heel later

		thigh->globalMat = root->localMat * thigh->localMat;
		shin->globalMat = thigh->globalMat * shin->localMat;
		//riseHeel(baseVec, TERMINAL_STANCE_HEEL_RISE_ANGLE, TERMINAL_STANCE_HEEL_RISE_ANGLE, 1.0);
		float preSwingT = (TERMINAL_STANCE - MID_STANCE) / (PRE_SWING - MID_STANCE);
		riseHeel(heelRiseCurve.YfromX(preSwingT));

		updateFootGlobal();
		updateToeGlobal();
		toePosInit = glm::vec3(toe->globalMat[3]);
		//prevToePos = toePosInit + prevAnklePos - glm::vec3(foot->globalMat[3]);
		t = 0.5;
	}
}

float getAngle(const glm::vec3 &vec1, const glm::vec3 &vec2, const glm::mat4 &ref, const glm::vec3 &refDir)
{
	glm::vec3 refV = glm::normalize(glm::vec3(ref * glm::vec4(refDir, 1.0f)) - glm::vec3(ref[3]));
	float dot = glm::dot(vec1, vec2);
	float angle = acos(dot <= 1.0f ? dot : 1.0f); //we dont want undefined value when float is not accurate enough
	return glm::dot(glm::cross(vec1, vec2), refV) < 0.0 ? -angle : angle;
}

float Leg::getFootAngleFromTerrain(glm::mat4 &footGlobal)
{
	glm::vec3 toePos2 = getToePos(footGlobal);
	glm::vec3 heelPos2 = getHeelPos(glm::vec3(footGlobal[3]), footGlobal);
	glm::vec3 footVec = glm::normalize(toePos2 - heelPos2);
	toePos2.y = terrain->getHeight(toePos2);
	heelPos2.y = terrain->getHeight(heelPos2);
	glm::vec3 baseVec = glm::normalize(toePos2 - heelPos2);

	return getAngle(footVec, baseVec, footGlobal, glm::vec3(1.0, 0.0, 0.0));
}

glm::vec3 Leg::setNextPosition(float step)
{
	stepLengthSum += step;
	//stepLength = step;

	nextHeelPos.z += step;
	float height = terrain->getHeight(nextHeelPos);
	nextHeelPos.y = height;

//	swingCurve.setSwingFootHeight(prevHeelPos.y);
//	if (height > prevHeelPos.y)
//		swingCurve.stepUp(height - prevHeelPos.y);
//	else
//		swingCurve.stepDown(prevHeelPos.y - height);

	std::cout << glm::length(nextHeelPos - prevHeelPos) << std::endl;

	return nextHeelPos;
}

void Leg::update(float dt)
{
	//dt *= 0.1;
	t += dt;

	//std::cout << t << ": " << glm::length(getHeelPos(prevAnklePos) - getHeelPos(prevPrevAnklePos)) / dt << std::endl;
	prevPrevAnklePos = prevAnklePos;

	if (t > TERMINAL_SWING) {
		t -= TERMINAL_SWING;
		startNewStep = true; //so we know when to execute condition in initial phase
	}
	//faze zatezovani
	if (t < LOADING_RESPONSE) {
		if (startNewStep == true) {
			prevHeelPos = nextHeelPos;

			//move to next position
			solveIK(getAnklePosFromHeel(prevHeelPos, foot->globalMat));
			thigh->globalMat = root->localMat * thigh->localMat;
			shin->globalMat = thigh->globalMat * shin->localMat;

			//finish rotation
			float angle = swingRotationCurve.YfromX(1.0f) - footRot;
			foot->localMat = glm::rotate(foot->localMat, -angle, glm::vec3(1.0f, 0.0f, 0.0f));
			footRot += angle;
			updateFootGlobal();

			//move to next position with correct rotation
			solveIK(getAnklePosFromHeel(prevHeelPos, foot->globalMat));
			updateFootGlobal();
			thigh->globalMat = root->localMat * thigh->localMat;
			shin->globalMat = thigh->globalMat * shin->localMat;

			initialLoadingResponseRot = getFootAngleFromTerrain(foot->globalMat);

			std::cout << prevHeelPos.x << " " << prevHeelPos.y << " " << prevHeelPos.z << std::endl;
		}
		float loadingResposeT = pow(t / LOADING_RESPONSE, 1.0f);

		//rotate to correct position to get correct heel position
		float phi = lerp(initialLoadingResponseRot, 0.0f, loadingResposeT) - getFootAngleFromTerrain(foot->globalMat);
		foot->localMat = glm::rotate(foot->localMat, -phi, glm::vec3(1.0f, 0.0f, 0.0f));
		footRot += phi;
		updateFootGlobal();

		//move to previous heel position
		solveIK(getAnklePosFromHeel(prevHeelPos, foot->globalMat));
		updateFootGlobal();
		updateToeGlobal();

		//rotate again because moving ruined our rotation
		phi = lerp(initialLoadingResponseRot, 0.0f, loadingResposeT) - getFootAngleFromTerrain(foot->globalMat);
		foot->localMat = glm::rotate(foot->localMat, -phi, glm::vec3(1.0f, 0.0f, 0.0f));
		footRot += phi;
		updateFootGlobal();

		//prevHeelPos = getHeelPos(prevAnklePos);
		startNewStep = false;
	}
	//mezistoj
	else if (t < MID_STANCE) {
		//finish loading response
		if (t - dt < LOADING_RESPONSE) {
			float phi = 0.0f - getFootAngleFromTerrain(foot->globalMat);
			foot->localMat = glm::rotate(foot->localMat, -phi, glm::vec3(1.0f, 0.0f, 0.0f));
			footRot += phi;
			updateFootGlobal();

			prevAnklePos = getAnklePosFromHeel(prevHeelPos, foot->globalMat);
		}

		solveIK(prevAnklePos);
		updateFootGlobal();
		updateToeGlobal();

		//rotate foot so that its alignes with the ground
		float actRot = getFootAngleFromTerrain(foot->globalMat);
		foot->localMat = glm::rotate(foot->localMat, actRot, glm::vec3(1.0f, 0.0f, 0.0f));
		footRot -= actRot;
	}
	else if (t < PRE_SWING) {
		if (t - dt < MID_STANCE) {
			toeOffPos = getHeelPos(prevAnklePos, foot->globalMat); //position from which we start swing phase
			toePosInit = glm::vec3(toe->globalMat[3]) + prevAnklePos - glm::vec3(foot->globalMat[3]);
			heelRiseCurve.setCoeffImmediately(pow(stepLength / 1.8f, 2.0f));
		}

		float preSwingT = (t - MID_STANCE) / (PRE_SWING - MID_STANCE);  //(t - TERMINAL_STANCE) / (PRE_SWING - TERMINAL_STANCE);

		riseHeel(heelRiseCurve.YfromX(preSwingT));
	}
	//pocatecni svih, mezisvih a konecny svih
	else if (t < TERMINAL_SWING) {
		if (t - dt < PRE_SWING) {
			//finish heel rise
			riseHeel(heelRiseCurve.YfromX(1.0f));

			swingDist = glm::length(glm::vec3(nextHeelPos.x, 0.0, nextHeelPos.z) - glm::vec3(getHeelPos(prevAnklePos, foot->globalMat).x, 0.0, getHeelPos(prevAnklePos, foot->globalMat).z));
			toeOffPos = getHeelPos(prevAnklePos, foot->globalMat);
			swingForwardCurve.setCoeffImmediately(swingDist);

			updateFootGlobal();

			configureSwing();

			terminalSwingInitToeRot = toeRot;
		}
		float swingT = (t - PRE_SWING) / (TERMINAL_SWING - PRE_SWING);

		//rotate toe back to its original position
		if (swingT <= 0.2f) {
			float angle = lerp(terminalSwingInitToeRot, 0.0f, 5.0f * swingT) - toeRot;
			toe->localMat = glm::rotate(toe->localMat, -angle, glm::vec3(1.0f, 0.0f, 0.0f));
			toeRot += angle;
		}
		else if (toeRot != 0.0) {
			toe->localMat = glm::rotate(toe->localMat, toeRot, glm::vec3(1.0f, 0.0f, 0.0f));
			toeRot = 0.0;
		}

		float widthDisp = root->localMat[3].x; //displacement to width of swing phase so foot is under hip and not on the side
		if (swingT <= 0.5) {
			widthDisp = lerp(toeOffPos.x, root->localMat[3].x + thighWidth, swingWidthFixCurve.YfromX(swingT));
		}
		if (swingT >= 0.5) {
			widthDisp = lerp(footWidth, root->localMat[3].x + thighWidth, swingWidthFixCurve.YfromX(swingT));
		}

		//rotate foot while being mid air
		float angle = swingRotationCurve.YfromX(swingT) - footRot;//bezierCurve(swingRotControlPointsVec, swingT) - footRot;
		foot->localMat = glm::rotate(foot->localMat, -angle, glm::vec3(1.0f, 0.0f, 0.0f));
		footRot += angle;

		//set new heel position
		updateFootGlobal();
		float heelY = swingCurve.YfromX(swingT);
		float heelZ = toeOffPos.z + swingForwardCurve.YfromX(swingT);
		solveIK(getAnklePosFromHeel(glm::vec3(widthDisp, heelY, heelZ), foot->globalMat));
	
		//compute position again using new rotation so heel fits the curve more accurately
		updateFootGlobal();
		thigh->globalMat = root->localMat * thigh->localMat;
		shin->globalMat = thigh->globalMat * shin->localMat;
		solveIK(getAnklePosFromHeel(glm::vec3(widthDisp, heelY, heelZ), foot->globalMat));
	
		startNewStep = true; //so we know when to execute condition in initial phase
	}
	else {
		startNewStep = true; //so we know when to execute condition in initial phase
	}
}

float Leg::solveIKStanceLegPelvisConfiguration(float time, glm::mat4 &thighGlobal, glm::mat4 &thighLocal, glm::mat4 &shinLocal, 
	float &testKneeRot, glm::mat4 &footLocal, glm::mat4 &rootLocal, std::vector<AngleDifferences> &vec)
{
	float testT = time + LOADING_RESPONSE * 0.5f;
	float testFootRot = 0.0f;
	static float lastT = 0.0;
	static glm::vec3 testToePosInit;
	static glm::vec3 testPrevAnklePos;
	//glm::vec3 testPrevToePos;
	float testHeelRiseAngle = 0.0f;
	//float error = 0.0f;

	glm::mat4 footGlobal = rootLocal * thighLocal * shinLocal * footLocal;

	if (testT < LOADING_RESPONSE) {
		//rotate to correct position to get correct heel position
		float phi = -(lerp(initialLoadingResponseRot, 0.0f, testT / LOADING_RESPONSE) - getFootAngleFromTerrain(footGlobal));
		footLocal = glm::rotate(footLocal, phi, glm::vec3(1.0f, 0.0f, 0.0f));
		footGlobal = rootLocal * thighLocal * shinLocal * footLocal;
		testFootRot += phi;
	}
	else if (testT >= MID_STANCE) {
		if (lastT < MID_STANCE) {
			testToePosInit = glm::vec3((footGlobal * toe->localMat)[3]);
			//testPrevAnklePos = glm::vec3(footGlobal[3]);
		}

		//testPrevToePos = testToePosInit + testPrevAnklePos - glm::vec3(footGlobal[3]);

		testHeelRiseAngle = heelRiseCurve.YfromX((testT - MID_STANCE) / (PRE_SWING - MID_STANCE));

		float phi = testHeelRiseAngle + getFootAngleFromTerrain(footGlobal);
		footLocal = glm::rotate(footLocal, phi, glm::vec3(1.0f, 0.0f, 0.0f));
		footGlobal = rootLocal * thighLocal * shinLocal * footLocal;
		testFootRot += phi;
	/*	if (testT >= 0.559f) {
			float dot = glm::dot(glm::normalize(glm::vec3(foot->localMat * glm::vec4(0.0, 1.0, 0.0, 1.0) - foot->localMat[3])),
				glm::normalize(glm::vec3(footLocal * glm::vec4(0.0, 1.0, 0.0, 1.0) - footLocal[3])));
			float diff = footRot - acos(dot > 1.0f ? 1.0f : dot);

			if (diff > 0.35f)
				return 1000000.0 * (diff - 0.35f);
		}*/
	}

	//////////////////

	glm::mat4 testReferenceGlobal = rootLocal * thighBindMat;
	glm::vec3 testReferenceVec = glm::normalize(glm::vec3(glm::rotate(glm::rotate(testReferenceGlobal, -(PI * 0.5f), glm::vec3(1.0, 0.0, 0.0)), thighWidth >= 0.0 ? toeOutAngle : -toeOutAngle, glm::vec3(0.0, 0.0, 1.0)) * glm::vec4(0.0, 1.0, 0.0, 1.0) - rootLocal[3]));

	glm::mat4 prevThighLocal = thighLocal;
	float prevTestKneeRot = testKneeRot;
	
	float dist = 0.0;
	if (testT < LOADING_RESPONSE) {
		glm::vec3 desiredPos = getAnklePosFromHeel(nextHeelPos, footGlobal);
		if ((dist = glm::length(desiredPos - glm::vec3(thighGlobal[3])) - (thigh->scale + shin->scale - 0.0001f)) > 0.0f)
			return dist;
		twoJointsIK(desiredPos, thighGlobal, thighLocal, thigh->scale, shinLocal, shin->scale,
			testKneeRot, testReferenceVec, rootLocal);
	}
	else if (testT >= MID_STANCE) {
		glm::vec3 desiredPos = testToePosInit - glm::vec3((footGlobal * toe->localMat)[3]) + glm::vec3(footGlobal[3]);
		if ((dist = glm::length(desiredPos - glm::vec3(thighGlobal[3])) - (thigh->scale + shin->scale - 0.0001f)) > 0.0f)
			return dist;
		twoJointsIK(desiredPos, thighGlobal, thighLocal, thigh->scale, shinLocal, shin->scale,
			testKneeRot, testReferenceVec, rootLocal);
	}
	else {
		if ((dist = glm::length(testPrevAnklePos - glm::vec3(thighGlobal[3])) - (thigh->scale + shin->scale - 0.0001f)) > 0.0f)
			return dist;
		twoJointsIK(testPrevAnklePos, thighGlobal, thighLocal, thigh->scale, shinLocal, shin->scale,
			testKneeRot, testReferenceVec, rootLocal);
	}


	footGlobal = rootLocal * thighLocal * shinLocal * footLocal;
	//rotate foot so that its alignes with the ground
	float actRot;
	if (testT < LOADING_RESPONSE) 
		actRot = -(lerp(initialLoadingResponseRot, 0.0f, testT / LOADING_RESPONSE) - getFootAngleFromTerrain(footGlobal));
	else if (testT >= MID_STANCE)
		actRot = testHeelRiseAngle + getFootAngleFromTerrain(footGlobal);
	else
		actRot = getFootAngleFromTerrain(footGlobal);
	footLocal = glm::rotate(footLocal, actRot, glm::vec3(1.0f, 0.0f, 0.0f));
	testFootRot += actRot;

	float dot = glm::dot(glm::normalize(glm::vec3(prevThighLocal * glm::vec4(0.0, 1.0, 0.0, 1.0) - prevThighLocal[3])),
		glm::normalize(glm::vec3(thighLocal * glm::vec4(0.0, 1.0, 0.0, 1.0) - thighLocal[3])));
	vec.push_back(AngleDifferences(acos(dot > 1.0f ? 1.0f : dot),
		prevTestKneeRot - testKneeRot,
		testFootRot));

	////////
	lastT = testT;
	testPrevAnklePos = glm::vec3(footGlobal[3]);

	return 0.0f;
}

bool Leg::pelvisConfigurationInitMidStance(glm::mat4 &thighGlobal, glm::mat4 &thighLocal, glm::mat4 &shinLocal,
	float &testKneeRot, glm::mat4 &footLocal, glm::mat4 &rootLocal, glm::mat4 &rootLocalHeelStrike)
{
	swingRotationCurve.setAddCoeff(2, 0.0f);
	swingRotationCurve.setAddCoeff(3, 0.0f);
	
	//try to move leg to heel strike position
	glm::mat4 testReferenceGlobal = rootLocalHeelStrike * thighBindMat;
	glm::vec3 testReferenceVec = glm::normalize(glm::vec3(glm::rotate(glm::rotate(testReferenceGlobal, -(PI * 0.5f), glm::vec3(1.0, 0.0, 0.0)), thighWidth >= 0.0 ? toeOutAngle : -toeOutAngle, glm::vec3(0.0, 0.0, 1.0)) * glm::vec4(0.0, 1.0, 0.0, 1.0) - rootLocalHeelStrike[3]));
	
	glm::mat4 footGlobal = rootLocalHeelStrike * thighLocal * shinLocal * footLocal;

	glm::vec3 testDesiredPos = getAnklePosFromHeel(nextHeelPos, footGlobal);
	if (glm::length(testDesiredPos - glm::vec3(thighGlobal[3])) >= (thigh->scale + shin->scale - 0.0001f)) {
		testDesiredPos = glm::vec3(thighGlobal[3]) + glm::normalize(testDesiredPos - glm::vec3(thighGlobal[3])) * (thigh->scale + shin->scale - 0.0001f);
	}

	twoJointsIK(testDesiredPos, thighGlobal, thighLocal, thigh->scale, shinLocal, shin->scale,
		testKneeRot, testReferenceVec, rootLocalHeelStrike);

	//move leg again to heel strike position with correct foot rotation
	thighGlobal = rootLocalHeelStrike * thighLocal;
	glm::mat4 testShinGlobal = thighGlobal * shinLocal;
	footLocal = glm::rotate(foot->localMat, footRot - swingRotControlPoints[1][3], glm::vec3(1.0f, 0.0f, 0.0f));
	footGlobal = testShinGlobal * footLocal;

	testDesiredPos = getAnklePosFromHeel(nextHeelPos, footGlobal);
	if (glm::length(testDesiredPos - glm::vec3(thighGlobal[3])) >= (thigh->scale + shin->scale - 0.0001f)) {
		testDesiredPos = glm::vec3(thighGlobal[3]) + glm::normalize(testDesiredPos - glm::vec3(thighGlobal[3])) * (thigh->scale + shin->scale - 0.0001f);
	}

	twoJointsIK(testDesiredPos, thighGlobal, thighLocal, thigh->scale, shinLocal, shin->scale,
		testKneeRot, testReferenceVec, rootLocalHeelStrike);

	//know we have leg at heel strike
	//now get it in the middle of loading response

	thighGlobal = rootLocalHeelStrike * thighLocal;
	footGlobal = rootLocalHeelStrike * thighLocal * shinLocal * footLocal;
	float dist = glm::length(getAnklePosFromHeel(nextHeelPos, footGlobal) - glm::vec3(thighGlobal[3])) - (thigh->scale + shin->scale - 0.0001f);
	//if (dist > 0.0f)
		//return false;

	initialLoadingResponseRot = getFootAngleFromTerrain(footGlobal);

	thighGlobal = rootLocal * thighLocal;
	testShinGlobal = thighGlobal * shinLocal;
	footGlobal = testShinGlobal * footLocal;
	float phi = -(lerp(initialLoadingResponseRot, 0.0f, 0.5f) - getFootAngleFromTerrain(footGlobal));
	footLocal = glm::rotate(footLocal, phi, glm::vec3(1.0f, 0.0f, 0.0f));
	footGlobal = testShinGlobal * footLocal;
	
	testDesiredPos = getAnklePosFromHeel(nextHeelPos, footGlobal);
	if (glm::length(testDesiredPos - glm::vec3(thighGlobal[3])) >= (thigh->scale + shin->scale - 0.0001f)) {
		testDesiredPos = glm::vec3(thighGlobal[3]) + glm::normalize(testDesiredPos - glm::vec3(thighGlobal[3])) * (thigh->scale + shin->scale - 0.0001f);
	}

	twoJointsIK(testDesiredPos, thighGlobal, thighLocal, thigh->scale, shinLocal, shin->scale,
		testKneeRot, testReferenceVec, rootLocal);

	thighGlobal = rootLocal * thighLocal;
	testShinGlobal = thighGlobal * shinLocal;
	footGlobal = testShinGlobal * footLocal;

	dist = glm::length(getAnklePosFromHeel(nextHeelPos, footGlobal) - glm::vec3(thighGlobal[3])) - (thigh->scale + shin->scale - 0.0001f);
	//if (glm::length(getAnklePosFromHeel(nextHeelPos, footGlobal) - glm::vec3(thighGlobal[3])) >= (thigh->scale + shin->scale - 0.0001f))
	//	return false;

	phi = -(lerp(initialLoadingResponseRot, 0.0f, 0.5f) - getFootAngleFromTerrain(footGlobal));
	footLocal = glm::rotate(footLocal, phi, glm::vec3(1.0f, 0.0f, 0.0f));

	return true;
}

void Leg::pelvisConfigurationInitMidSwing(glm::mat4 &thighGlobal, glm::mat4 &thighLocal, glm::mat4 &shinLocal,
	float &testKneeRot, glm::mat4 &footLocal, glm::mat4 &rootLocal)
{
	//try to move to position
	glm::mat4 testReferenceGlobal = rootLocal * thighBindMat;
	glm::vec3 testReferenceVec = glm::normalize(glm::vec3(glm::rotate(glm::rotate(testReferenceGlobal, -(PI * 0.5f), glm::vec3(1.0, 0.0, 0.0)), thighWidth >= 0.0 ? toeOutAngle : -toeOutAngle, glm::vec3(0.0, 0.0, 1.0)) * glm::vec4(0.0, 1.0, 0.0, 1.0) - rootLocal[3]));
	/*//glm::mat4 footGlobal = rootLocal * thighLocal * shinLocal * footLocal;
	glm::vec3 testDesiredPos = getAnklePosFromHeel(nextHeelPos, foot->globalMat); //we use rotation of foot because it is already on desired position
	if (glm::length(testDesiredPos - glm::vec3(thighGlobal[3])) >= (thigh->scale + shin->scale - 0.0001f)) {
		testDesiredPos = glm::vec3(thighGlobal[3]) + glm::normalize(testDesiredPos - glm::vec3(thighGlobal[3])) * (thigh->scale + shin->scale - 0.0001f);
	}
	twoJointsIK(testDesiredPos, thighGlobal, thighLocal, thigh->scale, shinLocal, shin->scale,
		testKneeRot, testReferenceVec, rootLocal);*/
	
	//rotate with the ground
	//footGlobal = rootLocal * thighLocal * shinLocal * footLocal;
	float actRot = getFootAngleFromTerrain(foot->globalMat); //we use rotation of foot because it is already on desired position
	footLocal = glm::rotate(footLocal, actRot, glm::vec3(1.0f, 0.0f, 0.0f));

	//try to move to position
	glm::mat4 footGlobal = shin->globalMat * footLocal; //use shin because we just want to get correct desired position
	glm::vec3 testDesiredPos = getAnklePosFromHeel(nextHeelPos, footGlobal);
	if (glm::length(testDesiredPos - glm::vec3(thighGlobal[3])) >= (thigh->scale + shin->scale - 0.0001f)) {
		testDesiredPos = glm::vec3(thighGlobal[3]) + glm::normalize(testDesiredPos - glm::vec3(thighGlobal[3])) * (thigh->scale + shin->scale - 0.0001f);
	}
	twoJointsIK(testDesiredPos, thighGlobal, thighLocal, thigh->scale, shinLocal, shin->scale,
		testKneeRot, testReferenceVec, rootLocal);


	//rotate with the ground
	thighGlobal = rootLocal * thighLocal;
	footGlobal = thighGlobal * shinLocal * footLocal;
	actRot = getFootAngleFromTerrain(footGlobal);
	footLocal = glm::rotate(footLocal, actRot, glm::vec3(1.0f, 0.0f, 0.0f));
	//move to position
	footGlobal = rootLocal * thighLocal * shinLocal * footLocal;
	testDesiredPos = getAnklePosFromHeel(nextHeelPos, footGlobal);
	if (glm::length(testDesiredPos - glm::vec3(thighGlobal[3])) >= (thigh->scale + shin->scale - 0.0001f)) {
		testDesiredPos = glm::vec3(thighGlobal[3]) + glm::normalize(testDesiredPos - glm::vec3(thighGlobal[3])) * (thigh->scale + shin->scale - 0.0001f);
	}
	twoJointsIK(testDesiredPos, thighGlobal, thighLocal, thigh->scale, shinLocal, shin->scale,
		testKneeRot, testReferenceVec, rootLocal);

	//fix rotation
	thighGlobal = rootLocal * thighLocal;
	footGlobal = thighGlobal * shinLocal * footLocal;
	actRot = getFootAngleFromTerrain(footGlobal);
	footLocal = glm::rotate(footLocal, actRot, glm::vec3(1.0f, 0.0f, 0.0f));
}

void Leg::configureSwingRotation()
{
	swingRotationCurve.setAddCoeff(2, 0.0f);
	swingRotationCurve.setAddCoeff(3, 0.0f);

	glm::vec3 testRootPos = *prevRootPos;
	testRootPos.z = prevRootPos->z + pelvisSpeedCurve(pelvisSpeed1Curve, pelvisSpeed2Curve, 1.0f - LOADING_RESPONSE);//pelvisSpeedCurve->YfromX(1.0f - LOADING_RESPONSE);
	testRootPos.y = pelvisVerticalCurve->YfromX(1.0f - LOADING_RESPONSE * 0.5f) + *maxPelvisHeight;
	glm::mat4 testRootLocal = *staticRootMat;
	testRootLocal[3] = glm::vec4(testRootPos, 1.0f);

	glm::mat4 testThighLocal = thigh->localMat;
	glm::mat4 testThighGlobal = testRootLocal * testThighLocal;
	glm::mat4 testShinLocal = shin->localMat;
	float testKneeRot = kneeRot;
	glm::mat4 testReferenceGlobal = testRootLocal * thighBindMat;
	glm::vec3 testReferenceVec = glm::normalize(glm::vec3(glm::rotate(glm::rotate(testReferenceGlobal, -(PI * 0.5f), glm::vec3(1.0, 0.0, 0.0)), thighWidth >= 0.0 ? toeOutAngle : -toeOutAngle, glm::vec3(0.0, 0.0, 1.0)) * glm::vec4(0.0, 1.0, 0.0, 1.0) - testRootLocal[3]));

	glm::vec3 testDesiredPos = getAnklePosFromHeel(nextHeelPos, foot->globalMat);
	if (glm::length(testDesiredPos - glm::vec3(testThighGlobal[3])) >= (thigh->scale + shin->scale - 0.0001f)) {
		testDesiredPos = glm::vec3(testThighGlobal[3]) + glm::normalize(testDesiredPos - glm::vec3(testThighGlobal[3])) * (thigh->scale + shin->scale - 0.0001f);
	}

	twoJointsIK(testDesiredPos, testThighGlobal, testThighLocal, thigh->scale, testShinLocal, shin->scale,
		testKneeRot, testReferenceVec, testRootLocal);

	testThighGlobal = testRootLocal * testThighLocal;
	glm::mat4 testShinGlobal = testThighGlobal * testShinLocal;
	glm::mat4 testFootGlobal = testShinGlobal * glm::rotate(foot->localMat, footRot - swingRotControlPoints[1][3], glm::vec3(1.0f, 0.0f, 0.0f));

	testDesiredPos = getAnklePosFromHeel(nextHeelPos, testFootGlobal);
	if (glm::length(testDesiredPos - glm::vec3(testThighGlobal[3])) >= (thigh->scale + shin->scale - 0.0001f)) {
		testDesiredPos = glm::vec3(testThighGlobal[3]) + glm::normalize(testDesiredPos - glm::vec3(testThighGlobal[3])) * (thigh->scale + shin->scale - 0.0001f);
	}

	twoJointsIK(testDesiredPos, testThighGlobal, testThighLocal, thigh->scale, testShinLocal, shin->scale,
		testKneeRot, testReferenceVec, testRootLocal);

	testFootGlobal = testRootLocal * testThighLocal * testShinLocal * glm::rotate(foot->localMat, footRot - swingRotControlPoints[1][3], glm::vec3(1.0f, 0.0f, 0.0f));
	float testAngle = getFootAngleFromTerrain(testFootGlobal);
	if (testAngle < 0.1f) {
		swingRotationCurve.setAddCoeff(2, -testAngle + 0.1f);
		swingRotationCurve.setAddCoeff(3, -testAngle + 0.1f);
	}
}

void Leg::configureSwing()
{
	swingCurve.setCoeffImmediately((getHeelPos(prevAnklePos, foot->globalMat).y - prevHeelPos.y) / swingControlPoints[1][0]);

	swingCurve.stepUp(nextHeelPos.y - prevHeelPos.y);
	swingCurve.setAddCoeffImmediately(prevHeelPos.y);

	swingRotationCurve.resetCoeff();
	swingRotationCurve.setCoeffImmediately(footRot / swingRotControlPoints[1][0]); //15 degrees is start of bezier curve

	configureSwingRotation();

	//set up some values
	glm::mat4 testThighLocal = thigh->localMat;
	glm::mat4 testShinLocal = shin->localMat;
	float testKneeRot = kneeRot;
	glm::mat4 testFootLocal = glm::rotate(foot->localMat, footRot, glm::vec3(1.0, 0.0, 0.0)); //local mat rotated at 0 degrees

	for (float i = 0.02f; i <= 0.95f; i += 0.02f) {
		float pelvisT = 2.0f * ((TERMINAL_SWING - PRE_SWING) * i + PRE_SWING - (0.5f * LOADING_RESPONSE + 0.5f));
		float swingT = ((TERMINAL_SWING - PRE_SWING) * i) / (TERMINAL_SWING - PRE_SWING);
		
		//set pelvis to new position
		glm::mat4 testRootLocal = *staticRootMat;
		testRootLocal[3].z = prevRootPos->z + pelvisSpeedCurve(pelvisSpeed1Curve, pelvisSpeed2Curve, pelvisT);//pelvisSpeedCurve->YfromX(pelvisT);//glm::vec4(testRootPos, 1.0f);
		testRootLocal[3].y = pelvisVerticalCurve->YfromX(pelvisT) + *maxPelvisHeight;
		
		//
		//glm::mat4 testThighLocal = thigh->localMat;
		glm::mat4 testThighGlobal = testRootLocal * testThighLocal;
		glm::mat4 testShinGlobal = testThighGlobal * testShinLocal;
		glm::mat4 testReferenceGlobal = testRootLocal * thighBindMat;
		glm::vec3 testReferenceVec = glm::normalize(glm::vec3(glm::rotate(glm::rotate(testReferenceGlobal, -(PI * 0.5f), glm::vec3(1.0, 0.0, 0.0)), thighWidth >= 0.0 ? toeOutAngle : -toeOutAngle, glm::vec3(0.0, 0.0, 1.0)) * glm::vec4(0.0, 1.0, 0.0, 1.0) - testRootLocal[3]));

		//check if position is in reach
 		float widthDisp = testRootLocal[3].x; //displacement to width of swing phase so foot is under hip and not on the side
		if (swingT <= 0.5) {
			widthDisp = lerp(toeOffPos.x, testRootLocal[3].x + thighWidth, swingWidthFixCurve.YfromX(swingT));
		}
		if (swingT >= 0.5) {
			widthDisp = lerp(footWidth, testRootLocal[3].x + thighWidth, swingWidthFixCurve.YfromX(swingT));
		}
		float heelY = swingCurve.YfromX(swingT);
		float heelZ = toeOffPos.z + swingForwardCurve.YfromX(swingT);

		float testFootRot = swingRotationCurve.YfromX(swingT);
		glm::mat4 testFootGlobal = testShinGlobal * glm::rotate(testFootLocal, -testFootRot, glm::vec3(1.0, 0.0, 0.0));

		glm::vec3 testDesiredPos = getAnklePosFromHeel(glm::vec3(widthDisp, heelY, heelZ), testFootGlobal);

		int maxIter = 0;
		while (1) {
			float diff;
			if ((diff = glm::length(testDesiredPos - glm::vec3(testThighGlobal[3])) - (thigh->scale + shin->scale - 0.0001f)) > 0.0f) {
				swingCurve.increase(swingT, diff);
			}
			else
				break;

			heelY = swingCurve.YfromX(swingT);
			testDesiredPos = getAnklePosFromHeel(glm::vec3(widthDisp, heelY, heelZ), testFootGlobal);

			maxIter++;
			if (maxIter > 10)
				break;
		}

		twoJointsIK(testDesiredPos, testThighGlobal, testThighLocal, thigh->scale, testShinLocal, shin->scale,
			testKneeRot, testReferenceVec, testRootLocal);

		testThighGlobal = testRootLocal * testThighLocal;
		testShinGlobal = testThighGlobal * testShinLocal;
		testFootGlobal = testShinGlobal * glm::rotate(testFootLocal, -testFootRot, glm::vec3(1.0, 0.0, 0.0));
		glm::vec3 testToePos = getToePos(testFootGlobal);
		glm::vec3 testHeelPos = getHeelPos(glm::vec3(testFootGlobal[3]), testFootGlobal);
		float diff = 0.0f, testDiff = 0.0f;
		if ((testDiff = testHeelPos.y - terrain->getHeight(testHeelPos)) < 0.0f)
			diff = testDiff;
		if ((testDiff = testToePos.y - terrain->getHeight(testToePos)) < 0.0f) {
			if (testDiff < diff)
				diff = testDiff;
		}
		if (diff < 0.0f)
			swingCurve.increase(swingT, -diff);
	}

	swingCurve.controlPointUp(1, footUpCoeff);
	swingCurve.controlPointUp(2, footUpCoeff);
}

glm::vec3 Leg::getToePos(glm::mat4 &footGlobal)
{
	return glm::vec3(footGlobal * glm::vec4(toePos, 1.0));
}

glm::vec3 Leg::getHeelPos(glm::vec3 pos, glm::mat4 &footGlobal)
{
	return glm::vec3(footGlobal * glm::vec4(heelPos, 1.0)) + pos - glm::vec3(footGlobal[3]);
}

glm::vec3 Leg::getAnklePosFromHeel(glm::vec3 pos, glm::mat4 &footGlobal)
{
	glm::vec3 actPos = getHeelPos(glm::vec3(footGlobal[3]), footGlobal);
	return glm::vec3(footGlobal[3]) + pos - actPos;
}

void Leg::riseHeel(float angle)
{
	//rotate foot to the correct angle with the ground
	float phi = angle + getFootAngleFromTerrain(foot->globalMat);
	foot->localMat = glm::rotate(foot->localMat, phi, glm::vec3(1.0f, 0.0f, 0.0f));
	footRot -= phi;
	updateFootGlobal();
	updateToeGlobal();

	//move foot so fingers fit their previous position
	solveIK(toePosInit - glm::vec3(toe->globalMat[3]) + glm::vec3(foot->globalMat[3]));
	updateFootGlobal();
	updateToeGlobal();

	//rotate foot again because IK ruined our rotation with the ground
	phi = angle + getFootAngleFromTerrain(foot->globalMat);
	foot->localMat = glm::rotate(foot->localMat, phi, glm::vec3(1.0f, 0.0f, 0.0f));
	footRot -= phi;
	updateFootGlobal();
	updateToeGlobal();
	
	//rotate fingers so that overal rotation is the same as angle between ground and foot
	phi = -getFootAngleFromTerrain(foot->globalMat) - toeRot;
	toe->localMat = glm::rotate(toe->localMat, -phi, glm::vec3(1.0f, 0.0f, 0.0f));
	toeRot += phi;
}

void Leg::updateFootGlobal()
{
	foot->globalMat = root->localMat * thigh->localMat * shin->localMat * foot->localMat;
}

void Leg::updateToeGlobal()
{
	toe->globalMat = root->localMat * thigh->localMat * shin->localMat * foot->localMat * toe->localMat;
}

glm::vec3 twoJointsIK(glm::vec3 desiredPos, glm::mat4 &sphericalJointGlobal, glm::mat4 &sphericalJointLocal, float sphericalJointScale,
	glm::mat4 &hingeJointLocal, float hingeJointScale, float &hingeRot, glm::vec3 referenceVec, glm::mat4 &rootGlobal)
{
	//dont let it get out of reach
	if (glm::length(desiredPos - glm::vec3(sphericalJointGlobal[3])) >= (sphericalJointScale + hingeJointScale - 0.0001f)) {
		desiredPos = glm::vec3(sphericalJointGlobal[3]) + glm::normalize(desiredPos - glm::vec3(sphericalJointGlobal[3])) * (sphericalJointScale + hingeJointScale - 0.0001f);
	}

	//transformation of spherical joint so it points at desired position
	glm::vec3 direction(glm::normalize(glm::vec3(glm::inverse(sphericalJointGlobal) * glm::vec4(desiredPos, 1.0))));
	glm::vec3 up = glm::vec3(0.0, 0.0, -1.0);
	glm::vec3 right = glm::normalize(glm::cross(up, direction));
	up = glm::normalize(glm::cross(right, direction));
	glm::mat4 mat(right.x, right.y, right.z, 0.0f,
		direction.x, direction.y, direction.z, 0.0f,
		up.x, up.y, up.z, 0.0f,
		0.0, 0.0, 0.0, 1.0f);

	sphericalJointLocal = sphericalJointLocal * mat;

	//angles in triangle made of joints and line between spherical joint and desired pos
	float L = glm::length(glm::vec3(sphericalJointGlobal[3] - glm::vec4(desiredPos, 1.0)));
	float phi_1 = acos((pow(sphericalJointScale, 2) + L*L - pow(hingeJointScale, 2)) / (2 * sphericalJointScale*L));
	float phi_2 = PI - acos((pow(sphericalJointScale, 2) + pow(hingeJointScale, 2) - L*L) / (2 * sphericalJointScale*hingeJointScale));

	//transformation of joints
	sphericalJointLocal = glm::rotate(sphericalJointLocal, -phi_1, glm::vec3(1.0, 0.0, 0.0));
	hingeJointLocal = glm::rotate(hingeJointLocal, phi_2 - hingeRot, glm::vec3(1.0, 0.0, 0.0));
	hingeRot = phi_2;

	//count angle delta to rotate around vector from spherical joint and desired pos, so thigh is as close as possible to reference position
	//reference position is thigh in 2D (rotated only around x) rotated by the same angle plus a bit more to keep leg point forward
	glm::vec3 a = glm::normalize(glm::vec3(glm::vec4(desiredPos, 1.0) - sphericalJointGlobal[3]));
	glm::vec3 v = glm::normalize(glm::vec3(rootGlobal * sphericalJointLocal * glm::vec4(0.0, 1.0, 0.0, 1.0) - sphericalJointGlobal[3]));
	glm::vec3 k = referenceVec;
	float A = glm::dot(glm::dot(a, v) * a, k) - glm::dot(glm::cross(glm::cross(a, v), a), k);
	float B = glm::dot(glm::cross(a, v), k);
	float C = glm::dot(v, k);

	float delta = atan2(2 * B, C - A);
	if ((((A - C) / 2.0) * cos(delta) - B * sin(delta)) > 0.0)
		delta += PI;

	//rotate by delta
	sphericalJointLocal = glm::rotate(sphericalJointLocal, phi_1, glm::vec3(1.0, 0.0, 0.0));
	sphericalJointLocal = glm::rotate(sphericalJointLocal, delta, glm::vec3(0.0, 1.0, 0.0));
	sphericalJointLocal = glm::rotate(sphericalJointLocal, -phi_1, glm::vec3(1.0, 0.0, 0.0));

	return desiredPos;
}

void Leg::solveIK(glm::vec3 desiredPos)
{
	prevAnklePos = desiredPos;

	if (desiredPos.y > (*staticRootMat)[3].y)
		return;

	glm::mat4 referenceGlobal = *staticRootMat * thighBindMat;
	glm::vec3 referenceVec = glm::normalize(glm::vec3(glm::rotate(glm::rotate(referenceGlobal, -(PI * 0.5f), glm::vec3(1.0, 0.0, 0.0)), thighWidth >= 0.0 ? toeOutAngle : -toeOutAngle, glm::vec3(0.0, 0.0, 1.0)) * glm::vec4(0.0, 1.0, 0.0, 1.0) - (*staticRootMat)[3]));

	prevAnklePos = twoJointsIK(desiredPos, thigh->globalMat, thigh->localMat, thigh->scale, shin->localMat,
		shin->scale, kneeRot, referenceVec, root->globalMat);

	if (prevAnklePos != desiredPos) {
		std::cout << "IK: Could not reach desired position! t: " << t << std::endl;
		IKFalse = true;
	}
}

bool Leg::getIKFalse()
{
	bool temp = IKFalse;
	IKFalse = false;

	return temp;
}

float Leg::getLegLength()
{
	return thighLength + shinLength;
}

void Leg::getHeelSwingCurvePoints(std::vector<float> &vec)
{
	const BezierCurve *curve = &swingCurve;
	for (unsigned int i = 0; i < vec.size(); i++) {
		vec[i] = curve->YfromX(i * (1.0f / (vec.size() - 1)));
	}
}
