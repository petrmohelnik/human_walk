#include "Leg.h"

float lerp(float start, float end, float t)
{
	return start + (end - start) * t;
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
	heelRiseCurve.setCoeffImmediately(1.0);
}

void Leg::init(int init, glm::vec3 center, std::shared_ptr<const BezierCurve> pVert, std::shared_ptr<const BezierCurve> pSpeed,
	std::shared_ptr<const float> maxPelvisH, std::shared_ptr<const glm::vec3> _prevRootPos,
	std::shared_ptr<const glm::vec3> _nextRootPos, std::shared_ptr<const glm::mat4> staticRootM)
{
	pelvisVerticalCurve = pVert;
	pelvisSpeedCurve = pSpeed;
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
		prevToePos = toePosInit + prevAnklePos - glm::vec3(foot->globalMat[3]); //save toe pos so we know where to put foot while raising heel later

		thigh->globalMat = root->localMat * thigh->localMat;
		shin->globalMat = thigh->globalMat * shin->localMat;
		//riseHeel(baseVec, TERMINAL_STANCE_HEEL_RISE_ANGLE, TERMINAL_STANCE_HEEL_RISE_ANGLE, 1.0);
		float preSwingT = (TERMINAL_STANCE - MID_STANCE) / (PRE_SWING - MID_STANCE);
		riseHeel(heelRiseCurve.YfromX(preSwingT));

		updateFootGlobal();
		updateToeGlobal();
		toePosInit = glm::vec3(toe->globalMat[3]);
		prevToePos = toePosInit + prevAnklePos - glm::vec3(foot->globalMat[3]);
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

	nextHeelPos.z += step;
	float height = terrain->getHeight(nextHeelPos);
	nextHeelPos.y = height;

	swingCurve.setSwingFootHeight(prevHeelPos.y);
	if (height > prevHeelPos.y)
		swingCurve.stepUp(height - prevHeelPos.y);
	else
		swingCurve.stepDown(prevHeelPos.y - height);

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
			initialLoadingResponseRot = getFootAngleFromTerrain(foot->globalMat);
		
//			prevHeelPos = glm::vec3(width, bezierCurve(swingControlPointsVec, 1.0), lerp(toeOffPos, toeOffPos + 1.6, 1.0)); //end of swing curve
			//updateFootGlobal();
			//prevHeelPos = getHeelPos(prevAnklePos);
			prevHeelPos = nextHeelPos;
			//prevHeelPos.x = footWidth;
			//nextHeelPos = prevHeelPos;
			//nextHeelPos.x = footWidth;
			//nextHeelPos.z += stepLength;
			//.y += 0.05;
			//swingCurve.incrementAddCoeff(0.05);
			//rootStaticMat[3].z += stepLength; //MUST MOVE THIS MATRIX WITH BODY BUT ONLY IN DIRECTION, NOT SMALL ROTATIONS AND DISPLACEMENTS CAUSED BY MOVEMENT
			//toePosInit.z += stepLength;

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
		solveIK(prevAnklePos);
		updateFootGlobal();
		updateToeGlobal();

		//rotate foot so that its alignes with the ground
		float actRot = getFootAngleFromTerrain(foot->globalMat);
		foot->localMat = glm::rotate(foot->localMat, actRot, glm::vec3(1.0f, 0.0f, 0.0f));
		footRot -= actRot;

		//updateToeGlobal();
		//prevToePos = toePosInit + prevAnklePos - glm::vec3(foot->globalMat[3]); //save toe pos so we know where to put foot while raising heel later
	}
	else if (t < PRE_SWING) {
		if (t - dt < MID_STANCE) {
			toeOffPos = getHeelPos(prevAnklePos, foot->globalMat); //position from which we start swing phase
			toePosInit = glm::vec3(toe->globalMat[3]);
		}

		float preSwingT = (t - MID_STANCE) / (PRE_SWING - MID_STANCE);  //(t - TERMINAL_STANCE) / (PRE_SWING - TERMINAL_STANCE);

		prevToePos = toePosInit + prevAnklePos - glm::vec3(foot->globalMat[3]);
		riseHeel(heelRiseCurve.YfromX(preSwingT));

		//updateToeGlobal();
	}
	//pocatecni svih, mezisvih a konecny svih
	else if (t < TERMINAL_SWING) {
		if (t - dt < PRE_SWING) {
			//nextHeelPos.y += 0.2;
			//swingCurve.incrementAddCoeff(0.2);

			//finish heel rise
			prevToePos = toePosInit + prevAnklePos - glm::vec3(foot->globalMat[3]);
			riseHeel(heelRiseCurve.YfromX(1.0f));

			//toeOffPos = (toeOffPos == -1.0 ? -0.4 : getHeelPos(prevAnklePos).z); //position from which we start swing phase
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
			/*float angle = toeRot / (((PRE_SWING + ((TERMINAL_SWING - PRE_SWING) * 0.2f)) - t) / dt);
			if (angle != angle)
				angle = 0.0;
			toe->localMat = glm::rotate(toe->localMat, -angle, glm::vec3(1.0f, 0.0f, 0.0f));
			toeRot -= angle;*/
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

		/*float widthFix = widthDisp; //after heel raise the heel is not in position he is expected, so we have to slowly get him there
		if (swingT <= 0.2) {
			widthFix = lerp(widthFix, getHeelPos(prevAnklePos).x, pow(1.0 - (5.0 * swingT), 2));
		}*/

		//rotate foot while being mid air
		float angle = swingRotationCurve.YfromX(swingT) - footRot;//bezierCurve(swingRotControlPointsVec, swingT) - footRot;
		foot->localMat = glm::rotate(foot->localMat, -angle, glm::vec3(1.0f, 0.0f, 0.0f));
		footRot += angle;

		//set new heel position
		updateFootGlobal();
		float heelY = swingCurve.YfromXSwingFoot(swingT);
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

void Leg::configureSwing()
{
	swingCurve.setCoeffImmediately((getHeelPos(prevAnklePos, foot->globalMat).y - prevHeelPos.y) / swingControlPoints[1][0]);

	swingRotationCurve.resetCoeff();
	swingRotationCurve.setCoeffImmediately(footRot / swingRotControlPoints[1][0]); //15 degrees is start of bezier curve

	glm::vec3 testRootPos = *prevRootPos;
	testRootPos.z = prevRootPos->z + pelvisSpeedCurve->YfromX(1.0f - LOADING_RESPONSE);
	testRootPos.y = pelvisVerticalCurve->YfromX(1.0f - LOADING_RESPONSE * 0.5f) + *maxPelvisHeight;
	glm::mat4 testRootLocal = *staticRootMat;
	testRootLocal[3] = glm::vec4(testRootPos, 1.0f);
	
	glm::mat4 testThighLocal = thigh->localMat;
	glm::mat4 testThighGlobal = testRootLocal * testThighLocal;
	glm::mat4 testShinLocal = shin->localMat;
	float testKneeRot = kneeRot;
	glm::mat4 testReferenceGlobal = testRootLocal * thighBindMat;
	glm::vec3 testReferenceVec = glm::normalize(glm::vec3(glm::rotate(glm::rotate(testReferenceGlobal, -(PI * 0.5f), glm::vec3(1.0, 0.0, 0.0)), thighWidth >= 0.0 ? 0.1f : -0.1f, glm::vec3(0.0, 0.0, 1.0)) * glm::vec4(0.0, 1.0, 0.0, 1.0) - testRootLocal[3]));

	/*this shouldnt be neccessary later*/
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
	if (testAngle < 0.0f)
		swingRotationCurve.setAddCoeff(-testAngle);
	//swingRotationCurve.setAddCoeff(0.5f - testAngle);
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
	solveIK(prevToePos - glm::vec3(toe->globalMat[3]) + glm::vec3(foot->globalMat[3]));
	updateFootGlobal();
	updateToeGlobal();

	//rotate foot again because IK ruined our rotation with the ground
	phi = angle + getFootAngleFromTerrain(foot->globalMat);
	foot->localMat = glm::rotate(foot->localMat, phi, glm::vec3(1.0f, 0.0f, 0.0f));
	footRot -= phi;
	updateFootGlobal();
	updateToeGlobal();

	/*int i = 0;
	while (abs(glm::length(prevToePos - glm::vec3(toe->globalMat[3]))) > 0.001f && i < 10) {
		solveIK(prevToePos - glm::vec3(toe->globalMat[3]) + glm::vec3(foot->globalMat[3]));
		thigh->globalMat = root->localMat * thigh->localMat;
		shin->globalMat = thigh->globalMat * shin->localMat;
		updateFootGlobal();
		updateToeGlobal();
		i++;
	}
	std::cout << "i: " << i << std::endl;

	glm::vec3 footVec = glm::normalize(getToePos() - getHeelPos(glm::vec3(foot->globalMat[3])));
	baseVec = glm::normalize(glm::vec3(footVec.x, 0.0, footVec.z));
	float phi = lerp(startAngle, endAngle, t) - getAngle(baseVec, footVec, foot->globalMat, glm::vec3(1.0, 0.0, 0.0));
	if (phi > 0.0) {
		foot->localMat = glm::rotate(foot->localMat, phi, glm::vec3(1.0f, 0.0f, 0.0f));
		footRot -= phi;
		updateFootGlobal();
		updateToeGlobal();

		//move foot so fingers fit their previous position
		solveIK(prevToePos - glm::vec3(toe->globalMat[3]) + glm::vec3(foot->globalMat[3]));
		updateFootGlobal();
		updateToeGlobal();

		//rotate foot again because IK ruined our rotation with the ground
		footVec = glm::normalize(getToePos() - getHeelPos(glm::vec3(foot->globalMat[3])));
		baseVec = glm::normalize(glm::vec3(footVec.x, 0.0, footVec.z));
		phi = lerp(startAngle, endAngle, t) - getAngle(baseVec, footVec, foot->globalMat, glm::vec3(1.0, 0.0, 0.0));
		foot->localMat = glm::rotate(foot->localMat, phi, glm::vec3(1.0f, 0.0f, 0.0f));
		footRot -= phi;
		updateFootGlobal();
		updateToeGlobal();
	}*/
	
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
	float y = sphericalJointGlobal[3].y - desiredPos.y, z = desiredPos.z - sphericalJointGlobal[3].z;
	float L = glm::length(glm::vec3(sphericalJointGlobal[3] - glm::vec4(desiredPos, 1.0)));
	float phi_t = atan(z / y);
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
	if (sphericalJointLocal != sphericalJointLocal)
		sphericalJointLocal = sphericalJointLocal;

	return desiredPos;
}

void Leg::solveIK(glm::vec3 desiredPos)
{
	prevAnklePos = desiredPos;

	glm::mat4 referenceGlobal = *staticRootMat * thighBindMat;
	glm::vec3 referenceVec = glm::normalize(glm::vec3(glm::rotate(glm::rotate(referenceGlobal, -(PI * 0.5f), glm::vec3(1.0, 0.0, 0.0)), thighWidth >= 0.0 ? 0.1f : -0.1f, glm::vec3(0.0, 0.0, 1.0)) * glm::vec4(0.0, 1.0, 0.0, 1.0) - (*staticRootMat)[3]));

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
