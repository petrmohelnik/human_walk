#include "Leg.h"

glm::vec3 bezierCurve(std::vector<glm::vec3> &P, float t)
{
	glm::vec3 result;

	result.x = P[0].x*pow((1 - t), 3) + P[1].x * 3 * t*pow((1 - t), 2) +
		P[2].x * 3 * t*t*(1 - t) + P[3].x*pow(t, 3);
	result.y = P[0].y*pow((1 - t), 3) + P[1].y * 3 * t*pow((1 - t), 2) +
		P[2].y * 3 * t*t*(1 - t) + P[3].y*pow(t, 3);
	result.z = P[0].z*pow((1 - t), 3) + P[1].z * 3 * t*pow((1 - t), 2) +
		P[2].z * 3 * t*t*(1 - t) + P[3].z*pow(t, 3);

	return result;
}

float bezierCurve(std::vector<float> &P, float t)
{
	float result;

	result = P[0]*pow((1 - t), 3) + P[1] * 3 * t*pow((1 - t), 2) +
		P[2] * 3 * t*t*(1 - t) + P[3]*pow(t, 3);

	return result;
}


float lerp(float start, float end, float t)
{
	return start + (end - start) * t;
}

Leg::Leg(Bone *hips, Bone *sphericalJoint, Bone *hingeJoint, Bone *endEffector, Bone *fingers, float length)
{
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
	float L = glm::length(glm::vec3(thigh->globalMat[3] - foot->globalMat[3]));
	float phi = PI - acos((thighLength*thighLength + shinLength*shinLength - L*L) / (2 * thighLength*shinLength));
	kneeRot = phi;
	
	width = thigh->globalMat[3].x;
	glm::mat4 newThigh(1.0);
	newThigh = glm::rotate(newThigh, PI, glm::vec3(1.0, 0.0, 0.0));
	newThigh[3] = thigh->localMat[3];
	thighBindMat = newThigh;// thigh->localMat;
	heelPos = glm::vec3(glm::inverse(foot->globalMat) * glm::vec4(foot->globalMat[3].x, 0.0, 0.0, 1.0));
	toePos = glm::vec3(glm::inverse(foot->globalMat) * glm::vec4(toe->globalMat[3].x, 0.0, toe->globalMat[3].z, 1.0));
	if (t < PRE_SWING)
		toeOffPos = 0.0;
	prevAnklePos = glm::vec3(foot->globalMat[3]);

	glm::mat4 newShin(1.0);
	newShin = glm::rotate(glm::translate(newShin, glm::vec3(shin->localMat[3])), phi, glm::vec3(1.0, 0.0, 0.0));
	shin->localMat = newShin;

	swingControlPointsVec.resize(4);
	swingRotControlPointsVec.resize(4);
}

void Leg::init(int init, glm::vec3 center)
{

	if (init == INIT_HEEL_STRIKE) {
		prevHeelPos = glm::vec3(center.x + width, 0.0f, center.z + 0.375f);
		//rotate foot while being mid air
		float angle = -0.03;
		foot->localMat = glm::rotate(foot->localMat, -angle, glm::vec3(1.0f, 0.0f, 0.0f));
		footRot += angle;

		//set new heel position
		thigh->globalMat = root->localMat * thigh->localMat;
		shin->globalMat = thigh->globalMat * shin->localMat;
		updateFootGlobal();
		solveIK(getAnklePosFromHeel(prevHeelPos));

		//updateFootGlobal();
		//prevHeelPos = getHeelPos(prevAnklePos);

		//compute position again using new rotation so heel fits more accurately
		updateFootGlobal();
		thigh->globalMat = root->localMat * thigh->localMat;
		shin->globalMat = thigh->globalMat * shin->localMat;
		solveIK(getAnklePosFromHeel(prevHeelPos));

		//updateFootGlobal();
		//prevHeelPos = getHeelPos(prevAnklePos);
		//prevHeelPos.z -= stepLength;

		t = 0.0;
	}
	else if (init == INIT_TERMINAL_STANCE) {
		prevHeelPos = glm::vec3(center.x + width, 0.0f, center.z - 0.625f);
		solveIK(getAnklePosFromHeel(prevHeelPos));
		updateFootGlobal();
		updateToeGlobal();

		//rotate foot so that it alignes with the ground
		glm::vec3 footVec = glm::normalize(getToePos() - getHeelPos(glm::vec3(foot->globalMat[3])));
		glm::vec3 baseVec = glm::normalize(glm::vec3(footVec.x, 0.0, footVec.z));
		float actRot = getAngle(footVec, baseVec, foot->globalMat, glm::vec3(1.0, 0.0, 0.0));
		foot->localMat = glm::rotate(foot->localMat, actRot, glm::vec3(1.0f, 0.0f, 0.0f));
		footRot -= actRot;

		//updateFootGlobal();
		//toeOffPos = getHeelPos(prevAnklePos).z; //position from which we start swing phase
		//updateFootGlobal();
		//prevHeelPos = glm::vec3(width, 0.0f, -0.35f);// getHeelPos(prevAnklePos);

		updateFootGlobal();
		updateToeGlobal();
		prevToePos = glm::vec3(toe->globalMat[3]) + prevAnklePos - glm::vec3(foot->globalMat[3]); //save toe pos so we know where to put foot while raising heel later

		footVec = glm::normalize(getToePos() - getHeelPos(glm::vec3(foot->globalMat[3])));
		baseVec = glm::normalize(glm::vec3(footVec.x, 0.0, footVec.z));
		thigh->globalMat = root->localMat * thigh->localMat;
		shin->globalMat = thigh->globalMat * shin->localMat;
		raiseHeel(baseVec, TERMINAL_STANCE_HEEL_RISE_ANGLE, TERMINAL_STANCE_HEEL_RISE_ANGLE, 1.0);

		updateFootGlobal();
		updateToeGlobal();
		prevToePos = glm::vec3(toe->globalMat[3]) + prevAnklePos - glm::vec3(foot->globalMat[3]);
		t = 0.5;
	}
}

float Leg::getAngle(const glm::vec3 &vec1, const glm::vec3 &vec2, const glm::mat4 &ref, const glm::vec3 &refDir)
{
	glm::vec3 refV = glm::normalize(glm::vec3(ref * glm::vec4(refDir, 1.0f)) - glm::vec3(ref[3]));
	float dot = glm::dot(vec1, vec2);
	float angle = acos(dot <= 1.0f ? dot : 1.0f); //we dont want undefined value when float is not accurate enough
	return glm::dot(glm::cross(vec1, vec2), refV) < 0.0 ? -angle : angle;
}

void Leg::update(float dt)
{
	//dt *= 0.1;
	t += dt;

	if (t > TERMINAL_SWING)
		t -= TERMINAL_SWING;

	//faze zatezovani
	if (t < LOADING_RESPONSE) {
		if (startNewStep == true) {
			glm::vec3 footVec = glm::normalize(getToePos() - getHeelPos(glm::vec3(foot->globalMat[3])));
			glm::vec3 baseVec = glm::normalize(glm::vec3(footVec.x, 0.0, footVec.z));
			initialLoadingResponseRot = getAngle(footVec, baseVec, foot->globalMat, glm::vec3(1.0, 0.0, 0.0));
		
//			prevHeelPos = glm::vec3(width, bezierCurve(swingControlPointsVec, 1.0), lerp(toeOffPos, toeOffPos + 1.6, 1.0)); //end of swing curve
			//updateFootGlobal();
			//prevHeelPos = getHeelPos(prevAnklePos);
			prevHeelPos.z += stepLength;

			std::cout << prevHeelPos.x << " " << prevHeelPos.y << " " << prevHeelPos.z << std::endl;
		}
		//rotate to correct position to get correct heel position
		glm::vec3 footVec = glm::normalize(getToePos() - getHeelPos(glm::vec3(foot->globalMat[3])));
		glm::vec3 baseVec = glm::normalize(glm::vec3(footVec.x, 0.0, footVec.z));
		float phi = lerp(initialLoadingResponseRot, 0.0f, t / LOADING_RESPONSE) - getAngle(footVec, baseVec, foot->globalMat, glm::vec3(1.0, 0.0, 0.0));
		foot->localMat = glm::rotate(foot->localMat, -phi, glm::vec3(1.0f, 0.0f, 0.0f));
		footRot += phi;
		updateFootGlobal();

		//move to previous heel position
		solveIK(getAnklePosFromHeel(prevHeelPos));
		updateFootGlobal();
		updateToeGlobal();

		//rotate again because moving ruined our rotation
		footVec = glm::normalize(getToePos() - getHeelPos(glm::vec3(foot->globalMat[3])));
		baseVec = glm::normalize(glm::vec3(footVec.x, 0.0, footVec.z));
		phi = lerp(initialLoadingResponseRot, 0.0f, t / LOADING_RESPONSE) - getAngle(footVec, baseVec, foot->globalMat, glm::vec3(1.0, 0.0, 0.0));
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
		glm::vec3 footVec = glm::normalize(getToePos() - getHeelPos(glm::vec3(foot->globalMat[3])));
		glm::vec3 baseVec = glm::normalize(glm::vec3(footVec.x, 0.0, footVec.z));
		float actRot = getAngle(footVec, baseVec, foot->globalMat, glm::vec3(1.0, 0.0, 0.0));
		foot->localMat = glm::rotate(foot->localMat, actRot, glm::vec3(1.0f, 0.0f, 0.0f));
		footRot -= actRot;

		updateToeGlobal();
		prevToePos = glm::vec3(toe->globalMat[3]) + prevAnklePos - glm::vec3(foot->globalMat[3]); //save toe pos so we know where to put foot while raising heel later
	}
	//konecny stoj
	else if (t < TERMINAL_STANCE) {
		/*if (t - dt < MID_STANCE) {
			toeOffPos = getHeelPos(prevAnklePos).z; //position from which we start swing phase
		}*/

		float terminalStanceT = (t - MID_STANCE) / (TERMINAL_STANCE - MID_STANCE);

		glm::vec3 footVec = glm::normalize(getToePos() - getHeelPos(glm::vec3(foot->globalMat[3])));
		glm::vec3 baseVec = glm::normalize(glm::vec3(footVec.x, 0.0, footVec.z));
		raiseHeel(baseVec, 0.0, TERMINAL_STANCE_HEEL_RISE_ANGLE, terminalStanceT);

		updateToeGlobal();
		prevToePos = glm::vec3(toe->globalMat[3]) + prevAnklePos - glm::vec3(foot->globalMat[3]);
	}
	//predsvih
	else if (t < PRE_SWING) {
		float preSwingT = (t - TERMINAL_STANCE) / (PRE_SWING - TERMINAL_STANCE);

		glm::vec3 footVec = glm::normalize(getToePos() - getHeelPos(glm::vec3(foot->globalMat[3])));
		glm::vec3 baseVec = glm::normalize(glm::vec3(footVec.x, 0.0, footVec.z));
		raiseHeel(baseVec, TERMINAL_STANCE_HEEL_RISE_ANGLE, TERMINAL_STANCE_HEEL_RISE_ANGLE + PRE_SWING_HEEL_RISE_ANGLE, preSwingT);

		updateToeGlobal();
		prevToePos = glm::vec3(toe->globalMat[3]) + prevAnklePos - glm::vec3(foot->globalMat[3]);
	}
	//pocatecni svih, mezisvih a konecny svih
	else if (t < TERMINAL_SWING) {
		if (t - dt < PRE_SWING) {
			//toeOffPos = (toeOffPos == -1.0 ? -0.4 : getHeelPos(prevAnklePos).z); //position from which we start swing phase
			swingDist = stepLength - glm::length(glm::vec3(getHeelPos(prevAnklePos).x, 0.0, getHeelPos(prevAnklePos).z) - prevHeelPos);
			toeOffPos = getHeelPos(prevAnklePos).z;

			updateFootGlobal();
			for (auto i = 0; i < swingControlPointsVec.size(); i++)
				swingControlPointsVec[i] = swingControlPoints[i] + getHeelPos(prevAnklePos).y;
			swingControlPointsVec[3] = 0.0f;

			for (auto i = 0; i < swingRotControlPointsVec.size(); i++)
				swingRotControlPointsVec[i] = swingRotControlPoints[i] + footRot;
			swingRotControlPointsVec[3] = -0.03;
		}

		float swingT = (t - PRE_SWING) / (TERMINAL_SWING - PRE_SWING);

		//rotate toe back to its original position
		if (t < (PRE_SWING + ((TERMINAL_SWING - PRE_SWING) * 0.2f))) {
			float angle = toeRot / (((PRE_SWING + ((TERMINAL_SWING - PRE_SWING) * 0.2f)) - t) / dt);
			toe->localMat = glm::rotate(toe->localMat, -angle, glm::vec3(1.0f, 0.0f, 0.0f));
			toeRot -= angle;
		}

		//rotate foot while being mid air
		float angle = bezierCurve(swingRotControlPointsVec, swingT) - footRot;
		foot->localMat = glm::rotate(foot->localMat, -angle, glm::vec3(1.0f, 0.0f, 0.0f));
		footRot += angle;

		//set new heel position
		updateFootGlobal();
		solveIK(getAnklePosFromHeel(glm::vec3(width, bezierCurve(swingControlPointsVec, swingT), lerp(toeOffPos, toeOffPos + swingDist, swingT))));
	
		//compute position again using new rotation so heel fits the curve more accurately
		updateFootGlobal();
		thigh->globalMat = root->localMat * thigh->localMat;
		shin->globalMat = thigh->globalMat * shin->localMat;
		solveIK(getAnklePosFromHeel(glm::vec3(width, bezierCurve(swingControlPointsVec, swingT), lerp(toeOffPos, toeOffPos + swingDist, swingT))));
	
		startNewStep = true; //so we know when to execute condition in initial phase
	}
}

glm::vec3 Leg::getToePos()
{
	return glm::vec3(foot->globalMat * glm::vec4(toePos, 1.0));
}

glm::vec3 Leg::getHeelPos(glm::vec3 pos)
{
	return glm::vec3(foot->globalMat * glm::vec4(heelPos, 1.0)) + pos - glm::vec3(foot->globalMat[3]);
}

glm::vec3 Leg::getAnklePosFromHeel(glm::vec3 pos)
{
	glm::vec3 actPos = getHeelPos(glm::vec3(foot->globalMat[3]));
	return glm::vec3(foot->globalMat[3]) + pos - actPos;
}

/*void Leg::riseHeel(float angle)
{
	//get new heel position y rotatting around toe
	glm::vec3 toePos = glm::vec3(glm::inverse(foot->globalMat) * toe->globalMat[3]);
	glm::mat4 rotatedMat(1.0);
	rotatedMat = glm::translate(glm::rotate(glm::translate(rotatedMat, -toePos), -angle, glm::vec3(1.0f, 0.0f, 0.0f)), toePos);
	solveIK(glm::vec3((foot->globalMat * rotatedMat)[3]));
	
	//rotate foot to point to toe using new global foot matrix
	updateFootGlobal();
	glm::vec3 newFootPosOnY = glm::vec3(foot->globalMat * glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
	glm::vec3 newFootPosOnX = glm::vec3(foot->globalMat * glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)); //ref pos for cross product
	glm::vec3 refV = glm::normalize(newFootPosOnX - glm::vec3(foot->globalMat[3]));
	glm::vec3 adjacent = glm::normalize(newFootPosOnY - glm::vec3(foot->globalMat[3]));
	glm::vec3 adjacent2 = glm::normalize(glm::vec3(toe->globalMat[3]) - glm::vec3(foot->globalMat[3]));
	float dot = glm::dot(adjacent, adjacent2);
	float phi = acos(dot <= 1.0f ? dot : 1.0f); //we dont want undefined value when float is not accurate enough
	phi = angle != 0.0 ? phi : 0.0f;
	phi = glm::dot(glm::cross(adjacent, adjacent2), refV) < 0.0 ? phi : -phi;
	foot->localMat = glm::rotate(foot->localMat, phi, glm::vec3(1.0f, 0.0f, 0.0f));
	footRot -= phi;

	//rotate fingers to point in the same direction they pointed before we raised heel
	glm::vec3 oldToePosOnY = glm::vec3(toe->globalMat * glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
	updateToeGlobal();
	glm::vec3 newToePosOnY = glm::vec3(toe->globalMat * glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
	glm::vec3 newToePosOnX = glm::vec3(toe->globalMat * glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)); //ref pos for cross product
	refV = glm::normalize(newFootPosOnX - glm::vec3(toe->globalMat[3]));
	adjacent = glm::normalize(oldToePosOnY - glm::vec3(toe->globalMat[3]));
	adjacent2 = glm::normalize(newToePosOnY - glm::vec3(toe->globalMat[3]));
	dot = glm::dot(adjacent, adjacent2);
	phi = acos(dot <= 1.0f ? dot : 1.0f); //we dont want undefined value when float is not accurate enough
	phi = angle != 0.0 ? phi : 0.0f;
	phi = glm::dot(glm::cross(adjacent, adjacent2), refV) < 0.0 ? -phi : phi;
	toe->localMat = glm::rotate(toe->localMat, phi, glm::vec3(1.0f, 0.0f, 0.0f));
	toeRot += phi;
}*/

//TODO BASEVEC SCHOULDNT BE COUNTED FROM FOOTVEC
void Leg::raiseHeel(glm::vec3 &baseVec, float startAngle, float endAngle, float t)
{
	//rotate foot to the correct angle with the ground
	glm::vec3 footVec = glm::normalize(getToePos() - getHeelPos(glm::vec3(foot->globalMat[3])));
	baseVec = glm::normalize(glm::vec3(footVec.x, 0.0, footVec.z));
	float phi = lerp(startAngle, endAngle, t) - getAngle(baseVec, footVec, foot->globalMat, glm::vec3(1.0, 0.0, 0.0));
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

	//rotate fingers so that overal rotation is the same as angle between ground and foot
	phi = lerp(startAngle, endAngle, t) - toeRot;
	toe->localMat = glm::rotate(toe->localMat, phi, glm::vec3(1.0f, 0.0f, 0.0f));
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

void Leg::solveIK(glm::vec3 desiredPos)
{
	prevAnklePos = desiredPos;

	//dont let it get out of reach
	if (glm::length(desiredPos - glm::vec3(thigh->globalMat[3])) >= (thighLength + shinLength - 0.0001f))
		desiredPos = glm::vec3(thigh->globalMat[3]) + glm::normalize(desiredPos - glm::vec3(thigh->globalMat[3])) * (thighLength + shinLength - 0.0001f);

	//transformation of spherical joint so it points at desired position
	glm::vec3 direction(glm::normalize(glm::vec3(glm::inverse(thigh->globalMat) * glm::vec4(desiredPos, 1.0))));
	glm::vec3 up = glm::vec3(0.0, 0.0, -1.0);
	glm::vec3 right = glm::normalize(glm::cross(up, direction));
	up = glm::normalize(glm::cross(right, direction));
	glm::mat4 mat(right.x, right.y, right.z, 0.0f,
		direction.x, direction.y, direction.z, 0.0f,
		up.x, up.y, up.z, 0.0f,
		0.0, 0.0, 0.0, 1.0f);

	thigh->localMat = thigh->localMat * mat;

	//angles in triangle made of joints and line between spherical joint and desired pos
	float y = thigh->globalMat[3].y - desiredPos.y, z = desiredPos.z - thigh->globalMat[3].z;
	float L = glm::length(glm::vec3(thigh->globalMat[3] - glm::vec4(desiredPos, 1.0)));
	float phi_t = atan(z / y);
	float phi_1 = acos((thighLength*thighLength + L*L - shinLength*shinLength) / (2 * thighLength*L));
	float phi_2 = PI - acos((thighLength*thighLength + shinLength*shinLength - L*L) / (2 * thighLength*shinLength));

	//transformation of joints
	thigh->localMat = glm::rotate(thigh->localMat, -phi_1, glm::vec3(1.0, 0.0, 0.0));
	shin->localMat = glm::rotate(shin->localMat, phi_2 - kneeRot, glm::vec3(1.0, 0.0, 0.0));
	kneeRot = phi_2;

	//count angle delta to rotate around vector from spherical joint and desired pos, so thigh is as close as possible to reference position
	//reference position is thigh in 2D (rotated only around x) rotated by the same angle plus a bit more to keep leg point forward
	glm::vec3 a = glm::normalize(glm::vec3(glm::vec4(desiredPos, 1.0) - thigh->globalMat[3]));
	glm::vec3 v = glm::normalize(glm::vec3(thigh->parent->localMat * thigh->localMat * glm::vec4(0.0, 1.0, 0.0, 1.0) - thigh->globalMat[3]));
	glm::mat4 referenceGlobal = root->localMat * thighBindMat;
	//float angleRef = getAngle(a, glm::normalize(glm::vec3(glm::rotate(referenceGlobal, -phi_t, glm::vec3(1.0, 0.0, 0.0)) * glm::vec4(0.0, 1.0, 0.0, 1.0) - referenceGlobal[3])), glm::rotate(referenceGlobal, -phi_t, glm::vec3(1.0, 0.0, 0.0)), glm::vec3(0.0, 0.0, 1.0));
	//referenceGlobal = glm::rotate(referenceGlobal, -angleRef/3, glm::vec3(0.0, 0.0, 1.0));
	glm::vec3 k = glm::normalize(glm::vec3(glm::rotate(referenceGlobal, -(PI * 0.5f), glm::vec3(1.0, 0.0, 0.0)) * glm::vec4(0.0, 1.0, 0.0, 1.0) - thigh->globalMat[3]));
	float A = glm::dot(glm::dot(a, v) * a, k) - glm::dot(glm::cross(glm::cross(a, v), a), k);
	float B = glm::dot(glm::cross(a, v), k);
	float C = glm::dot(v, k);

	float delta = atan2(2 * B, C - A);
	if ((((A - C) / 2.0) * cos(delta) - B * sin(delta)) > 0.0)
		delta += PI;

	//rotate by delta
	thigh->localMat = glm::rotate(thigh->localMat, phi_1, glm::vec3(1.0, 0.0, 0.0));
	thigh->localMat = glm::rotate(thigh->localMat, delta, glm::vec3(0.0, 1.0, 0.0));
	thigh->localMat = glm::rotate(thigh->localMat, -phi_1, glm::vec3(1.0, 0.0, 0.0));
}

float Leg::getLegLength()
{
	return thighLength + shinLength;
}
