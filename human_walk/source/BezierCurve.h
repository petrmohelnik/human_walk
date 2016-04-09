#ifndef BEZIER_CURVE_H
#define BEZIER_CURVE_H

#include <vector>
#include <iostream>

class BezierCurve
{
private:
	std::vector<std::vector<float>> x;
	std::vector<std::vector<float>> y;
	std::vector<float> refY;
	std::vector<float> refX;
	std::vector<float> addCoeff;
	float actCoeff = 1.0f;
	float nextCoeff = 1.0f;
	float actAddCoeff = 0.0f;
	float nextAddCoeff = 0.0f;
	int prevSegment = -1;
	float prevX = -1.0f;
	float solveForT(const float *x, float X) const;
public:
	BezierCurve() = default;
	BezierCurve(const float *yPoints, int size, int repeat, bool changeOddSign);
	BezierCurve(const float *xPoints, const float *yPoints, int size, int repeat, bool changeOddSign);
	float solve(const float *P, float t) const;
	float YfromX(float X);
	float YfromX(float X) const;
	//float YfromXSwingFoot(float X);
	void stepDown(float down);
	void resetPelvisAddCoeff();
	void resetPelvisAddCoeff(float lastAddCoeff);
	void pelvisUp(float up);
	void pelvisDown(float down);
	void setSwingFootHeight(float c);
	void stepUp(float up);
	void controlPointUp(unsigned int i, float up);
	void increase(float pos, float up);
	void swingIncrease(float inc);
	void resetCoeff() { actCoeff = 1.0f; nextCoeff = 1.0f; actAddCoeff = 0.0f; nextAddCoeff = 0.0f; }
	void setCoeff(float c) { nextCoeff = c;	}
	void setAddCoeff(float c) { nextAddCoeff = c; }
	void setAddCoeff(int i, float c) { addCoeff[i] = c; }
	void setAddCoeffImmediately(float c) { actAddCoeff = c;  nextAddCoeff = c; }
	void incrementAddCoeff(float c) { nextAddCoeff += c; }
	void setCoeffImmediately(float c) { actCoeff = c;  nextCoeff = c; }
	float getNextCoeff() const { return nextCoeff; }
	float getActCoeff() const { return actCoeff; }
	float getNextAddCoeff() const { return nextAddCoeff; }
	float getActAddCoeff() const { return actAddCoeff; }
	float getLastAddCoeff() const { return addCoeff.back(); }
	float solveY(float t);
	float solveX(float t) const;
	float solveY(float t) const;
	void recountOnNext() { prevSegment = -1; }
};

#endif //BEZIER_CURVE_H