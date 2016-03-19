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
	float actCoeff = 1.0;
	float nextCoeff = 1.0;
	int prevSegment = -1;
	float prevX = -1.0;
	float solveForT(float *x, float X);
public:
	BezierCurve() = default;
	BezierCurve(const float *yPoints, int size, int repeat, bool changeOddSign);
	BezierCurve(const float *xPoints, const float *yPoints, int size, int repeat, bool changeOddSign);
	float solve(float *P, float t);
	float YfromX(float X);
	void setCoeff(float c) { nextCoeff = c;	}
	void setCoeffImmediately(float c) { actCoeff = c;  nextCoeff = c; }
	float getCoeff() { return nextCoeff; }
	float solveY(float t);
	float solveX(float t);
};

#endif //BEZIER_CURVE_H