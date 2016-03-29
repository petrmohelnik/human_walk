#include "BezierCurve.h"

BezierCurve::BezierCurve(const float *yPoints, int size, int repeat, bool changeOddSign)
{
	for (int r = 0; r < repeat; r++) {
		for (int i = 0; i < size; i += 4) {
			y.push_back(std::vector<float>());
			for (int j = 0; j < 4; j++) {
				y.back().push_back(r % 2 == 1 && changeOddSign ? -yPoints[i + j] : yPoints[i + j]);
				refY.push_back(r % 2 == 1 && changeOddSign ? -yPoints[i + j] : yPoints[i + j]);
			}
		}
	}
}

BezierCurve::BezierCurve(const float *xPoints, const float *yPoints, int size, int repeat, bool changeOddSign)
{
	for (int r = 0; r < repeat; r++) {
		for (int i = 0; i < size; i += 4) {
			x.push_back(std::vector<float>());
			y.push_back(std::vector<float>());
			for (int j = 0; j < 4; j++) {
				x.back().push_back(xPoints[i + j] + r*xPoints[size-1]);
				refX.push_back(xPoints[i + j] + r*xPoints[size - 1]);
				y.back().push_back(r % 2 == 1 && changeOddSign ? -yPoints[i + j] : yPoints[i + j]);
				refY.push_back(r % 2 == 1 && changeOddSign ? -yPoints[i + j] : yPoints[i + j]);
			}
		}
	}

	addCoeff.resize(size);
}

float BezierCurve::solveForT(const float *x, float X) const
{
	float fa, fb, a = -0.01f, b = 1.01f;
	float fc, prevC, c;

	if ((fa = solve(x, a) - X) > (fb = solve(x, b) - X)) {
		float temp = a;	a = b; b = temp;
		temp = fa; fa = fb;	fb = temp;
	}

	c = b - ((b - a) / (fb - fa)) * fb;
	if ((fc = solve(x, c) - X) > 0.0){
		b = c;
		fb = fc;
	}
	else {
		a = c;
		fa = fc;
	}
//	prevC = c;
	int i = 1;
	//std::cout << "bezier i: " << i << ", t: " << c << std::endl;
	do {
		i++;
		prevC = c;
		c = b - ((b - a) / (fb - fa)) * fb;
		if ((fc = solve(x, c) - X) > 0.0){
			b = c;
			fb = fc;
		}
		else {
			a = c;
			fa = fc;
		}
		//std::cout << "bezier i: " << i << ", t: " << c << std::endl;
	} while (abs(prevC - c) > 0.001);

	return c;
}

float BezierCurve::YfromX(float X)
{
	int i = 0;
	float t = 0.0;

	while (X > x[i][3]) {
		i++;
	}

	if (prevSegment != i || X < prevX) {
		if (actCoeff != nextCoeff || actAddCoeff != nextAddCoeff) {
			for (int j = 0; j < 2; j++) {
				y[i][j] = refY[i * 4 + j] * actCoeff + actAddCoeff;
				x[i][j] = refX[i * 4 + j];
			}
			//x[i][1] = x[i][0] + (x[i][1] - x[i][0]) * (nextCoeff);
			//y[i][1] = y[i][0] + (y[i][1] - y[i][0]) * (nextCoeff);
			for (int j = 2; j < 4; j++) {
				y[i][j] = refY[i * 4 + j] * nextCoeff + nextAddCoeff;
				x[i][j] = refX[i * 4 + j];
			}
			actCoeff = nextCoeff;
			actAddCoeff = nextAddCoeff;
		}
		else {
			for (int j = 0; j < 4; j++) {
				y[i][j] = refY[i * 4 + j] * actCoeff + actAddCoeff;
				x[i][j] = refX[i * 4 + j];
			}
		}
	}

	t = solveForT(&x[i][0], X);

	prevSegment = i;
	prevX = X;

	return solve(&y[i][0], t);
}

float BezierCurve::YfromX(float X) const
{
	int i = 0;
	float t = 0.0;

	while (X > x[i][3]) {
		i++;
	}

	float constY[4], constX[4];

	if (prevSegment != i || X < prevX) {
		if (actCoeff != nextCoeff || actAddCoeff != nextAddCoeff) {
			for (int j = 0; j < 2; j++) {
				constY[j] = refY[i * 4 + j] * actCoeff + actAddCoeff;
				constX[j] = refX[i * 4 + j];
			}
			//x[i][1] = x[i][0] + (x[i][1] - x[i][0]) * (nextCoeff);
			//y[i][1] = y[i][0] + (y[i][1] - y[i][0]) * (nextCoeff);
			for (int j = 2; j < 4; j++) {
				constY[j] = refY[i * 4 + j] * nextCoeff + nextAddCoeff;
				constX[j] = refX[i * 4 + j];
			}
		}
		else {
			for (int j = 0; j < 4; j++) {
				constY[j] = refY[i * 4 + j] * actCoeff + actAddCoeff;
				constX[j] = refX[i * 4 + j];
			}
		}
	}

	for (int j = 0; j < 4; j++) {
		constY[j] = refY[i * 4 + j] * actCoeff + actAddCoeff;
		constX[j] = refX[i * 4 + j];
	}

	t = solveForT(&constX[0], X);

	return solve(&constY[0], t);
}

float BezierCurve::YfromXSwingFoot(float X)
{
	int i = 0;
	float t = 0.0;

	for (auto segment : x) {
		if (X <= segment[3]) {
			t = solveForT(&segment[0], X);
			break;
		}
		i++;
	}

	if (prevSegment != i || X < prevX) {
		for (int j = 0; j < 4; j++) {
			y[i][j] = refY[i * 4 + j] * actCoeff + addCoeff[i * 4 + j];
		}
	}

	return solve(&y[i][0], t);
}

void BezierCurve::stepDown(float down)
{
	addCoeff[10] -= down;
	addCoeff[11] -= down;
}

void BezierCurve::setSwingFootHeight(float c)
{
	for (unsigned int i = 0; i < addCoeff.size(); i++)
		addCoeff[i] = c;
}

void BezierCurve::stepUp(float c)
{
	for (unsigned int i = 6; i < addCoeff.size(); i++)
		addCoeff[i] += c;
}

void BezierCurve::swingIncrease(float inc)
{
	for (unsigned int i = 2; i < addCoeff.size() - 2; i++)
		addCoeff[i] += inc;
}

float BezierCurve::solveY(float t)
{
	unsigned int i = static_cast<int>(t);
	i = i >= y.size() ? i - 1 : i;

	if (prevSegment != i || t < prevX) {
		if (actCoeff != nextCoeff) {
			for (int j = 0; j < 2; j++) {
				y[i][j] = refY[i * 4 + j] * actCoeff;
			}
			for (int j = 3; j < 4; j++) {
				y[i][j] = refY[i * 4 + j] * nextCoeff;
			}
			actCoeff = nextCoeff;
		}
		else {
			for (int j = 0; j < 4; j++) {
				y[i][j] = refY[i * 4 + j] * actCoeff;
			}
		}
	}

	prevSegment = i;
	prevX = t;

	return solve(&y[i][0], t - i);
}

float BezierCurve::solveX(float t)
{
	unsigned int i = static_cast<int>(t);
	i = i >= x.size() ? i - 1 : i;

	return solve(&x[i][0], t - i);
}

float BezierCurve::solve(const float *P, float t) const
{
	float u = 1.0f - t;
	float uu = u*u;
	float uuu = uu*u;
	float tt = t*t;
	float ttt = tt*t;

	float p = P[0] * uuu;
	p += P[1] * 3 * t * uu;
	p += P[2] * 3 * tt * u;
	p += P[3] * ttt;

	return p;
}