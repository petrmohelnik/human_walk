#include "Camera.h"

void Camera::init(float _fov, int _width, int _height, float _nearPlane, float _farPlane, int _mode)
{
	fov = _fov;
	width = _width;
	height = _height;
	nearPlane = _nearPlane;
	farPlane = _farPlane;
	rot = glm::vec2(0.0f);
	pos = glm::vec3(0.0f);
	mode = _mode;
}

void Camera::rotateX(float r)
{
	rot.x += r;
}

void Camera::rotateY(float r)
{
	rot.y += r;
}

void Camera::translate(glm::vec3 t)
{
	pos -= t;
}

glm::mat4 Camera::getProjection()
{
	glm::mat4 projection = glm::perspective(fov, width / (float)height, nearPlane, farPlane);
	if (mode == CAM_ROT_TRANS) {
		projection = glm::rotate(projection, rot.y, glm::vec3(1.0f, 0.0f, 0.0f));
		projection = glm::rotate(projection, rot.x, glm::vec3(0.0f, 1.0f, 0.0f));
		projection = glm::translate(projection, pos);
	}
	else {
		projection = glm::translate(projection, pos);
		projection = glm::rotate(projection, rot.y, glm::vec3(1.0f, 0.0f, 0.0f));
		projection = glm::rotate(projection, rot.x, glm::vec3(0.0f, 1.0f, 0.0f));
	}
	return projection;
}

glm::vec3 Camera::getPos()
{
	if (mode == CAM_ROT_TRANS)
		return pos;
	else {
		glm::mat4 m = glm::mat4(1.0);
		m = glm::rotate(m, -rot.x, glm::vec3(0.0f, 1.0f, 0.0f));
		m = glm::rotate(m, -rot.y, glm::vec3(1.0f, 0.0f, 0.0f));
		m = glm::translate(m, glm::vec3(pos.x, pos.y, pos.z));
		glm::vec4 p = m * glm::vec4(0.0, 0.0, 0.0, 1.0);
		return glm::vec3(p);
	}
}

glm::vec2 Camera::getRotation()
{
	return rot;
}

void Camera::setMode(int m)
{
	mode = m;
}