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
	return glm::perspective(fov, width / (float)height, nearPlane, farPlane);
}

glm::mat4 Camera::getView()
{
	glm::mat4 view = glm::mat4(1.0);

	if (mode == CAM_ROT_TRANS) {
		view = glm::rotate(view, rot.y, glm::vec3(1.0f, 0.0f, 0.0f));
		view = glm::rotate(view, rot.x, glm::vec3(0.0f, 1.0f, 0.0f));
		view = glm::translate(view, pos);
	}
	else {
		view = glm::translate(view, pos);
		view = glm::rotate(view, rot.y, glm::vec3(1.0f, 0.0f, 0.0f));
		view = glm::rotate(view, rot.x, glm::vec3(0.0f, 1.0f, 0.0f));
	}

	return view;
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