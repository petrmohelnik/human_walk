#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>

#define CAM_TRANS_ROT 0
#define CAM_ROT_TRANS 1

class Camera
{
private:
	float fov;
	int width;
	int height;
	float nearPlane;
	float farPlane;
	glm::vec3 pos;
	glm::vec3 relativePos;
	glm::vec2 rot;
	int mode;
public:
	void init(float _fov, int _width, int _height, float _nearPlane, float _farPlane, int _mode); //nacpat uhel pohledu a dalsi cipoviny
	void rotateX(float r);
	void rotateY(float r);
	void translate(glm::vec3 t);
	void translateRelative(glm::vec3 t);
	glm::mat4 getProjection();
	glm::mat4 getView();
	glm::vec3 getPos();
	glm::vec2 getRotation();
	void setMode(int mode);
	void onWindowResize(int _width, int _height) { width = _width;  height = _height; }
};

#endif //CAMERA_H