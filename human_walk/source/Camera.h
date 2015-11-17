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
	glm::vec2 rot;
	int mode;
public:
	void init(float fov, int width, int height, float nearPlane, float farPlane, int _mode); //nacpat uhel pohledu a dalsi cipoviny
	void rotateX(float r);
	void rotateY(float r);
	void translate(glm::vec3 t);
	glm::mat4 getProjection();
	glm::mat4 getView();
	glm::vec3 getPos();
	glm::vec2 getRotation();
	void setMode(int mode);
};

#endif //CAMERA_H