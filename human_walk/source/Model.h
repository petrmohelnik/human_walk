#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <glm/glm.hpp>
#include <memory>

struct Texture
{
	std::vector<unsigned char> tex;
	unsigned int width;
	unsigned int height;
};

class Material
{
private:
	Texture difTex;
public:
	void setDifTex(Texture tex);
	Texture getDifTex();
};

class Mesh
{
protected:
	std::vector<glm::vec3> v; //pozice vrcholu
	std::vector<glm::vec3> n; //normaly
	std::vector<glm::vec2> t; //texturovaci koordinaty
	std::shared_ptr<Material> m; //material
public:
	virtual void addVertex(glm::vec3 vertex, glm::vec3 normal, glm::vec2 texCoord);
	void addMaterial(std::shared_ptr<Material> mat);
	float *getVertices();
	float *getNormals();
	float *getTexCoords();
	int getSize();
	std::shared_ptr<Material> getMaterial();
};

class WeightedMesh : public Mesh
{
private:
	std::vector<glm::vec4> weights; //vahy prirazenych kosti
	std::vector<glm::ivec4> jointIndices; //prirazene kosti
	//glm::mat4 bindMatrix;
public:
	void addVertex(glm::vec3 vertex, glm::vec3 normal, glm::vec2 texCoord, glm::vec4 w, glm::ivec4 j);
	void initWeightVectors();
	void addWeight(int i, glm::vec4 w, glm::ivec4 j);
	float *getWeights();
	int *getJointIndices();
	//void setBindMatrix(glm::mat4 m);
	//glm::mat4 getBindMatrix();
};

class Model
{
private:
	std::vector<std::shared_ptr<Mesh> > meshes;
public:
	std::vector<std::shared_ptr<Mesh> > getMeshes();
	int getMeshesSize();
	int addMesh(std::shared_ptr<Mesh> mesh);
};

#endif //MODEL_H