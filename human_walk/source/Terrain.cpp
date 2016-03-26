#include "Terrain.h"

Terrain::Terrain(float _z1, float _z2, float maxH, float w, float res)
{
	z1 = _z1;
	z2 = _z2;
	maxHeight = maxH;
	width = w;
	resolution = res;

	y = std::vector<float>(static_cast<int>(ceil(z2 - z1) / resolution) + 1, 0.0f);
}

void Terrain::midPointStep(int i, int j, float range, float H)
{
	if (j - i <= 1)
		return;

	int mid = (i + j) / 2;
	y[mid] = (y[i] + y[j]) / 2.0f + ((double)rand() / (RAND_MAX)) * 2 * range - range;
	range = range * pow(2, -H);

	midPointStep(i, mid, range, H);
	midPointStep(mid, j, range, H);
}

void Terrain::midPoint(float range, float H)
{
	midPointStep(0, y.size() - 1, range, H);
}

void Terrain::fillModel(Model &m, Texture &t)
{
	Mesh mesh;
	int texRatio = static_cast<int>(width / resolution);

	for (unsigned int i = 0; i < y.size() - 1; i++) {
		glm::vec3 p1(-width / 2.0f, y[i], z1 + i*resolution);
		glm::vec3 p2(width / 2.0f, y[i + 1], z1 + (i + 1)*resolution);
		glm::vec3 p3(width / 2.0f, y[i], z1 + i*resolution);
		glm::vec3 n = glm::cross(p2 - p1, p3 - p1);

		mesh.addVertex(p1, n, glm::vec2(0.0f, (i % texRatio) / static_cast<float>(texRatio)));
		mesh.addVertex(p2, n, glm::vec2(1.0f, (i % texRatio) / static_cast<float>(texRatio) + 1.0f/texRatio));
		mesh.addVertex(p3, n, glm::vec2(1.0f, (i % texRatio) / static_cast<float>(texRatio)));

		mesh.addVertex(glm::vec3(-width / 2.0f, y[i], z1 + i*resolution), n, 
			glm::vec2(0.0f, (i % texRatio) / static_cast<float>(texRatio)));
		mesh.addVertex(glm::vec3(-width / 2.0f, y[i + 1], z1 + (i + 1)*resolution), n, 
			glm::vec2(0.0f, (i % texRatio) / static_cast<float>(texRatio)+1.0f / texRatio));
		mesh.addVertex(glm::vec3(width / 2.0f, y[i + 1], z1 + (i + 1)*resolution), n, 
			glm::vec2(1.0f, (i % texRatio) / static_cast<float>(texRatio)+1.0f / texRatio));
	}

	std::shared_ptr<Material> mat(new Material);
	mat->setDifTex(t);
	mesh.addMaterial(mat);
	m.addMesh(std::make_shared<Mesh>(mesh));
}

float Terrain::getHeight(glm::vec3 pos)
{
	int i = static_cast<int>((pos.z - z1) / resolution);

	if (i < 0 || pos.z > z2 || abs(pos.x) > width / 2.0f)
		return 0.0f;
	
	float t = (pos.z - i * resolution) / resolution;
	return y[i] + (y[i + 1] - y[i]) * t;
}