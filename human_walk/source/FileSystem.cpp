#include "FileSystem.h"

std::vector<std::string> &FileSystem::split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}

std::vector<std::string> FileSystem::split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, elems);
	return elems;
}

bool FileSystem::loadFile(const char *path, std::string &buffer)
{
	char *b;

	std::ifstream file(path, std::ios::in | std::ios::binary | std::ios::ate);
	if (file.is_open())
	{
		file.seekg(0, file.end);
		int length = (int)file.tellg();
		file.seekg(0, file.beg);

		b = new char[length + 1];

		file.read(b, length);
		file.close();

		b[length] = '\0';

		std::cout << "file " << path << " loaded" << std::endl;
	}
	else
	{
		std::cout << "Unable to open file " << path << std::endl;
		return false;
	}

	buffer.append(b);
	return true;
}

bool FileSystem::parseObj(const char *path, Model &m)
{
	std::string line;
	std::ifstream fileStream(path);
	unsigned int ind = 0; //amout of indices

	std::vector<glm::vec3> v, n;
	std::vector<glm::vec2> t;

	if (fileStream.is_open())
	{
		while (fileStream.good())
		{
			getline(fileStream, line);

			std::vector<std::string> tokens = split(line, ' ');

			if (line.substr(0, 2) == "v ")
			{
				float x = (float)atof(tokens[1].c_str());
				float y = (float)atof(tokens[2].c_str());
				float z = (float)atof(tokens[3].c_str());
				v.push_back(glm::vec3(x, y, z));
			}

			else if (line.substr(0, 2) == "vt")
			{
				float x = (float)atof(tokens[1].c_str());
				float y = (float)atof(tokens[2].c_str());
				t.push_back(glm::vec2(x, y));
			}

			else if (line.substr(0, 2) == "vn")
			{
				float x = (float)atof(tokens[1].c_str());
				float y = (float)atof(tokens[2].c_str());
				float z = (float)atof(tokens[3].c_str());
				n.push_back(glm::vec3(x, y, z));
			}

			else if (line.substr(0, 2) == "f ")
			{
				glm::vec3 vertex, normal;
				glm::vec2 texCoord;

				std::vector<std::string> v1 = split(tokens[1], '/');
				int i = atoi(v1[0].c_str()) - 1;
				vertex.x = v[i].x; vertex.y = v[i].y; vertex.z = v[i].z;
				i = atoi(v1[1].c_str()) - 1;
				texCoord.x = t[i].x; texCoord.y = t[i].y;
				i = atoi(v1[2].c_str()) - 1;
				normal.x = n[i].x; normal.y = n[i].y; normal.z = n[i].z;
				m.addVertex(vertex, normal, texCoord);

				std::vector<std::string> v2 = split(tokens[2], '/');
				i = atoi(v2[0].c_str()) - 1;
				vertex.x = v[i].x; vertex.y = v[i].y; vertex.z = v[i].z;
				i = atoi(v2[1].c_str()) - 1;
				texCoord.x = t[i].x; texCoord.y = t[i].y;
				i = atoi(v2[2].c_str()) - 1;
				normal.x = n[i].x; normal.y = n[i].y; normal.z = n[i].z;
				m.addVertex(vertex, normal, texCoord);

				std::vector<std::string> v3 = split(tokens[3], '/');
				i = atoi(v3[0].c_str()) - 1;
				vertex.x = v[i].x; vertex.y = v[i].y; vertex.z = v[i].z;
				i = atoi(v3[1].c_str()) - 1;
				texCoord.x = t[i].x; texCoord.y = t[i].y;
				i = atoi(v3[2].c_str()) - 1;
				normal.x = n[i].x; normal.y = n[i].y; normal.z = n[i].z;
				m.addVertex(vertex, normal, texCoord);
			}
		}

		fileStream.close();
		return true;
	}

	std::cout << "Unable to open file " << path << std::endl;
	return false;
}

int FileSystem::findLastOpen(std::vector<bool> &closed)
{
	for (int i = closed.size() - 1; i >= 0; i--) {
		if (!closed[i])
			return i;
	}

	return -1;
}

bool FileSystem::loadModelAndSkeletonDae(const char *path, Model &m, Skeleton &s)
{
	std::string line;
	std::ifstream fileStream(path);
	std::vector<bool> closed;

	std::vector<float> v, n, t;

	if (fileStream.is_open())
	{
		getline(fileStream, line);
		while (line.find("<library_geometries>") == std::string::npos) {
			getline(fileStream, line);
		}

		while (line.find("<mesh>") == std::string::npos) {
			getline(fileStream, line);
		}

		//load model
		while (fileStream.good())
		{
			getline(fileStream, line);
			size_t pos;

			if ((pos = line.find("<source id=")) != std::string::npos) {
				std::string id = line;

				getline(fileStream, line);
				std::string values = line.substr((pos = line.find_first_of(">")) + 1, line.find_first_of("<", pos + 1) - (pos + 1));
				std::vector<std::string> tokens = split(values, ' ');
				for (unsigned int i = 0; i < tokens.size(); i++) {
					if ((pos = id.find("-positions")) != std::string::npos) {
						v.push_back((float)atof(tokens[i].c_str()));
					}
					else if ((pos = id.find("-normals")) != std::string::npos) {
						n.push_back((float)atof(tokens[i].c_str()));
					}
					else if ((pos = id.find("-map")) != std::string::npos) {
						t.push_back((float)atof(tokens[i].c_str()));
					}
				}
			}
			else if ((pos = line.find("<polylist")) != std::string::npos) {
				while (line.find("<p>") == std::string::npos) {
					getline(fileStream, line);
				}

				std::string values = line.substr((pos = line.find_first_of(">")) + 1, line.find_first_of("<", pos + 1) - (pos + 1));
				std::vector<std::string> tokens = split(values, ' ');
				std::vector<int> indices;
				indices.resize(tokens.size());
				for (unsigned int i = 0; i < tokens.size(); i++) {
					indices.push_back(atoi(tokens[i].c_str()));
				}

				for (unsigned int i = 0; i < indices.size(); i += 3) {
					glm::vec3 vertex, normal;
					glm::vec2 texCoord;

					vertex.x = v[indices[i] * 3];
					vertex.y = v[indices[i] * 3 + 1];
					vertex.z = v[indices[i] * 3 + 2];

					normal.x = n[indices[i + 1] * 3];
					normal.y = n[indices[i + 1] * 3 + 1];
					normal.z = n[indices[i + 1] * 3 + 2];

					if (t.size() > 0) {
						texCoord.x = t[indices[i + 2] * 2];
						texCoord.y = t[indices[i + 2] * 2 + 1];
					}
					else
						texCoord = glm::vec2(0.0f);

					m.addVertex(vertex, normal, texCoord);
				}

				break;
			}
		}


		while (line.find("<library_visual_scenes>") == std::string::npos) {
			getline(fileStream, line);
		}

		while (line.find("node id=\"metarig\"") == std::string::npos) {
			getline(fileStream, line);
		}

		//load skeleton
		while (fileStream.good())
		{
			getline(fileStream, line);
			size_t pos;

			if (line.find("</node>") != std::string::npos) {
				int last = findLastOpen(closed);
				if (last < 0)
					break;

				closed[last] = true;;
			}

			else if ((pos = line.find("<node id=")) != std::string::npos) {
				int parent = findLastOpen(closed);
				std::string name = line.substr((pos = line.find_first_of("\"")) + 1, line.find_first_of("\"", pos + 1) - (pos + 1));
					
				getline(fileStream, line);
				std::string matrix = line.substr((pos = line.find_first_of(">")) + 1, line.find_first_of("<", pos + 1) - (pos + 1));
				std::vector<std::string> tokens = split(matrix, ' ');
				glm::mat4 m;
				for (int i = 0; i < 16; i++)
					m[i / 4][i % 4] = (float)atof(tokens[i].c_str());
				s.addBone(glm::transpose(m), findLastOpen(closed), name.c_str());

				closed.push_back(false);
			}
		}

		s.fixScale();

		fileStream.close();
		return true;
	}

	std::cout << "Unable to open file " << path << std::endl;
	return false;
}