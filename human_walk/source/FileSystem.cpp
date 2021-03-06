#include "FileSystem.h"

std::vector<std::string> &FileSystem::split(std::string::iterator b, std::string::iterator e, char delim, std::vector<std::string> &elems) {
	std::stringstream ss;
	std::copy(b, e, std::ostream_iterator<char>(ss));
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}

std::vector<int> &FileSystem::split(std::string::iterator b, std::string::iterator e, char delim, std::vector<int> &elems) {
	std::stringstream ss;
	std::copy(b, e, std::ostream_iterator<char>(ss));
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(atoi(item.c_str()));
	}
	return elems;
}

std::vector<float> &FileSystem::split(std::string::iterator b, std::string::iterator e, char delim, std::vector<float> &elems) {
	std::stringstream ss;
	std::copy(b, e, std::ostream_iterator<char>(ss));
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back((float)atof(item.c_str()));
	}
	return elems;
}

std::vector<std::string> FileSystem::split(std::string &s, char delim) {
	std::vector<std::string> elems;
	split(s.begin(), s.end(), delim, elems);
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

bool FileSystem::loadTexture(const char *path, Texture &img)
{
	unsigned int error = lodepng::decode(img.tex, img.width, img.height, path);

	if (error) {
		std::cout << path <<  ": Decoder error " << error << ": " << lodepng_error_text(error) << std::endl;
		return false;
	}

	return true;
}

bool FileSystem::parseObj(const char *path, Model &m)
{
	std::string line;
	std::ifstream fileStream(path);
	unsigned int ind = 0; //amout of indices
	Mesh mesh;

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
				mesh.addVertex(vertex, normal, texCoord);

				std::vector<std::string> v2 = split(tokens[2], '/');
				i = atoi(v2[0].c_str()) - 1;
				vertex.x = v[i].x; vertex.y = v[i].y; vertex.z = v[i].z;
				i = atoi(v2[1].c_str()) - 1;
				texCoord.x = t[i].x; texCoord.y = t[i].y;
				i = atoi(v2[2].c_str()) - 1;
				normal.x = n[i].x; normal.y = n[i].y; normal.z = n[i].z;
				mesh.addVertex(vertex, normal, texCoord);

				std::vector<std::string> v3 = split(tokens[3], '/');
				i = atoi(v3[0].c_str()) - 1;
				vertex.x = v[i].x; vertex.y = v[i].y; vertex.z = v[i].z;
				i = atoi(v3[1].c_str()) - 1;
				texCoord.x = t[i].x; texCoord.y = t[i].y;
				i = atoi(v3[2].c_str()) - 1;
				normal.x = n[i].x; normal.y = n[i].y; normal.z = n[i].z;
				mesh.addVertex(vertex, normal, texCoord);
			}
		}

		std::shared_ptr<Material> mat(new Material);
		Texture tex;
		if (!loadTexture("resource/white_D.png", tex))
			return false;
		mat->setDifTex(tex);
		mesh.addMaterial(mat);

		m.addMesh(std::make_shared<Mesh>(mesh));

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
	size_t pos;
	std::string line, armatureName;
	std::ifstream fileStream(path);
	std::vector<bool> closed;

	glm::mat4 bindShapeMatrix;
	std::vector<float> v, n, t, w;
	std::vector<glm::mat4> bindPoses;
	std::vector<std::vector<int> > weightJoints, weightWeights, indices;
	std::vector<std::string> polylistMaterials;
	std::vector<int> vcount;

	if (fileStream.is_open())
	{
		getline(fileStream, line);
		while (line.find("<library_geometries>") == std::string::npos && fileStream.good()) {
			getline(fileStream, line);
		}

		while (line.find("<mesh>") == std::string::npos && fileStream.good()) {
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
				std::string::iterator begIt = line.begin() + (pos = line.find_first_of(">")) + 1;
				std::string::iterator endIt = line.begin() + line.find_first_of("<", pos + 1);

				if ((pos = id.find("-positions")) != std::string::npos) {
					v.reserve(atoi(line.substr((pos = line.find("count=\"")) + 7, line.find_first_of("\"", pos + 1) - (pos + 1)).c_str()));
					split(begIt, endIt, ' ', v);
				}
				else if ((pos = id.find("-normals")) != std::string::npos) {
					n.reserve(atoi(line.substr((pos = line.find("count=\"")) + 7, line.find_first_of("\"", pos + 1) - (pos + 1)).c_str()));
					split(begIt, endIt, ' ', n);
				}
				else if ((pos = id.find("-map")) != std::string::npos) {
					t.reserve(atoi(line.substr((pos = line.find("count=\"")) + 7, line.find_first_of("\"", pos + 1) - (pos + 1)).c_str()));
					split(begIt, endIt, ' ', t);
				}
			}
			else if ((pos = line.find("<polylist")) != std::string::npos) {
				polylistMaterials.push_back(line.substr((pos = line.find("material=\"") + 10), line.find("-material\"", pos + 1) - pos));
				indices.push_back(std::vector<int>());
				indices.back().reserve(atoi(line.substr((pos = line.find("count=\"")) + 7, line.find_first_of("\"", pos + 1) - (pos + 1)).c_str()));
				while (line.find("<p>") == std::string::npos && fileStream.good()) {
					getline(fileStream, line);
				}
				std::string::iterator begIt = line.begin() + (pos = line.find_first_of(">")) + 1;
				std::string::iterator endIt = line.begin() + line.find_first_of("<", pos + 1);
				split(begIt, endIt, ' ', indices.back());
			}
			else if ((pos = line.find("</mesh>")) != std::string::npos) {
				break;
			}
		}

		while (line.find("<library_controllers>") == std::string::npos && fileStream.good()) {
			getline(fileStream, line);
		}

		while (line.find("</library_controllers>") == std::string::npos && fileStream.good()) {
			if (line.find("<controller id") != std::string::npos) {
				if ((pos = line.find("-skin\"")) != std::string::npos) {
					pos = line.find("name=");
					armatureName = line.substr((pos = line.find_first_of("\"", pos)) + 1, line.find_first_of("\"", pos + 1) - (pos + 1));
					break;
				}
			}
			getline(fileStream, line);
		}

		while (line.find("<bind_shape_matrix>") == std::string::npos && fileStream.good()) {
			getline(fileStream, line);
		}
		{
			std::string::iterator begIt = line.begin() + (pos = line.find_first_of(">")) + 1;
			std::string::iterator endIt = line.begin() + line.find_first_of("<", pos + 1);
			std::vector<float> matrixValues;
			split(begIt, endIt, ' ', matrixValues);
			for (unsigned int i = 0; i < 16; i++)
				bindShapeMatrix[i / 4][i % 4] = matrixValues[i];
			bindShapeMatrix = glm::transpose(bindShapeMatrix);
		}

		//load vertex weights
		while (fileStream.good())
		{
			getline(fileStream, line);

			if ((pos = line.find("<source id=")) != std::string::npos) {
				getline(fileStream, line);
				std::string::iterator begIt = line.begin() + (pos = line.find_first_of(">")) + 1;
				std::string::iterator endIt = line.begin() + line.find_first_of("<", pos + 1);

				if ((pos = line.find("-bind_poses-array")) != std::string::npos) {
					std::vector<float> matrixValues;
					matrixValues.reserve(atoi(line.substr((pos = line.find("count=\"")) + 7, line.find_first_of("\"", pos + 1) - (pos + 1)).c_str()));
					split(begIt, endIt, ' ', matrixValues);
					for (unsigned int i = 0; i < matrixValues.size(); i++) {
						glm::mat4 m;
						for (int j = 0; j < 16; j++) {
							m[j / 4][j % 4] = matrixValues[i];
							i++;
						}
						i--;
						bindPoses.push_back(m);
					}
				}
				else if ((pos = line.find("-weights-array")) != std::string::npos) {
					w.reserve(atoi(line.substr((pos = line.find("count=\"")) + 7, line.find_first_of("\"", pos + 1) - (pos + 1)).c_str()));
					split(begIt, endIt, ' ', w);
				}
			}
			else if (line.find("<vertex_weights") != std::string::npos)
				break;
		}

		int weightsCount = atoi(line.substr((pos = line.find("count=\"")) + 7, line.find_first_of("\"", pos + 1) - (pos + 1)).c_str());

		while (line.find("<vcount>") == std::string::npos && fileStream.good()) {
			getline(fileStream, line);
		}
		std::string::iterator begIt = line.begin() + (pos = line.find_first_of(">")) + 1;
		std::string::iterator endIt = line.begin() + line.find_first_of("<", pos + 1);
		vcount.reserve(weightsCount);
		split(begIt, endIt, ' ', vcount);
		while (line.find("<v>") == std::string::npos && fileStream.good()) {
			getline(fileStream, line);
		}
		begIt = line.begin() + (pos = line.find_first_of(">")) + 1;
		endIt = line.begin() + line.find_first_of("<", pos + 1);
		std::vector<int> weights;
		weights.reserve(weightsCount);
		split(begIt, endIt, ' ', weights);
		int k = 0;
		weightJoints.reserve(vcount.size());
		weightWeights.reserve(vcount.size());
		for (unsigned int i = 0; i < vcount.size(); i++) {
			weightJoints.push_back(std::vector<int>()); weightJoints.back().reserve(vcount[i]);
			weightWeights.push_back(std::vector<int>()); weightWeights.back().reserve(vcount[i]);
			for (int j = 0; j < vcount[i]; j++) {
				weightJoints[i].push_back(weights[k]); k++;
				weightWeights[i].push_back(weights[k]); k++;
			}
		}

		while (line.find("<library_visual_scenes>") == std::string::npos && fileStream.good()) {
			getline(fileStream, line);
		}
		
		while (line.find("node id=\"" + armatureName + "\"") == std::string::npos && fileStream.good()) {
			getline(fileStream, line);
		}
		getline(fileStream, line);
		{
			std::string::iterator begIt = line.begin() + (pos = line.find_first_of(">")) + 1;
			std::string::iterator endIt = line.begin() + line.find_first_of("<", pos + 1);
			std::vector<float> mat; mat.reserve(16);
			split(begIt, endIt, ' ', mat);
			glm::mat4 m;
			for (unsigned int i = 0; i < 16; i++)
				m[i / 4][i % 4] = mat[i];
			s.setRootTransformMatrix(glm::transpose(m));
		}

		s.setBonesNumber(bindPoses.size());

		//load skeleton
		while (fileStream.good())
		{
			getline(fileStream, line);

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
				std::string::iterator begIt = line.begin() + (pos = line.find_first_of(">")) + 1;
				std::string::iterator endIt = line.begin() + line.find_first_of("<", pos + 1);
				std::vector<float> mat; mat.reserve(16);
				split(begIt, endIt, ' ', mat);
				glm::mat4 m;
				for (unsigned int i = 0; i < 16; i++)
					m[i / 4][i % 4] = mat[i];

				s.addBone(glm::transpose(m), findLastOpen(closed));

				closed.push_back(false);
			}
		}

		if (!fileStream.good()) {
			std::cout << "File is in incorrect format " << path << std::endl;
			return false;
		}


		//add inverse bind pose matrices
		for (unsigned int i = 0; i < bindPoses.size(); i++) {
			s.getBone(i)->inverseBindMatrix = glm::transpose(bindPoses[i]) * bindShapeMatrix;
		}

		s.fixScale();

		//sort weights
		std::vector<std::vector<float> > wSorted;
		std::vector<std::vector<int> > jSorted;
		wSorted.reserve(weightJoints.size());
		jSorted.reserve(weightJoints.size());
		for (unsigned int i = 0; i < weightJoints.size(); i++) {
			wSorted.push_back(std::vector<float>()); wSorted.back().reserve(weightJoints[i].size());
			jSorted.push_back(std::vector<int>()); jSorted.back().reserve(weightJoints[i].size());
			for (unsigned int j = 0; j < weightJoints[i].size(); j++) {
				if (j == 0) {
					wSorted[i].push_back(w[weightWeights[i][j]]);
					jSorted[i].push_back(weightJoints[i][j]);
				}
				else {
					for (unsigned int k = 0; k < wSorted[i].size(); k++) {
						if (w[weightWeights[i][j]] > wSorted[i][k]) {
							wSorted[i].insert(wSorted[i].begin() + k, w[weightWeights[i][j]]);
							jSorted[i].insert(jSorted[i].begin() + k, weightJoints[i][j]);
							break;
						}
						if (k == wSorted[i].size() - 1) {
							wSorted[i].push_back(w[weightWeights[i][j]]);
							jSorted[i].push_back(weightJoints[i][j]);
							break;
						}
					}
				}
			}
		}

		for (unsigned int i = 0; i < wSorted.size(); i++) {
			if (wSorted[i].size() > 4) {
				for (unsigned int j = wSorted[i].size() - 1; j > 3; j--) {
					for (int k = 0; k < 4; k++)
						wSorted[i][k] += wSorted[i][j] * 0.25f;

					wSorted[i].erase(wSorted[i].begin() + j);
					jSorted[i].erase(jSorted[i].begin() + j);
				}
			}
		}

		//push in faces
		for (unsigned int k = 0; k < indices.size(); k++)
		{
			WeightedMesh mesh;
			mesh.reserve(indices[k].size());
			for (unsigned int i = 0; i < indices[k].size(); i += 3) {
				glm::vec3 vertex, normal;
				glm::vec2 texCoord;
				glm::vec4 weights;
				glm::ivec4 joints;

				vertex.x = v[indices[k][i] * 3];
				vertex.y = v[indices[k][i] * 3 + 1];
				vertex.z = v[indices[k][i] * 3 + 2];
				weights = glm::vec4(0.0f); joints = glm::ivec4(0);

				for (unsigned int j = 0; j < wSorted[indices[k][i]].size(); j++) {
					weights[j] = wSorted[indices[k][i]][j];
					joints[j] = jSorted[indices[k][i]][j];
				}

				normal.x = n[indices[k][i + 1] * 3];
				normal.y = n[indices[k][i + 1] * 3 + 1];
				normal.z = n[indices[k][i + 1] * 3 + 2];

				if (t.size() > 0) {
					texCoord.x = t[indices[k][i + 2] * 2];
					texCoord.y = 1.0f - t[indices[k][i + 2] * 2 + 1]; //OpenGL coords
				}
				else
					texCoord = glm::vec2(0.0f);

				mesh.addVertex(vertex, normal, texCoord, weights, joints);
			}

			std::shared_ptr<Material> mat(new Material);
			Texture tex;
			if (!loadTexture(std::string("resource/").append(polylistMaterials[k].append("_D.png")).c_str(), tex)) {
				if (!loadTexture("resource/white_D.png", tex))
					return false;
			}
			mat->setDifTex(tex);
			mesh.addMaterial(mat);

			m.addMesh(std::make_shared<WeightedMesh>(mesh));
		}

		fileStream.close();
		return true;
	}
	
	std::cout << "Unable to open file " << path << std::endl;
	return false;
}