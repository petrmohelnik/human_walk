#version 450

uniform mat4 mvp;
uniform mat3 mv;
uniform mat4 skinningMatrix[250];

in vec3 v_pos;
in vec3 v_norm;
in vec2 v_texCoord;
in vec4 v_weights;
in ivec4 v_joints;

out vec3 f_pos;
out vec3 f_norm;
out vec2 f_texCoord;

void main()
{
	f_texCoord = v_texCoord;

	vec4 pos = vec4(0.0);
	pos += skinningMatrix[v_joints.x] * vec4(v_pos, 1.0) * v_weights.x;
	pos += skinningMatrix[v_joints.y] * vec4(v_pos, 1.0) * v_weights.y;
	pos += skinningMatrix[v_joints.z] * vec4(v_pos, 1.0) * v_weights.z;
	pos += skinningMatrix[v_joints.w] * vec4(v_pos, 1.0) * v_weights.w;
	vec4 norm = vec4(0.0);
	norm += transpose(inverse(skinningMatrix[v_joints.x])) * vec4(v_norm, 1.0) * v_weights.x;
	norm += transpose(inverse(skinningMatrix[v_joints.y])) * vec4(v_norm, 1.0) * v_weights.y;
	norm += transpose(inverse(skinningMatrix[v_joints.z])) * vec4(v_norm, 1.0) * v_weights.z;
	norm += transpose(inverse(skinningMatrix[v_joints.w])) * vec4(v_norm, 1.0) * v_weights.w;
	
	f_norm = normalize(vec3(norm));
	f_pos = mv * pos.xyz;
	gl_Position = mvp * vec4(pos.xyz, 1.0);
	//gl_Position = mvp * globalM[v_joints.x] * inverseM[v_joints.x] * vec4(v_pos, 1.0);
}