#version 450

uniform mat4 mvp;
uniform mat3 mv;
uniform mat4 globalM[125];
uniform mat4 inverseM[125];

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
	f_pos = mv * v_pos;
	f_texCoord = v_texCoord;
	f_norm = v_norm;

	vec4 pos = vec4(0.0);
	pos += inverseM[v_joints.x] * globalM[v_joints.x] * vec4(v_pos, 1.0) * v_weights.x;
	pos += inverseM[v_joints.y] * globalM[v_joints.y] * vec4(v_pos, 1.0) * v_weights.y;
	pos += inverseM[v_joints.z] * globalM[v_joints.z] * vec4(v_pos, 1.0) * v_weights.z;
	pos += inverseM[v_joints.w] * globalM[v_joints.w] * vec4(v_pos, 1.0) * v_weights.w;

	gl_Position = mvp * vec4(pos.xyz, 1.0);
	//gl_Position = mvp * globalM[v_joints.x] * inverseM[v_joints.x] * vec4(v_pos, 1.0);
}