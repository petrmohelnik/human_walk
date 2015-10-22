#version 450

uniform mat4 mvp;
uniform mat3 mv;

in vec3 v_pos;
in vec3 v_norm;
in vec2 v_texCoord;

out vec3 f_pos;
out vec3 f_norm;
out vec2 f_texCoord;

void main()
{
	f_pos = mv * v_pos;
	f_texCoord = v_texCoord;
	f_norm = v_norm;

	gl_Position = mvp * vec4(v_pos, 1.0);
}