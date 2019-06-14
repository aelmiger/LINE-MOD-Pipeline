#version 330 core

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_color;

uniform mat4 u_modelViewProj;
out vec4 v_color;

void main()
{
    gl_Position = u_modelViewProj * vec4(a_position, 1.0f);
	v_color =vec4(a_color,1.0f);

}