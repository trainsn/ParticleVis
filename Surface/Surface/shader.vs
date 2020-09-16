#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in int cluster_id;

out vec3 ourColor;

void main()
{
    gl_Position = vec4(aPos, 1.0);
	gl_PointSize = 10.0f;
	if (cluster_id == 0)
		ourColor = vec3(1.0f, 0.0f, 0.0f);
	else if (cluster_id == 1)
		ourColor = vec3(0.0f, 1.0f, 0.0f);
	else if (cluster_id == 2)
		ourColor = vec3(0.0f, 0.0f, 1.0f);
	else if (cluster_id == 3)
		ourColor = vec3(1.0f);
}