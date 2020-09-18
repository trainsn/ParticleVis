#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in int cluster_id;

out vec3 ourColor;

//3D transformation matrices
uniform mat4 uMVMatrix;
uniform mat4 uPMatrix;

void main()
{
    gl_Position = uPMatrix * uMVMatrix * vec4(aPos, 1.0);
	gl_PointSize = 1.5f;
	if (cluster_id == 0)
		//ourColor = vec3(0.2f, 0.2f, 0.2f);
		ourColor = vec3(0.2f, 0.2f, 0.2f);
	if (cluster_id == 1)
		//ourColor = vec3(0.2f, 0.2f, 0.2f);
		ourColor = vec3(1.0f, 0.0f, 0.0f);	
	if (cluster_id == 2)
		//ourColor = vec3(0.2f, 0.2f, 0.2f);
		ourColor = vec3(0.0f, 1.0f, 0.0f);
	if (cluster_id == 3)	
		//ourColor = vec3(1.0f, 0.0f, 0.0f);
		ourColor = vec3(0.0f, 0.0f, 1.0f);
}