#include <glad/glad.h>
#include <GLFW/glfw3.h>
#define _USE_MATH_DEFINES

#include <GL/glm/glm.hpp>
#include <GL/glm/gtc/matrix_transform.hpp>
#include <GL/glm/gtc/type_ptr.hpp>
#include <GL/glm/gtx/transform2.hpp>

#include"cnpy.h"
#include<cstdlib>
#include<iostream>
#include <math.h>
#include<string>

#include "shader.h"

#include "static_kd_tree_3d.hpp"
#include "point_cloud.hpp"
#include "point_type.hpp"
using namespace std;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 800;

glm::mat4 pMatrix;
glm::mat4 mvMatrix;

// Information related to the camera 
float dist = 3.0f;
float theta, phi;
glm::vec3 direction;
glm::vec3 up;
glm::vec3 center;
glm::vec3 eye;

glm::mat4 view;
glm::mat4 model;

// Used for time based animation.
float time_last = 0;

// Used to rotate the isosurface.
float rotation_radians = 0.0;
float rotation_radians_step = 0.2 * 180 / M_PI;

unsigned int vao[1];
unsigned int gbo[2];

const int nPoint = 192741;
const int target_cluster = 2;
vector<float> position;
vector<float> concentration;
vector<int> cluster;

const float boarder = 0.01f;
const int scale = 128;
float xmin = 1.0f, xmax = -1.0f, ymin = 1.0f, ymax = -1.0f, zmin = 1.0f, zmax = -1.0f;
int w, h, d;

kdtree::StaticKdTree3d<point_type::Point3f, point_cloud::PointCloud> kd_tree_3d;
vector<int> indexToPointID;

void loadPointsFromNpy(const string& file_raw, const string& file_cluster) {
	cnpy::NpyArray arr = cnpy::npy_load(file_raw);
	float* loaded = arr.data<float>();

	position.resize(nPoint * 3);
	concentration.resize(nPoint);
	for (int i = 0; i < nPoint; i++) {
		position[i * 3] = loaded[i * 7] / 10.0f;
		position[i * 3 + 1] = loaded[i * 7 + 1] / 10.0f;
		position[i * 3 + 2] = (loaded[i * 7 + 2] - 5.0f) / 10.0f;
		concentration[i] = loaded[i * 7 + 3];
	}

	cnpy::NpyArray arr_cluster = cnpy::npy_load(file_cluster);
	cluster.resize(nPoint);
	int* loaded_cluster = arr_cluster.data<int>();
	for (int i = 0; i < nPoint; i++)
		cluster[i] = loaded_cluster[i];

	for (int i = 0; i < nPoint; i++) {
		if (cluster[i] == target_cluster) {
			if (position[i * 3] < xmin)
				xmin = position[i * 3];
			if (position[i * 3] > xmax)
				xmax = position[i * 3];
			if (position[i * 3 + 1] < ymin)
				ymin = position[i * 3 + 1];
			if (position[i * 3 + 1] > ymax)
				ymax = position[i * 3 + 1];
			if (position[i * 3 + 2] < zmin)
				zmin = position[i * 3 + 2];
			if (position[i * 3 + 2] > zmax)
				zmax = position[i * 3 + 2];
		}
	}
	xmin -= boarder;
	xmax += boarder;
	ymin -= boarder;
	ymax += boarder;
	zmin -= boarder;
	zmax += boarder;

	// build KD-tree
	for (int i = 0; i < nPoint; i++) {
		float xPos = position[i * 3];
		float yPos = position[i * 3 + 1];
		float zPos = position[i * 3 + 2];
		if ((xPos > xmin) && (xPos < xmax) && (yPos > ymin) && (yPos < ymax) && (zPos > zmin) && (zPos < zmax)){
			kd_tree_3d.add(point_type::Point3f(xPos, yPos, zPos));
		}
	}
	kd_tree_3d.build();
}

void initVol() {
	float t = pow((float)(scale * scale * scale) / (xmax - xmin) / (ymax - ymin) / (zmax - zmin), 1.0f / 3.0f);
	w = (int)(t * (xmax - xmin) + 0.5);
	h = (int)(t * (ymax - ymin) + 0.5);
	d = (int)(t * (zmax - zmin) + 0.5);

	point_type::Point3f point_to_search(0.0f, 0.0f, 0.0f);
	vector<int> indices;
	vector<float> squared_distances;

	int num_3d = kd_tree_3d.radiusSearch(point_to_search, 0.05f, indices, squared_distances);
	std::cout << "Find: " << num_3d << " points," << std::endl;
	for (int i = 0; i < num_3d; i++) {
		std::cout << "Point " << i << ": " << kd_tree_3d[indices[i]];
		std::cout << ", Distance: " << sqrt(squared_distances[i]) << std::endl;
	}
}

void initBuffers() {
	glGenVertexArrays(1, vao);
	glGenBuffers(2, gbo);
	// bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
	glBindVertexArray(vao[0]);

	unsigned int positionDat = gbo[0];
	unsigned int clusterDat = gbo[1];

	glBindBuffer(GL_ARRAY_BUFFER, positionDat);
	glBufferData(GL_ARRAY_BUFFER, nPoint * sizeof(float) * 3, &position[0], GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, clusterDat);
	glBufferData(GL_ARRAY_BUFFER, nPoint * sizeof(int), &cluster[0], GL_STATIC_DRAW);
	glVertexAttribIPointer(1, 1, GL_INT, sizeof(int), 0);
	glEnableVertexAttribArray(1);

	// You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
	// VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
	glBindVertexArray(0);
}

int main()
{
	loadPointsFromNpy("run41_025.npy", "run41_025_cluster.npy");
	initVol();

	// glfw: initialize and configure
	// ------------------------------
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // uncomment this statement to fix compilation on OS X
#endif

	// glfw window creation
	// --------------------
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	// glad: load all OpenGL function pointers
	// ---------------------------------------
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	// configure global opengl state
	// -----------------------------
	glEnable(GL_DEPTH_TEST);

	// build and compile our shader program
	// ------------------------------------
	Shader ourShader("shader.vs", "shader.fs"); // you can name your shader files however you like

	initBuffers();

	// render loop
	// -----------
	while (!glfwWindowShouldClose(window))
	{
		float time_now = glfwGetTime();
		if (time_last != 0) {
			float time_delta = (time_now - time_last);

			rotation_radians += rotation_radians_step * time_delta;
			if (rotation_radians > 360)
				rotation_radians = 0.0;
		}
		time_last = time_now;
		
		// create the projection matrix 
		float near = 0.1f;
		float far = 5.0f;
		float fov_r = 30.0f;

		pMatrix = glm::perspective(fov_r, (float)SCR_WIDTH / (float)SCR_HEIGHT, near, far);

		// Move to the 3D space origin.
		mvMatrix = glm::mat4(1.0f);

		// transform 
		theta = M_PI / 2;
		phi = 0.0f;
		direction = glm::vec3(sin(theta) * cos(phi) * dist, sin(theta) * sin(phi) * dist, cos(theta) * dist);
		up = glm::vec3(sin(theta - M_PI / 2) * cos(phi), sin(theta - M_PI / 2) * sin(phi), cos(theta - M_PI / 2));
		center = glm::vec3(0.0f, 0.0f, 0.0f);
		eye = center + direction;

		view = glm::lookAt(eye, center, up);
		model = glm::mat4(1.0f);
		model *= glm::rotate(rotation_radians, glm::vec3(0.0f, 0.0f, 1.0f));

		mvMatrix = view * model;
		
		// input
		// -----
		processInput(window);

		// render
		// ------
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		ourShader.use();

		// Pass the vertex shader the projection matrix and the model-view matrix.
		ourShader.setMat4("uMVMatrix", mvMatrix);
		ourShader.setMat4("uPMatrix", pMatrix);

		glBindVertexArray(vao[0]);
		glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
		glDrawArrays(GL_POINTS, 0, nPoint);

		// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
		// -------------------------------------------------------------------------------
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	// optional: de-allocate all resources once they've outlived their purpose:
	// ------------------------------------------------------------------------
	glDeleteVertexArrays(1, vao);
	glDeleteBuffers(1, gbo);

	// glfw: terminate, clearing all previously allocated GLFW resources.
	// ------------------------------------------------------------------
	glfwTerminate();
	return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}
