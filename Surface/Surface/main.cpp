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
#include "Volume.h"
#include "GridPoint.h"
#include "marching_cubes.h"

#include "static_kd_tree_3d.hpp"
#include "point_cloud.hpp"
#include "point_type.hpp"
using namespace std;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);

// Base color used for the ambient, fog, and clear-to colors.
glm::vec3 base_color(10.0 / 255.0, 10.0 / 255.0, 10.0 / 255.0);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 800;

glm::mat4 pMatrix;
glm::mat4 mvMatrix;
glm::mat3 normalMatrix;

// Information related to the camera 
float dist = 30.0f;
float theta, phi;
glm::vec3 direction;
glm::vec3 up;
glm::vec3 center;
glm::vec3 eye;

glm::mat4 view;
glm::mat4 model;

// Lighting power.
float lighting_power = 0.5f;
float lighting_power1 = 0;

// Used for time based animation.
float time_last = 0;

// Used to rotate the isosurface.
float rotation_radians = 0.0;
float rotation_radians_step = 0.2 * 180 / M_PI;

// Use lighting?
int use_lighting = 1;

// Render different buffers.
int show_depth = 0;
int show_normals = 0;
int show_position = 0;

// Used to orbit the point lights.
float point_light_theta1 = M_PI / 2;
float point_light_phi1 = M_PI / 2;

unsigned int pointVAO[1];
unsigned int pointVBO[2];

const int nPoint = 192741;
const int target_cluster = 3;
vector<float> position;
vector<float> concentration;
vector<int> cluster;

const float boarder = 0.5f;
const int scale = 32;
float xmin = 10.0f, xmax = -10.0f, ymin = 10.0f, ymax = -10.0f, zmin = 10.0f, zmax = -10.0f;
int w, h, d;

kdtree::StaticKdTree3d<point_type::Point3f, point_cloud::PointCloud> kd_tree_3d;
vector<int> indexToPointID;

const float eps = 0.00001;

// Isosurface data structures for holding the vertices, vertex normals, and vertex colors.
vector<float> vertices;
vector<float> vertex_normals;
vector<float> vertex_colors;
vector<float> isosurface_vertices;
vector<float> isosurface_vertex_normals;
vector<float> isosurface_vertex_colors;

// volume information
Volume vol = Volume();

unsigned int vao[1];
unsigned int gbo[3];
unsigned int isosurfaceVertexPositionBuffer;
unsigned int isosurfaceVertexNormalBuffer;
unsigned int isosurfaceVertexColorBuffer;

void loadPointsFromNpy(const string& file_raw, const string& file_cluster) {
	cnpy::NpyArray arr = cnpy::npy_load(file_raw);
	float* loaded = arr.data<float>();

	position.resize(nPoint * 3);
	concentration.resize(nPoint);
	for (int i = 0; i < nPoint; i++) {
		position[i * 3] = loaded[i * 7];
		position[i * 3 + 1] = loaded[i * 7 + 1];
		position[i * 3 + 2] = (loaded[i * 7 + 2] - 5.0f);
		concentration[i] = loaded[i * 7 + 3];
	}

	cnpy::NpyArray arr_cluster = cnpy::npy_load(file_cluster);
	cluster.resize(nPoint);
	int* loaded_cluster = arr_cluster.data<int>();
	for (int i = 0; i < nPoint; i++)
		cluster[i] = loaded_cluster[i];
}

void computeNormals() {
	for (int k = 0; k < d; k++) {
		for (int j = 0; j < h; j++) {
			for (int i = 0; i < w; i++) {
				int idx = k * h * w + j * w + i;
				if (k == 0) {
					vol.grids[idx].normal_z = (vol.grids[idx + h * w].value - vol.grids[idx].value);
				}
				else if (k == d - 1) {
					vol.grids[idx].normal_z = (vol.grids[idx].value - vol.grids[idx - h * w].value);
				}
				else {
					vol.grids[idx].normal_z = 0.5 * (vol.grids[idx + h * w].value - vol.grids[idx - h * w].value);
				}

				if (j == 0) {
					vol.grids[idx].normal_y = (vol.grids[idx + w].value - vol.grids[idx].value);
				}
				else if (j == h - 1) {
					vol.grids[idx].normal_y = (vol.grids[idx].value - vol.grids[idx - w].value);
				}
				else {
					vol.grids[idx].normal_y = 0.5 * (vol.grids[idx + w].value - vol.grids[idx - w].value);
				}

				if (i == 0) {
					vol.grids[idx].normal_x = (vol.grids[idx + 1].value - vol.grids[idx].value);
				}
				else if (i == w - 1) {
					vol.grids[idx].normal_x = (vol.grids[idx].value - vol.grids[idx - 1].value);
				}
				else {
					vol.grids[idx].normal_x = 0.5 * (vol.grids[idx + 1].value - vol.grids[idx - 1].value);
				}
			}
		}
	}
}

void computeVol() {
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
		if ((xPos > xmin) && (xPos < xmax) && (yPos > ymin) && (yPos < ymax) && (zPos > zmin) && (zPos < zmax)) {
			kd_tree_3d.add(point_type::Point3f(xPos, yPos, zPos));
			indexToPointID.push_back(i);
		}
	}
	kd_tree_3d.build();

	float t = pow((float)(scale * scale * scale) / (xmax - xmin) / (ymax - ymin) / (zmax - zmin), 1.0f / 3.0f);
	w = (int)(t * (xmax - xmin) + 0.5);
	h = (int)(t * (ymax - ymin) + 0.5);
	d = (int)(t * (zmax - zmin) + 0.5);

	vol = Volume(w, h, d);

	for (int k = 0; k < d; k++) {
		for (int j = 0; j < h; j++) {
			for (int i = 0; i < w; i++) {
				int idx = k * h * w + j * w + i;
				float x = xmin + i * (xmax - xmin) / (w - 1);
				float y = ymin + j * (ymax - ymin) / (h - 1);
				float z = zmin + k * (zmax - zmin) / (d - 1);
				point_type::Point3f point_to_search(x, y, z);
				vector<int> indices;
				vector<float> squared_distances;

				int num_3d = kd_tree_3d.radiusSearch(point_to_search, 0.25f, indices, squared_distances);
				float value;
				if (num_3d == 0)
					value = -1.0f;
				else {
					// Inverse Distance Weighting (IDW) interpolation
					value = 0;
					float total_invere_dis = 0;
					for (int t = 0; t < num_3d; t++) {
						if (cluster[indexToPointID[indices[t]]] == target_cluster)
							value += 1.0f / squared_distances[t];
						else 
							value -= 1.0f / squared_distances[t];
						total_invere_dis += 1.0f / squared_distances[t];
					}
					value /= total_invere_dis;
				}
				
				vol.grids[idx] = GridPoint(x, y, z, value);
				
				//std::cout << "Find: " << num_3d << " points," << std::endl;
				//for (int t = 0; t < num_3d; t++) {
				//	std::cout << "Point " << t << ": " << kd_tree_3d[indices[t]];
				//	std::cout << ", Distance: " << sqrt(squared_distances[t]) << std::endl;
				//}
			}
		}
	}

	computeNormals();

}



void initPointBuffers() {
	glGenVertexArrays(1, pointVAO);
	glGenBuffers(2, pointVBO);
	// bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
	glBindVertexArray(pointVAO[0]);

	unsigned int positionDat = pointVBO[0];
	unsigned int clusterDat = pointVBO[1];

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

// Generates one triangle complete with vertex normals and vertex colors.
void triangle(GridPoint p1, GridPoint p2, GridPoint p3, bool invert_normals) {
	// Push the vertices to this triangle face.
	// Pushing point 3, then 2, and then 1 so that the front face of the triangle
	// points outward from the surface.

	// Push point 1, then 2, and then 3 so that the front front face of the triangle
	// points inward from the surface.
	vertices.push_back(p3.x); vertices.push_back(p3.y); vertices.push_back(p3.z);
	vertices.push_back(p2.x); vertices.push_back(p2.y); vertices.push_back(p2.z);
	vertices.push_back(p1.x); vertices.push_back(p1.y); vertices.push_back(p1.z);

	// Calculate the isosurface gradient at point 1, 2, and 3 of the triangle.
	// These three gradient vectors are the vertex normals of this triangle.
	// This will provide a nice smooth appearance when the lighting is calculated.
	// These three gradient vectors will also be the vertex colors.	

	int invert_normal = 1;
	if (invert_normals)
		invert_normal = -1;

	// Point 3
	float vertex_normal_x = p3.normal_x;
	float vertex_normal_y = p3.normal_y;
	float vertex_normal_z = p3.normal_z;

	float vertex_normal_length = sqrt((vertex_normal_x * vertex_normal_x) + (vertex_normal_y * vertex_normal_y) + (vertex_normal_z * vertex_normal_z));

	if (vertex_normal_length != 0) {
		vertex_normal_x = vertex_normal_x / vertex_normal_length;
		vertex_normal_y = vertex_normal_y / vertex_normal_length;
		vertex_normal_z = vertex_normal_z / vertex_normal_length;
	}

	vertex_normals.push_back(invert_normal * vertex_normal_x);
	vertex_normals.push_back(invert_normal * vertex_normal_y);
	vertex_normals.push_back(invert_normal * vertex_normal_z);

	// Push the vertex colors for this triangle face point.
	vertex_colors.push_back(1.0);
	vertex_colors.push_back(1.0);
	vertex_colors.push_back(0.0);

	// Point 2
	vertex_normal_x = p2.normal_x;
	vertex_normal_y = p2.normal_y;
	vertex_normal_z = p2.normal_z;

	vertex_normal_length = sqrt((vertex_normal_x * vertex_normal_x) + (vertex_normal_y * vertex_normal_y) + (vertex_normal_z * vertex_normal_z));

	if (vertex_normal_length != 0) {
		vertex_normal_x = vertex_normal_x / vertex_normal_length;
		vertex_normal_y = vertex_normal_y / vertex_normal_length;
		vertex_normal_z = vertex_normal_z / vertex_normal_length;
	}

	vertex_normals.push_back(invert_normal * vertex_normal_x);
	vertex_normals.push_back(invert_normal * vertex_normal_y);
	vertex_normals.push_back(invert_normal * vertex_normal_z);

	// Push the vertex colors for this triangle face point.
	vertex_colors.push_back(1.0);
	vertex_colors.push_back(1.0);
	vertex_colors.push_back(0.0);

	// Point 1
	vertex_normal_x = p1.normal_x;
	vertex_normal_y = p1.normal_y;
	vertex_normal_z = p1.normal_z;

	vertex_normal_length = sqrt((vertex_normal_x * vertex_normal_x) + (vertex_normal_y * vertex_normal_y) + (vertex_normal_z * vertex_normal_z));

	if (vertex_normal_length != 0) {
		vertex_normal_x = vertex_normal_x / vertex_normal_length;
		vertex_normal_y = vertex_normal_y / vertex_normal_length;
		vertex_normal_z = vertex_normal_z / vertex_normal_length;
	}

	vertex_normals.push_back(invert_normal * vertex_normal_x);
	vertex_normals.push_back(invert_normal * vertex_normal_y);
	vertex_normals.push_back(invert_normal * vertex_normal_z);

	// Push the vertex colors for this triangle face point.
	vertex_colors.push_back(1.0);
	vertex_colors.push_back(1.0);
	vertex_colors.push_back(0.0);
}

GridPoint edge_intersection_interpolation(GridPoint cube_va, GridPoint cube_vb, float iso_value) {
	if (abs(iso_value - cube_va.value) < eps)
		return cube_va;
	if (abs(iso_value - cube_vb.value) < eps)
		return cube_vb;
	if (abs(cube_vb.value - cube_va.value) < eps)
		return cube_va;

	float mean = (iso_value - cube_va.value) / (cube_vb.value - cube_va.value);
	float x = cube_va.x + mean * (cube_vb.x - cube_va.x);
	float y = cube_va.y + mean * (cube_vb.y - cube_va.y);
	float z = cube_va.z + mean * (cube_vb.z - cube_va.z);

	float normal_x = cube_va.normal_x + mean * (cube_vb.normal_x - cube_va.normal_x);
	float normal_y = cube_va.normal_y + mean * (cube_vb.normal_y - cube_va.normal_y);
	float normal_z = cube_va.normal_z + mean * (cube_vb.normal_z - cube_vb.normal_z);

	return GridPoint(x, y, z, iso_value, normal_x, normal_y, normal_z);
}

// The marching cubes algorithm.
void marching_cubes(int w, int h, int d, float iso_level, bool invert_normals) {
	for (int k = 0; k < d - 1; k++) {
		for (int j = 0; j < h - 1; j++) {
			for (int i = 0; i < w - 1; i++) {
				// Perform the algorithm on one cube in the grid.

				// The cube's vertices.
				// There are eight of them.

				//    4---------5
				//   /|        /|
				//  / |       / |
				// 7---------6  |
				// |  |      |  |
				// |  0------|--1
				// | /       | /
				// |/        |/
				// 3---------2
				int idx = k * h * w + j * w + i;
				GridPoint cube_v3 = vol.grids[idx]; // Lower left  front corner. [k][j][i]
				GridPoint cube_v2 = vol.grids[idx + 1]; // Lower right front corner. [k][j][i + 1]
				GridPoint cube_v6 = vol.grids[idx + w + 1]; // Upper right front corner. [k][j + 1][i + 1]
				GridPoint cube_v7 = vol.grids[idx + w]; // Upper left  front corner. [k][j + 1][i]

				GridPoint cube_v0 = vol.grids[idx + h * w]; // Lower left  back corner. [k + 1][j][i]
				GridPoint cube_v1 = vol.grids[idx + h * w + 1]; // Lower right back corner. [k + 1][j][i + 1]
				GridPoint cube_v5 = vol.grids[idx + h * w + w + 1]; // Upper right back corner. [k + 1][j + 1][i + 1]
				GridPoint cube_v4 = vol.grids[idx + h * w + w]; // Upper left  back corner. [k + 1][j + 1][i]

				int cube_index = 0;

				if (cube_v0.value < iso_level) cube_index |= 1;
				if (cube_v1.value < iso_level) cube_index |= 2;
				if (cube_v2.value < iso_level) cube_index |= 4;
				if (cube_v3.value < iso_level) cube_index |= 8;
				if (cube_v4.value < iso_level) cube_index |= 16;
				if (cube_v5.value < iso_level) cube_index |= 32;
				if (cube_v6.value < iso_level) cube_index |= 64;
				if (cube_v7.value < iso_level) cube_index |= 128;

				// Does the isosurface not intersect any edges of the cube?

				if (marching_cubes_edge_table[cube_index] == 0)
					continue;

				// What edges of the cube does the isosurface intersect?
				// For each cube edge intersected, interpolate an intersection vertex between the edge's incident vertices.
				// These vertices of intersection will form the triangle(s) that approximate the isosurface.

				// There are 12 edges in a cube.

				//       4----5----5
				//    8 /|       6/|
				//     / |9      / | 10
				//    7----7----6  |
				//    |  |      |  |
				// 12 |  0---1--|--1
				//    | /       | / 
				//    |/ 4   11 |/ 2
				//    3----3----2
				//
				// 1={0,1},  2={1,2},  3={2,3},  4={3,0},
				// 5={4,5},  6={5,6},  7={6,7},  8={7,4},
				// 9={0,4}, 10={5,1}, 11={2,6}, 12={3,7}

				// Base ten slot: 2048 | 1024 | 512 | 256 | 128 | 64 | 32 | 16 | 8 | 4 | 2 | 1
				// Base two slot:    0 |    0 |   0 |   0 |   0 |  0 |  0 |  0 | 0 | 0 | 0 | 0
				// Edge slot:       12 |   11 |  10 |   9 |   8 |  7 |  6 |  5 | 4 | 3 | 2 | 1  

				vector<GridPoint> vertices_of_intersection(12, GridPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

				if (marching_cubes_edge_table[cube_index] & 1) // Intersects edge one.
				{
					vertices_of_intersection[0] = edge_intersection_interpolation(cube_v0, cube_v1, iso_level);
				}

				if (marching_cubes_edge_table[cube_index] & 2) // Intersects edge two.
				{
					vertices_of_intersection[1] = edge_intersection_interpolation(cube_v1, cube_v2, iso_level);
				}

				if (marching_cubes_edge_table[cube_index] & 4) // Intersects edge three.
				{
					vertices_of_intersection[2] = edge_intersection_interpolation(cube_v2, cube_v3, iso_level);
				}

				if (marching_cubes_edge_table[cube_index] & 8) // Intersects edge four.
				{
					vertices_of_intersection[3] = edge_intersection_interpolation(cube_v3, cube_v0, iso_level);
				}

				if (marching_cubes_edge_table[cube_index] & 16) // Intersects edge five.
				{
					vertices_of_intersection[4] = edge_intersection_interpolation(cube_v4, cube_v5, iso_level);
				}

				if (marching_cubes_edge_table[cube_index] & 32) // Intersects edge six.
				{
					vertices_of_intersection[5] = edge_intersection_interpolation(cube_v5, cube_v6, iso_level);
				}

				if (marching_cubes_edge_table[cube_index] & 64) // Intersects edge seven.
				{
					vertices_of_intersection[6] = edge_intersection_interpolation(cube_v6, cube_v7, iso_level);
				}

				if (marching_cubes_edge_table[cube_index] & 128) // Intersects edge eight.
				{
					vertices_of_intersection[7] = edge_intersection_interpolation(cube_v7, cube_v4, iso_level);
				}

				if (marching_cubes_edge_table[cube_index] & 256) // Intersects edge nine.
				{
					vertices_of_intersection[8] = edge_intersection_interpolation(cube_v0, cube_v4, iso_level);
				}

				if (marching_cubes_edge_table[cube_index] & 512) // Intersects edge ten.
				{
					vertices_of_intersection[9] = edge_intersection_interpolation(cube_v1, cube_v5, iso_level);
				}

				if (marching_cubes_edge_table[cube_index] & 1024) // Intersects edge eleven.
				{
					vertices_of_intersection[10] = edge_intersection_interpolation(cube_v2, cube_v6, iso_level);
				}

				if (marching_cubes_edge_table[cube_index] & 2048) // Intersects edge twelve.
				{
					vertices_of_intersection[11] = edge_intersection_interpolation(cube_v3, cube_v7, iso_level);
				}

				// Create the triangles.
				// Three vertices make up a triangle per iteration.
				for (int a = 0; marching_cubes_triangle_table[cube_index][a] != -1; a += 3) {
					GridPoint v1 = vertices_of_intersection[marching_cubes_triangle_table[cube_index][a]];
					GridPoint v2 = vertices_of_intersection[marching_cubes_triangle_table[cube_index][a + 1]];
					GridPoint v3 = vertices_of_intersection[marching_cubes_triangle_table[cube_index][a + 2]];

					triangle(v1, v2, v3, invert_normals);
				}
			}
		}
	}
}

void initBuffers() {
	// bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
	glGenVertexArrays(1, vao);
	glGenBuffers(3, gbo);

	glBindVertexArray(vao[0]);

	// Begin creating the isosurfaces.
	isosurfaceVertexPositionBuffer = gbo[0];
	isosurfaceVertexNormalBuffer = gbo[1];
	isosurfaceVertexColorBuffer = gbo[2];

	// Grid min, grid max, resolution, iso-level, and invert normals.
	// Do not set the resolution to small.
	marching_cubes(vol.xSize, vol.ySize, vol.zSize, 0.0, true);
	isosurface_vertices.swap(vertices);
	isosurface_vertex_normals.swap(vertex_normals);
	isosurface_vertex_colors.swap(vertex_colors);

	// Bind and fill the isosurface vertex positions
	glBindBuffer(GL_ARRAY_BUFFER, isosurfaceVertexPositionBuffer);
	glBufferData(GL_ARRAY_BUFFER, isosurface_vertices.size() * sizeof(float), &isosurface_vertices[0], GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray(0);

	// Bind and fill the isosurface vertex normals.
	glBindBuffer(GL_ARRAY_BUFFER, isosurfaceVertexNormalBuffer);
	glBufferData(GL_ARRAY_BUFFER, isosurface_vertex_normals.size() * sizeof(float), &isosurface_vertex_normals[0], GL_STATIC_DRAW);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray(1);

	// Bind and fill the isosurface vertex colors.
	glBindBuffer(GL_ARRAY_BUFFER, isosurfaceVertexColorBuffer);
	glBufferData(GL_ARRAY_BUFFER, isosurface_vertex_colors.size() * sizeof(float), &isosurface_vertex_colors[0], GL_STATIC_DRAW);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray(2);

	glBindVertexArray(0);
}

void drawPoints(Shader ourShader) {
	ourShader.use();

	// Pass the vertex shader the projection matrix and the model-view matrix.
	ourShader.setMat4("uMVMatrix", mvMatrix);
	ourShader.setMat4("uPMatrix", pMatrix);

	glBindVertexArray(pointVAO[0]);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glDrawArrays(GL_POINTS, 0, nPoint);
	glBindVertexArray(0);
}

void drawSurface(Shader ourShader) {
	ourShader.use();
	ourShader.setInt("uPerspectiveProjection", 1);

	ourShader.setInt("uShowDepth", show_depth);
	ourShader.setInt("uShowNormals", show_normals);
	ourShader.setInt("uShowPosition", show_position);

	// Pass the vertex shader the projection matrix and the model-view matrix.
	ourShader.setMat4("uMVMatrix", mvMatrix);
	ourShader.setMat4("uPMatrix", pMatrix);
	// Pass the vertex normal matrix to the shader so it can compute the lighting calculations.
	normalMatrix = glm::transpose(glm::inverse(glm::mat3(mvMatrix)));
	ourShader.setMat3("uNMatrix", normalMatrix);

	// Disable alpha blending.
	glDisable(GL_BLEND);
	if (use_lighting == 1) {
		// Pass the lighting parameters to the fragment shader.
		// Global ambient color. 
		ourShader.setVec3("uAmbientColor", base_color);

		// Point light 1.
		float point_light_dist = 30.0f;
		glm::vec3 point_light_direction = direction / dist * point_light_dist;
		glm::vec3 point_light_position = center + point_light_direction;
		glm::vec4 light_pos(point_light_position.x, point_light_position.y, point_light_position.z, 1.0);
		light_pos = view * light_pos;
		ourShader.setVec3("uPointLightingColor", lighting_power, lighting_power, lighting_power);
		ourShader.setVec3("uPointLightingLocation", light_pos[0], light_pos[1], light_pos[2]);

		// Point light 2.
		float point_light_dist1 = 30.0f;
		float point_light_position_x1 = 0 + point_light_dist1 * cos(point_light_phi1) * sin(point_light_theta1);
		float point_light_position_y1 = 0 + point_light_dist1 * sin(point_light_phi1) * sin(point_light_theta1);
		float point_light_position_z1 = 0 + point_light_dist1 * cos(point_light_theta1);

		glm::vec4 light_pos1(point_light_position_x1, point_light_position_y1, point_light_position_z1, 1.0);
		light_pos1 = view * light_pos1;

		ourShader.setVec3("uPointLightingColor1", lighting_power1, lighting_power1, lighting_power1);
		ourShader.setVec3("uPointLightingLocation1", light_pos1[0], light_pos1[1], light_pos1[2]);

		// Turn off lighting for a moment so that the point light isosurface is 
		// bright simulating that the light is emanating from the surface.
		use_lighting = 0;

		ourShader.setInt("uUseLighting", use_lighting);

		use_lighting = 1;
	}

	ourShader.setInt("uUseLighting", use_lighting);

	glBindVertexArray(vao[0]);
	glDrawArrays(GL_TRIANGLES, 0, isosurface_vertices.size() / 3);
	glBindVertexArray(0);
}

int main()
{
	loadPointsFromNpy("run41_025.npy", "run41_025_cluster.npy");
	computeVol();

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
	Shader pointShader("point.vs", "point.fs"); 
	Shader surfaceShader("surface.vs", "surface.fs");

	initPointBuffers();
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
		float near = 1.0f;
		float far = 50.0f;
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
		
		drawPoints(pointShader);
		drawSurface(surfaceShader);
		
		// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
		// -------------------------------------------------------------------------------
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	// optional: de-allocate all resources once they've outlived their purpose:
	// ------------------------------------------------------------------------
	glDeleteVertexArrays(1, vao);
	glDeleteBuffers(1, gbo);
	glDeleteVertexArrays(1, pointVAO);
	glDeleteBuffers(1, pointVBO);

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
