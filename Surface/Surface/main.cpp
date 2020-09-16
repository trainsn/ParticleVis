#include"cnpy.h"
#include<cstdlib>
#include<iostream>
#include<string>

using namespace std;

const int nPoint = 192741;
vector<float> position;
vector<float> concentration;
int* cluster;

void loadPointsFromNpy(const string& file_raw, const string& file_cluster) {
	cnpy::NpyArray arr = cnpy::npy_load(file_raw);
	float* loaded = arr.data<float>();

	position.resize(nPoint * 3);
	concentration.resize(nPoint);
	for (int i = 0; i < nPoint; i++) {
		position[i * 3] = loaded[i * 7];
		position[i * 3 + 1] = loaded[i * 7 + 1];
		position[i * 3 + 2] = loaded[i * 7 + 2];
		concentration[i] = loaded[i * 7 + 3];
	}

	cnpy::NpyArray arr_cluster = cnpy::npy_load(file_cluster);
	cluster = arr_cluster.data<int>();
}

int main()
{
	loadPointsFromNpy("run41_025.npy", "run41_025_cluster.npy");
}
