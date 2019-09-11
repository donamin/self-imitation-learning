#pragma once
#ifndef EIGEN_INCLUDED
#define EIGEN_INCLUDED
#include <Eigen/Geometry>
//#include <Eigen/StdVector>
using namespace Eigen;
//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#endif // !EIGEN_INCLUDED

#ifndef STD_INCLUDED
#define STD_INCLUDED
#include <vector>
#include <map>
#include <queue>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <string>
#include <regex>
#include <memory>
#include <chrono>
#include <future>
using namespace std;
using namespace std::chrono;
#endif // !STD_INCLUDED

#include "RenderClient.h"
#include "RenderCommands.h"
#include "UnityOde.h"
#include "UnityOde_internal.h"

#include "mathutils.h"
#include "SCAController.h"
#include "FileUtils.h"

#include <windows.h>

#include "OdeRig.h"

using namespace AaltoGames;

struct Pose
{
	VectorXf angles;
	Quaternionf rotations[MAX_BONES];
};

void LoadPoseFromFile(const OdeRig *character, const char *fileName, Pose &pose);
void SavePoseToFile(const OdeRig *character, const char *fileName);
Quaternionf ComputeTargetRotation(const OdeRig *character, const Vector3f facingTarget);
//float GetPoseDeviationCost(const OdeRig *character, const Vector3f facingTarget, const float poseSd);
//float GetPoseDeviationCost(const OdeRig *character, const Vector3f facingTarget, const Pose &pose, const float poseSd, const int relaxHand);

class PoseEditor
{
public:
	PoseEditor(OdeRig *character);
	~PoseEditor();
	void Update(int frameIdx);

private:
	void SelectNextBone();
	void SelectPreviousBone();
	void ScanAxis();
	void SwitchAxis();
	void UpdateJointsAngles();
	void Print(int frameIdx);

	OdeRig *character;
	int lastInputFrame;
	int selectedDof, selectedBone, selectedAxis;
	VectorXf desiredBoneAngles;
	float maxSpeed;
	int frameIdx;
};