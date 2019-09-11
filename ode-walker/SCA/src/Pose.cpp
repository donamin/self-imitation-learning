#include "Pose.h"

void LoadPoseFromFile(const OdeRig *character, const char *fileName, Pose &pose)
{
	FILE *f = fopen(fileName, "r");
	if (f == NULL)
	{
		printf("Unable to load %s\n", fileName);
		return;
	}
	float a0, a1, a2, a3;
	pose.angles = VectorXf::Zero(character->controlDim);
	int counter = 0;
	for (int i = 0; i < character->joints.size(); ++i)
	{
		OdeRig::Joint *j = character->joints[i];
		if (j->motor <= 0)
		{
			//Hinge
			fscanf(f, "%f", &a0);
			pose.angles[counter++] = a0;
		}
		else
		{
			//AMotor
			fscanf(f, "%f %f %f", &a0, &a1, &a2);
			pose.angles[counter++] = a0;
			pose.angles[counter++] = a1;
			pose.angles[counter++] = a2;
		}
	}
	for (int i = 0; i < character->bones.size(); ++i)
	{
		fscanf(f, "%f %f %f %f", &a0, &a1, &a2, &a3);
		pose.rotations[i] = Quaternionf(a0, a1, a2, a3);
	}
	fclose(f);
}

void SavePoseToFile(const OdeRig *character, const char *fileName)
{
	FILE *f = fopen(fileName, "w+");
	for (int i = 0; i < character->joints.size(); ++i)
	{
		OdeRig::Joint *j = character->joints[i];
		if (j->motor <= 0)
		{
			//Hinge
			fprintf(f, "%.4f\n", odeJointGetHingeAngle(j->joint));
		}
		else
		{
			//AMotor
			fprintf(f, "%.4f %.4f %.4f\n", odeJointGetAMotorAngle(j->motor, 0), odeJointGetAMotorAngle(j->motor, 1), odeJointGetAMotorAngle(j->motor, 2));
		}
	}
	for (int i = 0; i < character->bones.size(); ++i)
	{
		Quaternionf rot = ode2eigenq(odeBodyGetQuaternion(character->bones[i]->body));
		fprintf(f, "%.4f %.4f %.4f %.4f\n", rot.w(), rot.x(), rot.y(), rot.z());
	}
	fclose(f);
}

Quaternionf ComputeTargetRotation(const OdeRig *character, const Vector3f facingTarget)
{
	Vector3f root_pos(odeBodyGetPosition(character->bones[0]->body));
	Vector3f root_to_target = facingTarget - root_pos;
	root_to_target.z() = 0.0f;
	root_to_target.normalize();
	//(1, 0, 0) is the initial forward vector in pose editor
	Quaternionf rotation = Quaternionf::FromTwoVectors(Vector3f(1, 0, 0), root_to_target);
	return rotation;
}

/*
//Get deviation from character's initial pose
float GetPoseDeviationCost(const OdeRig *character, const Vector3f facingTarget, const float poseSd)
{
	float cost = 0, angleDiff;
	const Quaternionf rootTargetRotation = ComputeTargetRotation(character, facingTarget);
	Quaternionf boneTargetRot, currentRot;
	for (int i = 0; i < character->bones.size(); ++i)
	{
		boneTargetRot = rootTargetRotation * character->initRotations[i];
		currentRot = ode2eigenq(odeBodyGetQuaternion(character->bones[i]->body)) * character->initForwardRotator;
		angleDiff = currentRot.angularDistance(boneTargetRot);
		cost += squared(angleDiff / poseSd);
	}
	return cost;
}

//Get deviation from given pose
float GetPoseDeviationCost(const OdeRig *character, const Vector3f facingTarget, const Pose &pose, const float poseSd, const int relaxHand)
{
	float cost = 0, angleDiff;
	const Quaternionf rootTargetRotation = ComputeTargetRotation(character, facingTarget);
	Quaternionf boneTargetRot, currentRot;
	float sd;
	for (int i = 0; i < character->bones.size(); ++i)
	{
		boneTargetRot = rootTargetRotation * pose.rotations[i];
		currentRot = ode2eigenq(odeBodyGetQuaternion(character->bones[i]->body))
			* Quaternionf::FromTwoVectors(Vector3f(1, 0, 0), character->initForward).inverse();
		angleDiff = currentRot.angularDistance(boneTargetRot);
		sd = poseSd;
		switch (i)
		{
			case 0: //bPelvis
				if (relaxHand != 0) sd *= 3.0f;
				break;
			case 1: //bSpine
				if (relaxHand != 0) sd *= 3.0f;
				break;
			case 2: //bLeftUpperArm
			case 3: //bLeftForeArm
			case 4: //bLeftHand
				if (relaxHand == -1) sd *= 5.0f;
				break;
			case 5: //bRightUpperArm
			case 6: //bRightForeArm
			case 7: //bRightHand
				if(relaxHand == 1) sd *= 5.0f;
				break;
			case 8: //bHead
				sd /= 2.0f;
				break;
		}
		cost += squared(angleDiff / sd);
	}
	return cost;
}
*/

PoseEditor::PoseEditor(OdeRig *character)
{
	frameIdx = 0;
	this->character = character;
	lastInputFrame = 0;
	maxSpeed = 2 * PI;
	desiredBoneAngles = VectorXf::Zero(character->controlDim);
	selectedDof = 0;
	selectedBone = 0;
	if (character->joints[0]->motor <= 0)
	{
		//Hinge joint
		selectedAxis = -1;
	}
	else
	{
		//AMotor joint
		selectedAxis = 0;
	}
}

PoseEditor::~PoseEditor()
{
}

void PoseEditor::Update(int frameIdx)
{
	if (frameIdx - lastInputFrame > 5)
	{
		if (GetAsyncKeyState(VK_UP))
		{
			SelectNextBone();
			lastInputFrame = frameIdx;
		}
		else if (GetAsyncKeyState(VK_DOWN))
		{
			SelectPreviousBone();
			lastInputFrame = frameIdx;
		}
		else if (GetAsyncKeyState(VK_SPACE))
		{
			SwitchAxis();
			lastInputFrame = frameIdx;
		}
		else if (GetAsyncKeyState(VK_RIGHT))
		{
			if (selectedBone != 0)
			{
				desiredBoneAngles[selectedDof] += 0.1;
			}
			lastInputFrame = frameIdx;
		}
		else if (GetAsyncKeyState(VK_LEFT))
		{
			if (selectedBone != 0)
			{
				desiredBoneAngles[selectedDof] -= 0.1;
			}
			lastInputFrame = frameIdx;
		}
		else if (GetAsyncKeyState(VK_RETURN))
		{
			if (selectedBone != 0)
			{
				desiredBoneAngles[selectedDof] = 0;
			}
			lastInputFrame = frameIdx;
		}
		else if (GetAsyncKeyState(VK_HOME))
		{
			for (int i = 0; i < desiredBoneAngles.size(); ++i)
				desiredBoneAngles[i] = 0;
			lastInputFrame = frameIdx;
		}
		else if (GetAsyncKeyState(0x53)) //S
		{
			//SavePoseToFile(character, ("MartialArts_" + character->namePrefix + "_PoseEditor.txt").c_str());
			lastInputFrame = frameIdx;
		}
		else if (GetAsyncKeyState(0x4C)) //L
		{
			Pose pose;
			//LoadPoseFromFile(character, ("MartialArts_" + character->namePrefix + "_PoseEditor.txt").c_str(), pose);
			desiredBoneAngles = pose.angles;
			lastInputFrame = frameIdx;
		}
	}
	for (int i = 0; i < character->bones.size(); ++i)
	{
		rcSetColor(1, 1, 1, 1);
		if (i == selectedBone)
		{
			if (selectedBone == character->bones.size())
			{
				rcSetColor(0.5, 0.5, 0.5, 1);
			}
			else
			{
				switch (selectedAxis)
				{
					case -1:
					case 0:
						rcSetColor(0.8, 0, 0, 1);
						break;
					case 1:
						rcSetColor(0, 0.8, 0, 1);
						break;
					case 2:
						rcSetColor(0, 0, 0.8, 1);
						break;
				}
			}
		}
		float radius, length;
		odeGeomCapsuleGetParams(character->bones[i]->geom, radius, length);
		rcDrawCapsule(odeBodyGetPosition(character->bones[i]->body), odeBodyGetRotation(character->bones[i]->body), length, radius);
	}
	UpdateJointsAngles();
	Print(frameIdx);
	Vector3f com;
	character->computeCOM(com);

	Vector3f meanFeet = Vector3f::Zero();
	if (character->numberOfLegs() == 2)
	{
		float footy[2];
		for (int i = 0; i < 2; ++i)
		{
			Vector3f footPos = character->getFootPos(i);
			footy[i] = footPos.z();
			meanFeet += footPos / 2.0;

		}
		meanFeet.z() = max(footy[0], footy[1]);
	}


	float pos1[3], pos2[3];

	pos1[0] = com.x();
	pos1[1] = com.y();
	pos1[2] = com.z();

	pos2[0] = meanFeet.x();
	pos2[1] = meanFeet.y();
	pos2[2] = meanFeet.z();

	rcSetColor(1, 1, 1, 1);
	rcDrawLine(pos1, pos2);

	character->debugVisualize();
}

void PoseEditor::SelectNextBone()
{
	selectedBone--;
	if (selectedBone < 0)
		selectedBone = character->bones.size() - 1;
	ScanAxis();
}

void PoseEditor::SelectPreviousBone()
{
	selectedBone = (selectedBone + 1) % character->bones.size();
	ScanAxis();
}

void PoseEditor::ScanAxis()
{
	OdeRig::Joint *joint = character->bones[selectedBone]->joint;
	if (joint == NULL)
	{
		//Pelvis
		selectedDof = selectedAxis = -1;
	}
	else
	{
		if (joint->motor <= 0)
		{
			//Hinge joint
			selectedAxis = -1;
		}
		else
		{
			//AMotor joint
			selectedAxis = 0;
		}
		for (selectedDof = 0; selectedDof < character->controlDim; ++selectedDof)
			if (character->dof2BoneMapping[selectedDof] == selectedBone)
				break;
	}
}

void PoseEditor::SwitchAxis()
{
	if (selectedAxis == -1)
		return;
	if (selectedAxis < 2)
	{
		++selectedAxis;
		++selectedDof;
	}
	else
	{
		selectedAxis = 0;
		selectedDof -= 2;
	}
}

void PoseEditor::UpdateJointsAngles()
{
	int counter = 0;
	for (int j = 0; j < character->joints.size(); ++j)
	{
		OdeRig::Joint *joint = character->joints[j];
		if (joint->motor <= 0)
		{
			//Hinge
			float curAngle = odeJointGetHingeAngle(joint->joint);
			float speed = desiredBoneAngles[counter++] - curAngle;
			if (fabs(speed) > maxSpeed)
			{
				speed = fsign(speed) * maxSpeed;
			}
			odeJointSetHingeParam(joint->joint, dParamVel1, speed);
		}
		else
		{
			//Motor
			for (int i = 0; i < 3; ++i)
			{
				float curAngle = odeJointGetAMotorAngle(joint->motor, i);
				float speed = desiredBoneAngles[counter++] - curAngle;
				if (fabs(speed) > maxSpeed)
				{
					speed = fsign(speed) * maxSpeed;
				}
				switch (i)
				{
				case 0:
					odeJointSetAMotorParam(joint->motor, dParamVel1, speed);
					break;
				case 1:
					odeJointSetAMotorParam(joint->motor, dParamVel2, speed);
					break;
				case 2:
					odeJointSetAMotorParam(joint->motor, dParamVel3, speed);
					break;
				}
			}
		}
	}
	frameIdx++;
}

void PoseEditor::Print(int frameIdx)
{
	OdeRig::Joint *joint = character->bones[selectedBone]->joint;
	if (joint == NULL)
	{
		rcPrintString("Cannot rotate bone!");
	}
	else
	{
		if (selectedAxis == -1)
		{
			float curAngle = odeJointGetHingeAngle(joint->joint);
			rcPrintString("Hinge joint %d", selectedBone);
			rcPrintString("Current angle = %.2f, desired angle = %.2f", curAngle, desiredBoneAngles[selectedDof]);
		}
		else
		{
			float curAngle = odeJointGetAMotorAngle(joint->motor, selectedAxis);
			rcPrintString("AMotor joint %d, angle %d", selectedBone, selectedAxis);
			rcPrintString("Current angle = %.2f, desired angle = %.2f", curAngle, desiredBoneAngles[selectedDof]);
		}
	}
}