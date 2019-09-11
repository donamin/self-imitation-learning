#pragma once
#include "OdeRig.h"

class Mech : public OdeRig
{
public:
	enum Bones
	{
		Pelvis = 0
		, Spine
		, LUpperLeg
		, LLowerLeg
		, LFoot
		, RUpperLeg
		, RLowerLeg
		, RFoot
		, bCount
	};

	void init(bool fixInAir = false)
	{
		enableSelfCollisions = true;

		bones.resize(bCount, NULL);
		angleSd.resize(bCount, 10);

		bones[Pelvis] = createBone(0.0000, -0.2376, 1.4009, 0.3000, 0.1000, Vector3f(0.0000, 0.0000, 1.0000));
		bones[Spine] = createBone(0.0000, -0.2376, 2.0509, 0.9000, 0.1000, Vector3f(0.0000, 0.0000, -1.0000));
		bones[LUpperLeg] = createBone(-0.3548, 0.0356, 1.2129, 0.3000, 0.1000, Vector3f(-0.0017, -0.7782, 0.6281));
		bones[LLowerLeg] = createBone(-0.3516, -0.0326, 0.8493, 0.4500, 0.0750, Vector3f(-0.0089, 0.4958, 0.8684));
		bones[LFoot] = createBone(-0.3510, -0.2318, 0.5599, 0.4000, 0.0500, Vector3f(0.0008, 0.9993, 0.0365));
		bones[RUpperLeg] = createBone(0.3548, 0.0361, 1.2135, 0.3000, 0.1000, Vector3f(0.0016, -0.7796, 0.6263));
		bones[RLowerLeg] = createBone(0.3517, -0.0302, 0.8491, 0.4500, 0.0750, Vector3f(0.0088, 0.4905, 0.8714));
		bones[RFoot] = createBone(0.3509, -0.2292, 0.5597, 0.4000, 0.0500, Vector3f(0.0003, 1.0000, -0.0002));

		angleSd[Pelvis] = deg2rad * 5;
		angleSd[Spine] = deg2rad * 5;
		angleSd[LUpperLeg] = deg2rad * 20;
		angleSd[LLowerLeg] = deg2rad * 20;
		angleSd[LFoot] = deg2rad * 10;
		angleSd[RUpperLeg] = deg2rad * 20;
		angleSd[RLowerLeg] = deg2rad * 20;
		angleSd[RFoot] = deg2rad * 10;

		createHinge(Pelvis, Spine, getBoneEndPos(bones[Spine]), Vector3f(0.0000, 0.0000, -1.0000), -deg2rad * 30.0f, deg2rad * 30.0f);
		createHinge(Pelvis, LUpperLeg, getBoneEndPos(bones[LUpperLeg]), Vector3f(1.0000, 0.0024, 0.0057), -deg2rad * 45.0f, deg2rad * 45.0f);
		createHinge(LUpperLeg, LLowerLeg, getBoneEndPos(bones[LLowerLeg]), Vector3f(0.9999, 0.0135, 0.0025), -deg2rad * 45.0f, deg2rad * 45.0f);
		createHinge(LLowerLeg, LFoot, getBoneEndPos(bones[LFoot]), Vector3f(1.0000, 0.0036, -0.0028), -deg2rad * 30.0f, deg2rad * 10.0f);
		createHinge(Pelvis, RUpperLeg, getBoneEndPos(bones[RUpperLeg]), Vector3f(1.0000, -0.0025, -0.0057), -deg2rad * 45.0f, deg2rad * 45.0f);
		createHinge(RUpperLeg, RLowerLeg, getBoneEndPos(bones[RLowerLeg]), Vector3f(0.9999, -0.0136, -0.0025), -deg2rad * 45.0f, deg2rad * 45.0f);
		createHinge(RLowerLeg, RFoot, getBoneEndPos(bones[RFoot]), Vector3f(1.0000, 0.0018, 0.0043), -deg2rad * 30.0f, deg2rad * 10.0f);

		if (fixInAir)
		{
			int joint = odeJointCreateFixed();
			odeJointAttach(joint, 0, bones[0]->body);
			odeJointSetFixed(joint);
		}
		genericInit(fixInAir ? 0.1f : 30.0f);
		commandLogFileName = "CommandsLog-Mech.txt";
	}
};