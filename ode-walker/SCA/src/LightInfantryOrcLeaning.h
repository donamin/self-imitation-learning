#pragma once
#include "OdeRig.h"

class LightInfantryOrcLeaning : public OdeRig
{
public:
	enum Bones
	{
		Pelvis = 0
		, LThigh
		, LCalf
		, LFoot
		, RThigh
		, RCalf
		, RFoot
		, Spine
		, LUpperArm
		, LForearm
		, Head
		, RUpperArm
		, RForearm
		, bCount
	};

	void init(bool fixInAir = false)
	{
		enableSelfCollisions = true;

		bones.resize(bCount, NULL);
		angleSd.resize(bCount, deg2rad * 10);

		bones[Pelvis] = createBone(0.0189, 0.0854, 1.1430, 0.3500, 0.1250, Vector3f(-0.0078, -0.0606, -0.9981));
		bones[LThigh] = createBone(-0.1439, -0.0694, 0.9097, 0.2250, 0.0750, Vector3f(0.2176, 0.4798, 0.8499));
		bones[LCalf] = createBone(-0.1794, -0.0567, 0.7100, 0.2250, 0.0750, Vector3f(-0.0072, -0.5836, 0.8120));
		bones[LFoot] = createBone(-0.1827, -0.0807, 0.5544, 0.3000, 0.0550, Vector3f(-0.0436, -0.9991, -0.0007));
		bones[RThigh] = createBone(0.2001, 0.0136, 0.8911, 0.2250, 0.0750, Vector3f(-0.2822, 0.3250, 0.9026));
		bones[RCalf] = createBone(0.2253, 0.0601, 0.6968, 0.2250, 0.0750, Vector3f(0.1534, -0.6861, 0.7112));
		bones[RFoot] = createBone(0.2297, 0.0523, 0.5544, 0.3000, 0.0550, Vector3f(0.2445, -0.9696, -0.0001));
		bones[Spine] = createBone(0.0215, 0.0025, 1.4790, 0.3500, 0.1250, Vector3f(-0.0163, 0.3923, -0.9197));
		bones[LUpperArm] = createBone(-0.3084, 0.0180, 1.3720, 0.3750, 0.0500, Vector3f(0.4998, -0.3552, 0.7899));
		bones[LForearm] = createBone(-0.3632, -0.1409, 1.1825, 0.3500, 0.0500, Vector3f(-0.1014, 0.9485, 0.3001));
		bones[Head] = createBone(0.0413, -0.1358, 1.7428, 0.2500, 0.0750, Vector3f(0.0129, 0.1011, -0.9948));
		bones[RUpperArm] = createBone(0.3279, 0.1350, 1.4235, 0.3750, 0.0500, Vector3f(-0.3839, -0.4441, 0.8095));
		bones[RForearm] = createBone(0.4577, 0.1079, 1.1027, 0.3500, 0.0500, Vector3f(-0.3040, 0.4293, 0.8505));

		angleSd[Pelvis] = deg2rad * 5;
		angleSd[LThigh] = deg2rad * 15;
		angleSd[LCalf] = deg2rad * 15;
		angleSd[LFoot] = deg2rad * 15;
		angleSd[RThigh] = deg2rad * 15;
		angleSd[RCalf] = deg2rad * 15;
		angleSd[RFoot] = deg2rad * 15;
		angleSd[Spine] = deg2rad * 5;
		angleSd[LUpperArm] = deg2rad * 10;
		angleSd[LForearm] = deg2rad * 10;
		angleSd[Head] = deg2rad * 2.5f;
		angleSd[RUpperArm] = deg2rad * 10;
		angleSd[RForearm] = deg2rad * 10;

		createMotoredBallJoint(Pelvis, Spine, getBoneEndPos(bones[Spine]), Vector3f(-0.9866, -0.1554, -0.0488), Vector3f(0.1621, -0.9066, -0.3896), deg2rad * Vector3f(-20.0f, -45.0f, -20.0f), deg2rad * Vector3f(40.0f, 45.0f, 20.0f));
		createMotoredBallJoint(Spine, Head, getBoneEndPos(bones[Head]), Vector3f(-0.9972, 0.0748, -0.0054), Vector3f(-0.0739, -0.9921, -0.1018), deg2rad * Vector3f(-30.0f, -30.0f, -45.0f), deg2rad * Vector3f(45.0f, 30.0f, 45.0f));
		createMotoredBallJoint(Spine, LUpperArm, getBoneEndPos(bones[LUpperArm]), Vector3f(-0.8365, -0.4346, 0.3338), Vector3f(-0.2247, 0.8276, 0.5144), deg2rad * Vector3f(-90.0f, -45.0f, -90.0f), deg2rad * Vector3f(45.0f, 45.0f, 30.0f));
		createHinge(LUpperArm, LForearm, getBoneEndPos(bones[LForearm]), Vector3f(-0.8487, -0.2399, 0.4714), -deg2rad * 45.0f, deg2rad * 45.0f);
		createMotoredBallJoint(Spine, RUpperArm, getBoneEndPos(bones[RUpperArm]), Vector3f(-0.9229, 0.2108, -0.3221), Vector3f(0.0276, 0.8708, 0.4908), deg2rad * Vector3f(-90.0f, -45.0f, -30.0f), deg2rad * Vector3f(45.0f, 45.0f, 90.0f));
		createHinge(RUpperArm, RForearm, getBoneEndPos(bones[RForearm]), Vector3f(-0.9212, 0.0952, -0.3774), -deg2rad * 45.0f, deg2rad * 45.0f);
		createMotoredBallJoint(Pelvis, LThigh, getBoneEndPos(bones[LThigh]), Vector3f(0.9753, -0.1394, -0.1711), Vector3f(-0.0364, -0.8662, 0.4983), deg2rad * Vector3f(-45.0f, -30.0f, -15.0f), deg2rad * Vector3f(90.0f, 30.0f, 45.0f));
		createHinge(LThigh, LCalf, getBoneEndPos(bones[LCalf]), Vector3f(0.9907, -0.1145, -0.0735), -deg2rad * 45.0f, deg2rad * 45.0f);
		createHinge(LCalf, LFoot, getBoneStartPos(bones[LFoot]), Vector3f(0.9991, -0.0436, -0.0003), -deg2rad * 25.0f, deg2rad * 15.0f);
		createMotoredBallJoint(Pelvis, RThigh, getBoneEndPos(bones[RThigh]), Vector3f(0.9101, 0.3882, 0.1447), Vector3f(0.3034, -0.8624, 0.4053), deg2rad * Vector3f(-45.0f, -30.0f, -45.0f), deg2rad * Vector3f(90.0f, 30.0f, 15.0f));
		createHinge(RThigh, RCalf, getBoneEndPos(bones[RCalf]), Vector3f(0.9515, 0.2968, 0.0811), -deg2rad * 45.0f, deg2rad * 45.0f);
		createHinge(RCalf, RFoot, getBoneStartPos(bones[RFoot]), Vector3f(0.9696, 0.2445, 0.0002), -deg2rad * 25.0f, deg2rad * 15.0f);

		if (fixInAir)
		{
			int joint = odeJointCreateFixed();
			odeJointAttach(joint, 0, bones[0]->body);
			odeJointSetFixed(joint);
		}
		genericInit(fixInAir ? 0.1f : 30.0f);
		commandLogFileName = "CommandsLog-LightInfantryOrcLeaning.txt";
	}
};