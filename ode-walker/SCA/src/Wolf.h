#pragma once
#include "OdeRig.h"

class Wolf : public OdeRig
{
public:
	enum Bones
	{
		WolfPelvis = 0
		, WolfLThigh
		, WolfLFoot
		, WolfLToe0
		, WolfRThigh
		, WolfRFoot
		, WolfRToe0
		, WolfSpine
		, WolfLUpperArm
		, WolfLForearm
		, WolfLHand
		, WolfHead
		, WolfRUpperArm
		, WolfRForearm
		, WolfRHand
		, WolfUpperTail
		, WolfLowerTail
		, bCount
	};

	void init(bool fixInAir = false)
	{
		enableSelfCollisions = true;

		bones.resize(bCount, NULL);
		angleSd.resize(bCount, 10);

		bones[WolfPelvis] = createBone(0.0000, 0.3801, 1.4193, 0.5000, 0.2000, Vector3f(0.0000, 0.9987, -0.0514));
		bones[WolfLThigh] = createBone(-0.1599, 0.5348, 1.0694, 0.3000, 0.1000, Vector3f(0.0000, -0.3370, 0.9415));
		bones[WolfLFoot] = createBone(-0.1717, 0.6561, 0.7803, 0.3500, 0.1000, Vector3f(-0.0986, -0.4886, -0.8669));
		bones[WolfLToe0] = createBone(-0.2274, 0.4252, 0.5851, 0.3000, 0.0750, Vector3f(0.1935, 0.9805, -0.0347));
		bones[WolfRThigh] = createBone(0.1599, 0.5348, 1.0694, 0.3000, 0.1000, Vector3f(0.0000, -0.3370, 0.9415));
		bones[WolfRFoot] = createBone(0.1717, 0.6561, 0.7803, 0.3500, 0.1000, Vector3f(0.0986, -0.4886, -0.8669));
		bones[WolfRToe0] = createBone(0.2274, 0.4252, 0.5851, 0.3000, 0.0750, Vector3f(-0.1935, 0.9805, -0.0347));
		bones[WolfSpine] = createBone(0.0000, -0.2485, 1.4341, 0.7000, 0.2000, Vector3f(0.0000, 0.9999, -0.0134));
		bones[WolfLUpperArm] = createBone(-0.1989, -0.2510, 1.0570, 0.3000, 0.1000, Vector3f(-0.0183, -0.2957, 0.9551));
		bones[WolfLForearm] = createBone(-0.1835, -0.2857, 0.7469, 0.3000, 0.1000, Vector3f(-0.0736, 0.5350, 0.8417));
		bones[WolfLHand] = createBone(-0.1938, -0.5397, 0.5663, 0.3000, 0.0800, Vector3f(0.1202, 0.9520, 0.2814));
		bones[WolfHead] = createBone(0.0000, -0.8736, 1.4018, 0.5000, 0.1000, Vector3f(0.0000, 0.9341, 0.3571));
		bones[WolfRUpperArm] = createBone(0.1989, -0.2510, 1.0570, 0.3000, 0.1000, Vector3f(0.0183, -0.2957, 0.9551));
		bones[WolfRForearm] = createBone(0.1835, -0.2857, 0.7469, 0.3000, 0.1000, Vector3f(0.0736, 0.5350, 0.8417));
		bones[WolfRHand] = createBone(0.1938, -0.5397, 0.5663, 0.3000, 0.0800, Vector3f(-0.1202, 0.9520, 0.2814));
		bones[WolfUpperTail] = createBone(0.0000, 0.8354, 1.4419, 0.4500, 0.0500, Vector3f(0.0000, -0.9058, 0.4237));
		bones[WolfLowerTail] = createBone(0.0000, 1.2867, 1.3407, 0.4500, 0.0500, Vector3f(0.0000, -0.9999, -0.0115));

		angleSd[WolfPelvis] = deg2rad * 5;
		angleSd[WolfLThigh] = deg2rad * 15;
		angleSd[WolfLFoot] = deg2rad * 15;
		angleSd[WolfLToe0] = deg2rad * 10;
		angleSd[WolfRThigh] = deg2rad * 15;
		angleSd[WolfRFoot] = deg2rad * 15;
		angleSd[WolfRToe0] = deg2rad * 10;
		angleSd[WolfSpine] = deg2rad * 5;
		angleSd[WolfLUpperArm] = deg2rad * 15;
		angleSd[WolfLForearm] = deg2rad * 15;
		angleSd[WolfLHand] = deg2rad * 10;
		angleSd[WolfHead] = deg2rad * 5;
		angleSd[WolfRUpperArm] = deg2rad * 15;
		angleSd[WolfRForearm] = deg2rad * 15;
		angleSd[WolfRHand] = deg2rad * 10;
		angleSd[WolfUpperTail] = deg2rad * 5;
		angleSd[WolfLowerTail] = deg2rad * 5;

		createMotoredBallJoint(WolfPelvis, WolfLThigh, getBoneEndPos(bones[WolfLThigh]), Vector3f(1.0000, 0.0000, 0.0000), Vector3f(0.0000, -0.9415, -0.3370), deg2rad * Vector3f(-45.0f, -15.0f, -15.0f), deg2rad * Vector3f(45.0f, 15.0f, 15.0f));
		createMotoredBallJoint(WolfPelvis, WolfRThigh, getBoneEndPos(bones[WolfRThigh]), Vector3f(1.0000, 0.0000, 0.0000), Vector3f(0.0000, -0.9415, -0.3370), deg2rad * Vector3f(-45.0f, -15.0f, -15.0f), deg2rad * Vector3f(45.0f, 15.0f, 15.0f));
		createMotoredBallJoint(WolfPelvis, WolfSpine, getBoneEndPos(bones[WolfSpine]), Vector3f(-1.0000, 0.0000, 0.0000), Vector3f(0.0000, -0.0134, -0.9999), deg2rad * Vector3f(-20.0f, -10.0f, -20.0f), deg2rad * Vector3f(20.0f, 10.0f, 20.0f));
		createMotoredBallJoint(WolfPelvis, WolfUpperTail, getBoneEndPos(bones[WolfUpperTail]), Vector3f(-1.0000, 0.0000, 0.0000), Vector3f(0.0000, 0.4237, 0.9058), deg2rad * Vector3f(-30.0f, -30.0f, -30.0f), deg2rad * Vector3f(45.0f, 30.0f, 30.0f));
		createHinge(WolfUpperTail, WolfLowerTail, getBoneEndPos(bones[WolfLowerTail]), Vector3f(-1.0000, 0.0000, 0.0000), -deg2rad * 30.0f, deg2rad * 30.0f);
		createHinge(WolfLThigh, WolfLFoot, getBoneStartPos(bones[WolfLFoot]), Vector3f(0.9951, -0.0484, -0.0859), -deg2rad * 45.0f, deg2rad * 30.0f);
		createHinge(WolfLFoot, WolfLToe0, getBoneEndPos(bones[WolfLToe0]), Vector3f(0.9811, -0.1934, 0.0063), -deg2rad * 30.0f, deg2rad * 30.0f);
		createHinge(WolfRThigh, WolfRFoot, getBoneStartPos(bones[WolfRFoot]), Vector3f(0.9951, 0.0484, 0.0859), -deg2rad * 45.0f, deg2rad * 30.0f);
		createHinge(WolfRFoot, WolfRToe0, getBoneEndPos(bones[WolfRToe0]), Vector3f(0.9811, 0.1934, -0.0063), -deg2rad * 30.0f, deg2rad * 30.0f);
		createMotoredBallJoint(WolfSpine, WolfHead, getBoneEndPos(bones[WolfHead]), Vector3f(-1.0000, 0.0000, 0.0000), Vector3f(0.0000, 0.3571, -0.9341), deg2rad * Vector3f(-10.0f, -10.0f, -10.0f), deg2rad * Vector3f(10.0f, 10.0f, 10.0f));
		createMotoredBallJoint(WolfSpine, WolfLUpperArm, getBoneEndPos(bones[WolfLUpperArm]), Vector3f(-0.9965, -0.0720, -0.0413), Vector3f(-0.0810, 0.9526, 0.2933), deg2rad * Vector3f(-45.0f, -15.0f, -30.0f), deg2rad * Vector3f(45.0f, 15.0f, 30.0f));
		createHinge(WolfLUpperArm, WolfLForearm, getBoneEndPos(bones[WolfLForearm]), Vector3f(-0.9965, -0.0720, -0.0413), -deg2rad * 45.0f, deg2rad * 45.0f);
		createHinge(WolfLForearm, WolfLHand, getBoneEndPos(bones[WolfLHand]), Vector3f(-0.9836, 0.1527, -0.0965), -deg2rad * 45.0f, deg2rad * 30.0f);
		createMotoredBallJoint(WolfSpine, WolfRUpperArm, getBoneEndPos(bones[WolfRUpperArm]), Vector3f(-0.9965, 0.0720, 0.0413), Vector3f(0.0810, 0.9526, 0.2933), deg2rad * Vector3f(-45.0f, -15.0f, -30.0f), deg2rad * Vector3f(45.0f, 15.0f, 30.0f));
		createHinge(WolfRUpperArm, WolfRForearm, getBoneEndPos(bones[WolfRForearm]), Vector3f(-0.9965, 0.0720, 0.0413), -deg2rad * 45.0f, deg2rad * 45.0f);
		createHinge(WolfRForearm, WolfRHand, getBoneEndPos(bones[WolfRHand]), Vector3f(-0.9836, -0.1527, 0.0965), -deg2rad * 45.0f, deg2rad * 30.0f);

		if (fixInAir)
		{
			int joint = odeJointCreateFixed();
			odeJointAttach(joint, 0, bones[0]->body);
			odeJointSetFixed(joint);
		}
		genericInit(fixInAir ? 0.1f : 50.0f);
		commandLogFileName = "CommandsLog-Wolf.txt";
	}
};