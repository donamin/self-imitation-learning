#pragma once
#include "OdeRig.h"

class WolfRiderOrc : public OdeRig
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
		, RiderPelvis
		, RiderSpine
		, RiderLUpperArm
		, RiderLForearm
		, RiderHead
		, RiderRUpperArm
		, RiderRForearm
		, WolfUpperTail
		, WolfLowerTail
		, bCount
	};

	void init(bool fixInAir = false)
	{
		enableSelfCollisions = true;

		bones.resize(bCount, NULL);
		angleSd.resize(bCount, 10);

		bones[WolfPelvis] = createBone(0.0000, 0.3801, 1.4193, 0.5500, 0.2500, Vector3f(0.0000, 0.9987, -0.0514));
		bones[WolfLThigh] = createBone(-0.1599, 0.5011, 1.1635, 0.5000, 0.1500, Vector3f(0.0000, -0.3370, 0.9415));
		bones[WolfLFoot] = createBone(-0.1717, 0.6561, 0.7803, 0.5000, 0.1000, Vector3f(-0.0986, -0.4886, -0.8669));
		bones[WolfLToe0] = createBone(-0.2332, 0.3952, 0.5711, 0.3000, 0.1000, Vector3f(0.1935, 0.9805, -0.0347));
		bones[WolfRThigh] = createBone(0.1599, 0.5011, 1.1635, 0.5000, 0.1500, Vector3f(0.0000, -0.3370, 0.9415));
		bones[WolfRFoot] = createBone(0.1717, 0.6561, 0.7803, 0.5000, 0.1000, Vector3f(0.0986, -0.4886, -0.8669));
		bones[WolfRToe0] = createBone(0.2332, 0.3952, 0.5711, 0.3000, 0.1000, Vector3f(-0.1935, 0.9805, -0.0347));
		bones[WolfSpine] = createBone(0.0000, -0.2235, 1.4338, 0.8000, 0.3000, Vector3f(0.0000, 0.9999, -0.0134));
		bones[WolfLUpperArm] = createBone(-0.1989, -0.2510, 1.0570, 0.4500, 0.1500, Vector3f(-0.0183, -0.2957, 0.9551));
		bones[WolfLForearm] = createBone(-0.1816, -0.2991, 0.7258, 0.3500, 0.1000, Vector3f(-0.0736, 0.5350, 0.8417));
		bones[WolfLHand] = createBone(-0.1938, -0.5397, 0.5663, 0.3000, 0.1000, Vector3f(0.1202, 0.9520, 0.2814));
		bones[WolfHead] = createBone(0.0000, -0.9203, 1.3840, 0.6500, 0.1500, Vector3f(0.0000, 0.9341, 0.3571));
		bones[WolfRUpperArm] = createBone(0.1989, -0.2510, 1.0570, 0.4500, 0.1500, Vector3f(0.0183, -0.2957, 0.9551));
		bones[WolfRForearm] = createBone(0.1816, -0.2991, 0.7258, 0.3500, 0.1000, Vector3f(0.0736, 0.5350, 0.8417));
		bones[WolfRHand] = createBone(0.1938, -0.5397, 0.5663, 0.3000, 0.1000, Vector3f(-0.1202, 0.9520, 0.2814));
		bones[RiderPelvis] = createBone(0.0000, 0.1796, 1.7530, 0.3000, 0.1500, Vector3f(0.0000, -0.0951, -0.9955));
		bones[RiderSpine] = createBone(0.0092, 0.1720, 2.0545, 0.3000, 0.1500, Vector3f(-0.0466, 0.2487, -0.9675));
		bones[RiderLUpperArm] = createBone(-0.3385, 0.1878, 1.9693, 0.4000, 0.1000, Vector3f(0.5782, -0.2649, 0.7717));
		bones[RiderLForearm] = createBone(-0.4088, 0.0031, 1.7989, 0.3500, 0.0800, Vector3f(-0.0111, 0.9481, 0.3178));
		bones[RiderHead] = createBone(0.0282, 0.0266, 2.3398, 0.3000, 0.1250, Vector3f(-0.0001, 0.2816, -0.9595));
		bones[RiderRUpperArm] = createBone(0.3274, 0.3120, 1.9751, 0.4000, 0.1000, Vector3f(-0.4096, -0.3761, 0.8312));
		bones[RiderRForearm] = createBone(0.5069, 0.2233, 1.7361, 0.3500, 0.0800, Vector3f(-0.5559, 0.5916, 0.5839));
		bones[WolfUpperTail] = createBone(0.0000, 0.8082, 1.4546, 0.5000, 0.0500, Vector3f(0.0000, -0.9058, 0.4237));
		bones[WolfLowerTail] = createBone(0.0000, 1.2867, 1.3407, 0.4500, 0.0500, Vector3f(0.0000, -0.9999, -0.0115));

		angleSd[WolfPelvis] = deg2rad * 5;
		angleSd[WolfLThigh] = deg2rad * 20;
		angleSd[WolfLFoot] = deg2rad * 20;
		angleSd[WolfLToe0] = deg2rad * 5;
		angleSd[WolfRThigh] = deg2rad * 20;
		angleSd[WolfRFoot] = deg2rad * 20;
		angleSd[WolfRToe0] = deg2rad * 5;
		angleSd[WolfSpine] = deg2rad * 5;
		angleSd[WolfLUpperArm] = deg2rad * 20;
		angleSd[WolfLForearm] = deg2rad * 20;
		angleSd[WolfLHand] = deg2rad * 5;
		angleSd[WolfHead] = deg2rad * 5;
		angleSd[WolfRUpperArm] = deg2rad * 20;
		angleSd[WolfRForearm] = deg2rad * 20;
		angleSd[WolfRHand] = deg2rad * 5;
		angleSd[RiderPelvis] = deg2rad * 2.5f;
		angleSd[RiderSpine] = deg2rad * 2.5f;
		angleSd[RiderLUpperArm] = deg2rad * 2.5f;
		angleSd[RiderLForearm] = deg2rad * 2.5f;
		angleSd[RiderHead] = deg2rad * 2.5f;
		angleSd[RiderRUpperArm] = deg2rad * 2.5f;
		angleSd[RiderRForearm] = deg2rad * 2.5f;
		angleSd[WolfUpperTail] = deg2rad * 2.5f;
		angleSd[WolfLowerTail] = deg2rad * 2.5f;

		createMotoredBallJoint(WolfPelvis, WolfLThigh, getBoneEndPos(bones[WolfLThigh]), Vector3f(1.0000, 0.0000, 0.0000), Vector3f(0.0000, -0.9415, -0.3370), deg2rad * Vector3f(-45.0f, -30.0f, -15.0f), deg2rad * Vector3f(45.0f, 30.0f, 45.0f));
		createMotoredBallJoint(WolfPelvis, WolfRThigh, getBoneEndPos(bones[WolfRThigh]), Vector3f(1.0000, 0.0000, 0.0000), Vector3f(0.0000, -0.9415, -0.3370), deg2rad * Vector3f(-45.0f, -30.0f, -45.0f), deg2rad * Vector3f(45.0f, 30.0f, 15.0f));
		createMotoredBallJoint(WolfPelvis, WolfSpine, getBoneEndPos(bones[WolfSpine]), Vector3f(-1.0000, 0.0000, 0.0000), Vector3f(0.0000, -0.0134, -0.9999), deg2rad * Vector3f(-20.0f, -30.0f, -30.0f), deg2rad * Vector3f(30.0f, 30.0f, 30.0f));
		createMotoredBallJoint(WolfPelvis, WolfUpperTail, getBoneEndPos(bones[WolfUpperTail]), Vector3f(-1.0000, 0.0000, 0.0000), Vector3f(0.0000, 0.4237, 0.9058), deg2rad * Vector3f(-30.0f, -10.0f, -45.0f), deg2rad * Vector3f(30.0f, 10.0f, 45.0f));
		createHinge(WolfUpperTail, WolfLowerTail, getBoneEndPos(bones[WolfLowerTail]), Vector3f(-1.0000, 0.0000, 0.0000), -deg2rad * 120.0f, deg2rad * 90.0f);
		createHinge(WolfLThigh, WolfLFoot, getBoneStartPos(bones[WolfLFoot]), Vector3f(0.9951, -0.0484, -0.0859), -deg2rad * 150.0f, deg2rad * 45.0f);
		createHinge(WolfLFoot, WolfLToe0, getBoneEndPos(bones[WolfLToe0]), Vector3f(0.9811, -0.1934, 0.0063), -deg2rad * 45.0f, deg2rad * 30.0f);
		createHinge(WolfRThigh, WolfRFoot, getBoneStartPos(bones[WolfRFoot]), Vector3f(0.9951, 0.0484, 0.0859), -deg2rad * 150.0f, deg2rad * 45.0f);
		createHinge(WolfRFoot, WolfRToe0, getBoneEndPos(bones[WolfRToe0]), Vector3f(0.9811, 0.1934, -0.0063), -deg2rad * 45.0f, deg2rad * 30.0f);
		createMotoredBallJoint(WolfSpine, WolfHead, getBoneEndPos(bones[WolfHead]), Vector3f(-1.0000, 0.0000, 0.0000), Vector3f(0.0000, 0.3537, -0.9353), deg2rad * Vector3f(-30.0f, -30.0f, -45.0f), deg2rad * Vector3f(30.0f, 30.0f, 45.0f));
		createMotoredBallJoint(WolfSpine, WolfLUpperArm, getBoneEndPos(bones[WolfLUpperArm]), Vector3f(-0.9965, -0.0722, -0.0411), Vector3f(-0.0810, 0.9536, 0.2899), deg2rad * Vector3f(-90.0f, -30.0f, -45.0f), deg2rad * Vector3f(45.0f, 30.0f, 15.0f));
		createHinge(WolfLUpperArm, WolfLForearm, getBoneEndPos(bones[WolfLForearm]), Vector3f(-0.9965, -0.0722, -0.0411), -deg2rad * 15.0f, deg2rad * 30.0f);
		createHinge(WolfLForearm, WolfLHand, getBoneEndPos(bones[WolfLHand]), Vector3f(-0.9836, 0.1527, -0.0965), -deg2rad * 30.0f, deg2rad * 30.0f);
		createMotoredBallJoint(WolfSpine, WolfRUpperArm, getBoneEndPos(bones[WolfRUpperArm]), Vector3f(-0.9965, 0.0722, 0.0411), Vector3f(0.0810, 0.9536, 0.2899), deg2rad * Vector3f(-90.0f, -30.0f, -15.0f), deg2rad * Vector3f(45.0f, 30.0f, 45.0f));
		createHinge(WolfRUpperArm, WolfRForearm, getBoneEndPos(bones[WolfRForearm]), Vector3f(-0.9965, 0.0722, 0.0411), -deg2rad * 15.0f, deg2rad * 30.0f);
		createHinge(WolfRForearm, WolfRHand, getBoneEndPos(bones[WolfRHand]), Vector3f(-0.9836, -0.1527, 0.0965), -deg2rad * 30.0f, deg2rad * 30.0f);
		createMotoredBallJoint(WolfSpine, RiderPelvis, getBoneEndPos(bones[RiderPelvis]), Vector3f(-1.0000, 0.0000, 0.0000), Vector3f(0.0000, -0.9955, 0.0951), deg2rad * Vector3f(-1.0f, -1.0f, -1.0f), deg2rad * Vector3f(1.0f, 1.0f, 1.0f));
		createMotoredBallJoint(RiderPelvis, RiderSpine, getBoneEndPos(bones[RiderSpine]), Vector3f(-0.9580, -0.2857, -0.0262), Vector3f(0.2831, -0.9264, -0.2481), deg2rad * Vector3f(-15.0f, -30.0f, -30.0f), deg2rad * Vector3f(30.0f, 30.0f, 30.0f));
		createMotoredBallJoint(RiderSpine, RiderLUpperArm, getBoneEndPos(bones[RiderLUpperArm]), Vector3f(-0.8159, -0.1904, 0.5459), Vector3f(-0.0040, 0.9461, 0.3239), deg2rad * Vector3f(-45.0f, -30.0f, -120.0f), deg2rad * Vector3f(45.0f, 30.0f, 30.0f));
		createHinge(RiderLUpperArm, RiderLForearm, getBoneEndPos(bones[RiderLForearm]), Vector3f(-0.8159, -0.1904, 0.5459), -deg2rad * 45.0f, deg2rad * 90.0f);
		createMotoredBallJoint(RiderSpine, RiderRUpperArm, getBoneEndPos(bones[RiderRUpperArm]), Vector3f(-0.8163, -0.2576, -0.5170), Vector3f(-0.4074, 0.8913, 0.1990), deg2rad * Vector3f(-45.0f, -30.0f, -30.0f), deg2rad * Vector3f(45.0f, 30.0f, 120.0f));
		createHinge(RiderRUpperArm, RiderRForearm, getBoneEndPos(bones[RiderRForearm]), Vector3f(-0.8163, -0.2576, -0.5170), -deg2rad * 45.0f, deg2rad * 90.0f);
		createMotoredBallJoint(RiderSpine, RiderHead, getBoneEndPos(bones[RiderHead]), Vector3f(-0.9993, 0.0371, 0.0109), Vector3f(-0.0387, -0.9598, -0.2779), deg2rad * Vector3f(-30.0f, -30.0f, -45.0f), deg2rad * Vector3f(45.0f, 30.0f, 45.0f));

		if (fixInAir)
		{
			int joint = odeJointCreateFixed();
			odeJointAttach(joint, 0, bones[0]->body);
			odeJointSetFixed(joint);
		}
		genericInit(fixInAir ? 0.1f : 50.0f);
		commandLogFileName = "CommandsLog-WolfRiderOrc.txt";
	}
};