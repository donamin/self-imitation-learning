#include "ZeroMQInterface.h"

#include "RenderClient.h"
#include "RenderCommands.h"
#include "UnityOde.h"
#include "UnityOde_internal.h"

#include "mathutils.h"
#include "SCAController.h"
#include "FileUtils.h"

#include "LightInfantryOrcLeaning.h"
#include "Wolf.h"
#include "Mech.h"
#include "WolfRiderOrc.h"

#include "Pose.h"

using namespace AaltoGames;

#define LIGHTINFANTRYORCLEANING 1
#define WOLF 2
#define MECH 3

#define CHARACTER LIGHTINFANTRYORCLEANING

static const bool rigTestMode = false;

static const bool trainPPO = true;
static const int startStateSamplingForPPOAt = 2 * 60 * 30;
static const int maxEpLenPPO = 100;
static const bool useNegCostAsTaskReward = false;
static const bool useImitationReward = true;
static const bool curriculumLearningPPO = true;
static const float curriculumInitialThreshold = 0.75f, curriculumFinalThreshold = 0.5f;
static const int maxIterationsPPO = 5000;
static const int maxEpisodesRender = 5;
static const int maxEpisodesLenRender = 1 * 60 * 30;
static bool loadPPOModel = false;
static std::string ppoModelFolderName = "openai-2018-09-28-14-03-53-007222";

#if CHARACTER == LIGHTINFANTRYORCLEANING
typedef LightInfantryOrcLeaning TestRig;
static const float controlAccSd = deg2rad * 15.0f;
static const int nTrajectories = 64;
static const int nRealtimeTrajectories = 2;

int non_ground_contact_bones[] = {
	(int)TestRig::Bones::Pelvis
	,(int)TestRig::Bones::LThigh
	,(int)TestRig::Bones::RThigh
	,(int)TestRig::Bones::Spine
	,(int)TestRig::Bones::LUpperArm
	,(int)TestRig::Bones::LForearm
	,(int)TestRig::Bones::Head
	,(int)TestRig::Bones::RUpperArm
	,(int)TestRig::Bones::RForearm
};
int feet_bones[] = { (int)TestRig::Bones::LFoot,(int)TestRig::Bones::RFoot };
static const int countEndEffectors = 4;
int end_effector_bones[] = { (int)TestRig::Bones::LFoot,(int)TestRig::Bones::RFoot,(int)TestRig::Bones::LForearm,(int)TestRig::Bones::RForearm };
#endif

#if CHARACTER == WOLF
typedef Wolf TestRig;
static const float controlAccSd = deg2rad * 15.0f;
static const int nTrajectories = 64;
static const int nRealtimeTrajectories = 2;

int non_ground_contact_bones[] = {
	(int)TestRig::Bones::WolfPelvis
	,(int)TestRig::Bones::WolfLThigh
	,(int)TestRig::Bones::WolfRThigh
	,(int)TestRig::Bones::WolfSpine
	,(int)TestRig::Bones::WolfLUpperArm
	,(int)TestRig::Bones::WolfRUpperArm
	,(int)TestRig::Bones::WolfHead
	,(int)TestRig::Bones::WolfUpperTail
	,(int)TestRig::Bones::WolfLowerTail
};
int feet_bones[] = { (int)TestRig::Bones::WolfLToe0,(int)TestRig::Bones::WolfRToe0,(int)TestRig::Bones::WolfLHand,(int)TestRig::Bones::WolfRHand };
static const int countEndEffectors = 4;
int end_effector_bones[] = { (int)TestRig::Bones::WolfLToe0,(int)TestRig::Bones::WolfRToe0,(int)TestRig::Bones::WolfLHand,(int)TestRig::Bones::WolfRHand };
#endif

#if CHARACTER == MECH
typedef Mech TestRig;
static const float controlAccSd = deg2rad * 10.0f;
static const int nTrajectories = 64;
static const int nRealtimeTrajectories = 2;

int non_ground_contact_bones[] = {
	(int)TestRig::Bones::Pelvis
	,(int)TestRig::Bones::Spine
	,(int)TestRig::Bones::LUpperLeg
	,(int)TestRig::Bones::LLowerLeg
	,(int)TestRig::Bones::RUpperLeg
	,(int)TestRig::Bones::RLowerLeg
};
int feet_bones[] = { (int)TestRig::Bones::LFoot,(int)TestRig::Bones::RFoot };
static const int countEndEffectors = 2;
int end_effector_bones[] = { (int)TestRig::Bones::LFoot,(int)TestRig::Bones::RFoot };
#endif

//Common parameters
static int num_motor_angles = 0;
static const float maxDistanceFromOrigin = 5.0f;
PoseEditor *poseEditor;
static const float poseSpringConstant = 10.0f;
static TestRig character;
static VectorXf controlMin, controlMax, controlRange, controlMean, controlSd, controlDiffSd;
static SCAControl *flc;

static const float planningHorizonSeconds = 1.2f;
static int nTimeSteps = (int)(planningHorizonSeconds / timeStep);
static const int fps = (int)(0.5f + 1.0f / timeStep);
static const bool useThreads = true;
static int resetSaveSlot = nTrajectories + 1;
static int masterContext;
static const float kmh2ms = 1.0f / 3.6f;
static float targetSpeed = 1.0f;
//static Vector3f targetVel(-6.0f*kmh2ms, 0, 0);  //run
//static Vector3f targetVel(0,0,0);
static const float ikSd = 0.05f;
static const bool scoreAngles = true;
static const float velSd = 0.05f;
static const bool useContactVelCost = false;
static const float contactVelSd = 0.2f;
static const float comDiffSd = 0.025f;
static const float controlSdRelToRange = 8.0f;

static const float angleSamplingSd = deg2rad*25.0f;  //sampling
static const float resampleThreshold = 2.0f;
static const float mutationSd = 0.1f;
static const float poseTorqueRelSd = 0.5f;
static const float poseTorqueK = maxControlTorque / (1.0f*PI); //we exert maximum torque towards default pose if the angle displacement is 90 degrees 
static const float friction = 0.5f; //higher friction needed compared to the biped to prevent "cheat gaits"
static int frameIdx = 0;
static int randomSeed = 2;
static const bool onlyAdvanceIfNotFalling = false;
static const bool multiTask = false;
static bool enableRealTimeMode = true;
static bool realtimeMode = false;
static int stateDim; //character state dim + 1 for target speed
static const bool useErrorLearning = false;
static const bool test_real_time_mode = false;

//static int autoExitAt = 1 * 60 * 30;
static int autoExitAt = 100 * 60 * 60 * 30;

//launched spheres
static const bool useSpheres = false;
static const int sphereInterval = 5000;
static int lastSphereLaunchTime = 0; //to delay ball throwing so that gait has time to emerge with video capturing

//acceleration
static const bool useAcceleration = false;
static const float acceleration = 1.0f / 10.0f; //1 m/s in 10 seconds
static const float acceleratedMaxSpeed = 100.0f; //practically inf (meters per second)

static bool use_external_prior = true;

//random impulses 
static bool useRandomImpulses = false; //if true, learning is made more robust through application of random impulses
static const float randomImpulseMagnitudeMin = 200.0f, randomImpulseMagnitudeMax = 500.0f;
static int randomImpulseTimer = -1;
static const int randomImpulseIntervalMin = 2 * 30, randomImpulseIntervalMax = 5 * 30;

//targeted walking
static bool useWalkTargets = false; //if true, targetVel computed every frame so that the character walks towards the walkTargets, targets switched after reached
static Vector3f walkTarget = Vector3f(0, -1000, 0);

static int walk_time = 200;

//recovery mode !
static const bool enableRecoveryMode = false;
static const bool includeRecoveryModeInState = true;// true;  //has to be included as long as the mode directly affects fmax and spring kp
static float recoveryModePoseSdMult = 5.0f;
static float recoveryModeAccSdMult = 1.0f;
static float recoveryModeFmax = defaultFmax*2.0f;
static bool inRecoveryMode = false;
static const float recoveryModeAngleLimit = 30.0f;
static const float recoveryModeSpringKp = springKp*5.0f;
static const float recoveryModeTimeUntilReset = 2.0f; //wait this many seconds before resetting

//misc
static int groundPlane = -1;

std::deque<Eigen::VectorXf> control_sequence;
std::deque<Eigen::VectorXf> machine_learning_control_sequence;


static bool run_on_neural_network = false;

static std::string cost_file_name = "costs.csv";
static std::string started_at_time_string = "";
static std::vector<std::vector<float>> costs;
static std::vector<std::string> comments;
static bool no_settings_exit = false;

static Eigen::VectorXf minControls;
static Eigen::VectorXf maxControls;

static ZMQ_Messanger *zmqMessanger;

static bool visualizePPO = false; //When true, always render. When false, only render every 100 PPO iterations.
static bool runningPPO = false;
static bool evaluatingPolicy = true;
static bool renderingPolicy = false;
static bool finishedPPO = false;
static int nStateSamplesForPPO = 0;
static int renderedEpisodes = 0;
static int nEpochs = 0;
static float rewardThreshold;

class MotionState
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  //needed as we have fixed size eigen matrices as members
	Vector3f avel[TestRig::Bones::bCount], endEffectorsPos[countEndEffectors], endEffectorsVel[countEndEffectors], com;
	Quaternionf q[TestRig::Bones::bCount];
};

static std::map<int, MotionState *> motionStates;
static int referenceMotionSlot[nTrajectories + 1];

static void write_vector_to_file(const std::string& filename, const std::vector<std::vector<float> >& data, const std::vector<std::string> comments = std::vector<std::string>())
{
	std::ofstream myfile;
	myfile.open(filename);
	myfile.clear();

	for (const std::string& comment_line : comments) {
		myfile << "//" << comment_line << std::endl;
	}

	for (const std::vector<float>& measurement : data) {
		for (unsigned i = 0; i < measurement.size(); i++) {
			myfile << measurement[i];
			if (i < (int)measurement.size() - 1) {
				myfile << ",";
			}
		}
		myfile << std::endl;
	}
	myfile.close();
}

//timing
high_resolution_clock::time_point t1;
void startPerfCount()
{
	t1 = high_resolution_clock::now();
}
int getDurationMs()
{
	high_resolution_clock::time_point t2 = high_resolution_clock::now();

	std::chrono::duration<double> time_span = duration_cast<std::chrono::duration<double>>(t2 - t1);
	return (int)(time_span.count()*1000.0);
}

class SphereData
{
public:
	int body, geom, spawnFrame;
};
static std::vector<SphereData> spheres;

class SimulationContext
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  //needed as we have fixed size eigen matrices as members
	VectorXf stateFeatures;
	VectorXf control;
	VectorXf priorMean, priorSd;
	VectorXf angleRates;
	Vector3f initialPosition, resultPosition;
	float stateCost;
	float controlCost;
	int trajectoryIdx;
};

static SimulationContext contexts[nTrajectories + 2];

Vector3f get_target_dir(Vector3f& com) {

	Vector3f dir = walkTarget - com;
	dir.z() = 0;
	dir.normalize();
	return dir;
}

void SaveMotionState(int slot)
{
	MotionState *state = new MotionState();

	character.computeCOM(state->com);

	Vector3f target_dir = get_target_dir(state->com);
	const Vector3f initialDir(-1, 0, 0);
	Quaternionf targetRotation = Quaternionf::FromTwoVectors(initialDir, target_dir);
	Quaternionf stateRotation = targetRotation.inverse();

	for (size_t i = 0; i < character.bones.size(); ++i)
	{
		//Vector3f pos(odeBodyGetPosition(bones[i]->body));
		//Vector3f vel_tmp(odeBodyGetLinearVel(bones[i]->body));
		//Vector3f vel = stateRotation * vel_tmp;

		Vector3f avel_tmp(odeBodyGetAngularVel(character.bones[i]->body));
		Vector3f avel = stateRotation * avel_tmp;

		Quaternionf q_tmp = ode2eigenq(odeBodyGetQuaternion(character.bones[i]->body));
		Quaternionf q = stateRotation * q_tmp;

		//pushStateVector3f(idx, out_state, pos - com);
		//pushStateVector3f(idx, out_state, vel);
		state->avel[i] = avel;
		state->q[i] = q;
	}

	Vector3f rootPos(odeBodyGetPosition(character.bones[0]->body));
	for(int i = 0; i < countEndEffectors; ++i)
	{
		state->endEffectorsPos[i] = Vector3f(odeBodyGetPosition(character.bones[end_effector_bones[i]]->body)) - rootPos;
		state->endEffectorsVel[i] = character.runningAvgLinVel[end_effector_bones[i]];
	}

	motionStates[slot] = state;
}

float ComputeImitationReward(int slot)
{
	auto it = motionStates.find(slot);
	if (it == motionStates.end())
	{
		printf("Trying to imitate a null motion!\n");
	}
	float pose_rew = 0, vel_rew = 0, end_effector_rew = 0, com_rew = 0;

	Vector3f com;
	character.computeCOM(com);
	Vector3f target_dir = get_target_dir(com);
	const Vector3f initialDir(-1, 0, 0);
	Quaternionf targetRotation = Quaternionf::FromTwoVectors(initialDir, target_dir);
	Quaternionf stateRotation = targetRotation.inverse();

	MotionState *reference = it->second;
	for (size_t i = 0; i < character.bones.size(); ++i)
	{
		Vector3f avel_tmp(odeBodyGetAngularVel(character.bones[i]->body));
		Vector3f avel = stateRotation * avel_tmp;

		Quaternionf q_tmp = ode2eigenq(odeBodyGetQuaternion(character.bones[i]->body));
		Quaternionf q = stateRotation * q_tmp;

		pose_rew += squared(q.angularDistance(reference->q[i]));
		vel_rew += (avel - reference->avel[i]).squaredNorm();
	}
	for (int i = 0; i < countEndEffectors; ++i)
	{
		end_effector_rew += (reference->endEffectorsPos[i] - Vector3f(odeBodyGetPosition(character.bones[end_effector_bones[i]]->body))).squaredNorm();
	}

	com_rew = (reference->com - com).squaredNorm();

	vel_rew = exp(-1.3f * vel_rew / float(TestRig::Bones::bCount));
	pose_rew = exp(-26.0f * pose_rew / float(TestRig::Bones::bCount));
	end_effector_rew = exp(-160.0f * end_effector_rew / float(countEndEffectors));
	com_rew = exp(-10.0f * com_rew);

	static const float pose_w = 0.65f, vel_w = 0.1f, end_effector_w = 0.15f, com_w = 0.1f;

	float rew = pose_w * pose_rew + vel_w * vel_rew + end_effector_w * end_effector_rew + com_w * com_rew;
	return rew;
}

//state vector seen by the controller, including both character's physical state and task information
int computeStateVector(float *out_state)
{
	Vector3f com;
	character.computeCOM(com);

	Vector3f target_dir = get_target_dir(com);
	const Vector3f initialDir(-1, 0, 0);
	Quaternionf targetRotation = Quaternionf::FromTwoVectors(initialDir, target_dir);
	Quaternionf stateRotation = targetRotation.inverse(); //The codes that were used in the paper had a bug here. The stateRotation was computed before running the MCTS instead of being computed here.

	const bool use_motor_angles = false;

	int nState = 0;
	if (!use_motor_angles)
	{
		nState = character.computeStateVector(out_state, stateRotation);
	}

	if (use_motor_angles) {

		//since ground contacts are highly significant, add foot bone tip y pos and vel with greater weight
		for (int i = 0; i <= 1; i++)
		{
			Vector3f footPos = character.getFootPos((OdeRig::BodySides)i);
			//Vector3f footVel(odeBodyGetLinearVel(character.getFootBone((OdeRig::BodySides)i)->body));
			out_state[nState] = footPos.z();
			nState++;
			//out_state[nState] = footVel.z();
			//nState++;
		}

		Vector3f tmp = com;
		out_state[nState] = tmp.z();
		nState++;

		character.computeMeanVel(tmp);
		out_state[nState] = tmp.z();
		nState++;

		Quaternionf q(odeBodyGetQuaternion(character.bones[0]->body));
		Quaternionf root_rotation = stateRotation*q;

		out_state[nState] = root_rotation.x();
		nState++;
		out_state[nState] = root_rotation.y();
		nState++;
		out_state[nState] = root_rotation.z();
		nState++;
		out_state[nState] = root_rotation.w();
		nState++;

		if (num_motor_angles == 0) {
			for (auto joint_ptr : character.joints) {
				num_motor_angles += joint_ptr->nMotorDof;
			}
		}

		float motor_angles[100];
		character.getCurrentMotorAngles(motor_angles);

		for (int motor_angle = 0; motor_angle < num_motor_angles; motor_angle++) {
			out_state[nState] = motor_angles[motor_angle];
			nState++;
		}

		character.getCurrentAngleRates(motor_angles);

		for (int motor_angle = 0; motor_angle < num_motor_angles; motor_angle++) {
			out_state[nState] = motor_angles[motor_angle];
			nState++;
		}

		//task variables scaled larger so that they dominate the state distances
		out_state[nState++] = target_dir.norm()*10.0f;

	}

	if (enableRecoveryMode && includeRecoveryModeInState  && flc->sampling_mode_ == SCAControl::SamplingMode::L1)
	{
		out_state[nState++] = 10.0f*(inRecoveryMode ? 1.0f : 0);  //needed as state cost computed differently, and recovery mode affects the state,action -> next state mapping (through spring constants and fmax)
	}

	/*if (useSpheres)
	{
		bool pushZeros = true;
		if (spheres.size() > 0)
		{
			//add the relative position of the last launched sphere
			SphereData &sd = spheres.back();
			Vector3f spherePos(odeBodyGetPosition(sd.body));
			Vector3f sphereVel(odeBodyGetLinearVel(sd.body));
			Vector3f pos(odeBodyGetPosition(character.bones[0]->body));
			Vector3f relPos = stateRotation*(spherePos - pos);
			Vector3f relVel = stateRotation*sphereVel;
			//if sphere flying towards us, add it to the state
			if (relPos.norm() < 5.0f && relVel.dot(relPos.normalized()) < -1.0f)
			{
				character.pushStateVector3f(nState, out_state, relPos);
				character.pushStateVector3f(nState, out_state, relVel);
				pushZeros = false;
			}
		}
		if (pushZeros)
		{
			character.pushStateVector3f(nState, out_state, Vector3f::Zero());
			character.pushStateVector3f(nState, out_state, Vector3f::Zero());
		}
	}*/
	out_state[nState++] = 0;
	return nState;
}

std::chrono::time_point<std::chrono::system_clock> startTimer, endTimer;

Eigen::VectorXf init_motor_angles;

static int character_root_bone = -1;

static int orig_ml_trajectories;
static int orig_noisy_ml_trajectories;
static int orig_nearest_neighbor_trajectories;

void EXPORT_API rcInit()
{
	if (useRandomImpulses)
	{
		randomImpulseTimer = randInt(randomImpulseIntervalMin, randomImpulseIntervalMax);
	}
#if CHARACTER == HUMANOID
	character_root_bone = (int)TestRig::Bones::bPelvis;
#endif
	if (test_real_time_mode) {
		enableRealTimeMode = true;
	}

	started_at_time_string = get_time_string();
	unsigned seed = time(nullptr);
	srand(seed);

	costs.clear();
	comments.clear();
	comments.push_back("This is a sample comment.");

	initOde(nTrajectories + 2);
	setCurrentOdeContext(ALLTHREADS);
	allocateODEDataForThread();
	odeRandSetSeed(randomSeed);
	// create world
	odeWorldSetGravity(0, 0, -9.81f);
	odeWorldSetCFM(1e-5);
	odeSetFrictionCoefficient(friction);
	//odeWorldSetLinearDamping(0.001);
	//dWorldSetAngularDamping(world, 0.001);
	//dWorldSetMaxAngularSpeed(world, 200);

	odeWorldSetContactMaxCorrectingVel(5);
	odeWorldSetContactSurfaceLayer(0.01f);
	groundPlane = odeCreatePlane(0, 0, 0, 1, 0);
	odeGeomSetCategoryBits(groundPlane, 0xf0000000);
	odeGeomSetCollideBits(groundPlane, 0xffffffff);
	character.init(rigTestMode);

	//controller init (motor target velocities are the controlled variables, one per joint)
	float tmp[1024];
	stateDim = computeStateVector(tmp);
	controlMin = Eigen::VectorXf::Zero(character.controlDim);
	controlMax = Eigen::VectorXf::Zero(character.controlDim);
	controlMean = Eigen::VectorXf::Zero(character.controlDim);
	controlSd = Eigen::VectorXf::Zero(character.controlDim);
	controlDiffSd = Eigen::VectorXf::Zero(character.controlDim);

	printf("State dim = %d, Action dim = %d\n", stateDim, character.controlDim);

	int controlVarIdx = 0;
	for (size_t i = 0; i < character.joints.size(); i++)
	{
		OdeRig::Joint *j = character.joints[i];
		for (int dofIdx = 0; dofIdx < j->nMotorDof; dofIdx++)
		{
			controlMin[controlVarIdx] = j->angleMin[dofIdx];
			controlMax[controlVarIdx] = j->angleMax[dofIdx];
			controlSd[controlVarIdx] = angleSamplingSd;

			controlMean[controlVarIdx] = 0;
			controlDiffSd[controlVarIdx] = controlSd[controlVarIdx];
			controlVarIdx++;
		}
	}

	for (int i = 0; i < nTrajectories + 1; i++)	//+1 because of master context
	{
		contexts[i].stateFeatures.resize(stateDim);
		contexts[i].control.resize(character.controlDim);
		contexts[i].trajectoryIdx = i;
		contexts[i].priorMean.resize(character.controlDim);
		contexts[i].priorSd.resize(character.controlDim);
		contexts[i].angleRates.resize(character.controlDim);
	}

	masterContext = nTrajectories;
	setCurrentOdeContext(masterContext);
	saveOdeState(resetSaveSlot, masterContext);
	saveOdeState(masterContext, masterContext);

	if (rigTestMode)
	{
		poseEditor = new PoseEditor(&character);
		autoExitAt = 10 * 60 * 60 * 30;
	}
	else
	{
		flc = new SCAControl();

		auto read_settings = [&] {

			std::ifstream infile("settings.txt");

			std::string comment = "//";

			std::deque<std::string> settings;

			for (std::string line; getline(infile, line); )
			{

				if (line.find(comment) != std::string::npos) {
					continue;
				}

				settings.push_back(line);

			}

			infile.close();

			if (settings.size() < 7) {
				autoExitAt = 0;
				return;
			}


			if (!settings[0].compare("MACHINE_LEARNING")) {
				flc->use_machine_learning_ = true;
			}
			else {
				flc->use_machine_learning_ = false;
			}

			if (!settings[1].compare("NEAREST_NEIGHBOR")) {
				flc->use_forests_ = true;
			}
			else {
				flc->use_forests_ = false;
			}


			if (!settings[2].compare("KL")) {
				flc->sampling_mode_ = SCAControl::SamplingMode::KL;
			}

			if (!settings[2].compare("L1")) {
				flc->sampling_mode_ = SCAControl::SamplingMode::L1;
			}


			if (!settings[3].compare("BSELU")) {
				flc->network_type_ = SCAControl::NetworkType::BSELU;
			}

			if (!settings[3].compare("ELU")) {
				flc->network_type_ = SCAControl::NetworkType::ELU;
			}


			int numb;
			std::istringstream(settings[4]) >> numb;

			flc->amount_network_layers_ = numb;


			std::istringstream(settings[5]) >> flc->drop_out_stdev_;
			std::istringstream(settings[6]) >> flc->regularization_noise_;


			std::ofstream outfile("settings.txt", std::ofstream::out);
			outfile.clear();
			outfile.close();

			useWalkTargets = false;
			flc->force_old_best_valid_ = true;
		};

		//read_settings();

		flc->force_old_best_valid_ = true; //Set this true so the controller is stable after shifting the character to some desired position (e.g., origin)

		controlRange = controlMax - controlMin;

		flc->amount_data_in_tree_ = 2 * 60 * 30; //2 minutes

		flc->init(nTrajectories, nTimeSteps, stateDim, character.controlDim, controlMin.data(), controlMax.data(), controlMean.data(), controlSd.data(), controlDiffSd.data(), mutationSd, false);
		flc->no_prior_trajectory_portion_ = 0.25f;
		flc->learning_budget_ = 2000;

		minControls = flc->control_min_;
		maxControls = flc->control_max_;

		float tmp_angles[1000];
		int amount_motor_angles = character.getCurrentMotorAngles(tmp_angles);

		init_motor_angles.resize(amount_motor_angles);
		for (int i = 0; i < amount_motor_angles; ++i) {
			init_motor_angles[i] = tmp_angles[i];
		}

		orig_ml_trajectories = flc->machine_learning_samples_;
		orig_noisy_ml_trajectories = flc->noisy_machine_learning_samples_;
		orig_nearest_neighbor_trajectories = flc->nn_trajectories_;

		startTimer = std::chrono::system_clock::now();

		if (!rigTestMode && trainPPO)
		{
			//std::system("python Python\\spline_actor.py --cpp_call True");
			zmqMessanger = new ZMQ_Messanger();
			zmqMessanger->Send(ParseIntoString('m', controlMin.data(), character.controlDim));
			zmqMessanger->Send(ParseIntoString('M', controlMax.data(), character.controlDim));
			int options[5];
			options[0] = stateDim;
			options[1] = character.controlDim;
			options[2] = nTrajectories + 1;
			options[3] = maxEpLenPPO;
			options[4] = maxIterationsPPO;
			zmqMessanger->Send(ParseIntoString('I', options, 5));
			if (loadPPOModel)
			{
				zmqMessanger->Send(ParseIntoString('L', ppoModelFolderName.data()));
				runningPPO = finishedPPO = true;
			}
		}
	}
}

void EXPORT_API rcUninit()
{
	if(poseEditor)
		delete poseEditor;
	if (zmqMessanger)
		delete zmqMessanger;
	delete flc;
	uninitOde();
}

void EXPORT_API rcGetClientData(RenderClientData &data)
{
	data.physicsTimeStep = timeStep;
	data.maxAllowedTimeStep = data.physicsTimeStep; //render every step
	data.defaultMouseControlsEnabled = true;
}

static bool paused = false;

void throwSphere()
{
	//launch a ball
	setCurrentOdeContext(ALLTHREADS);
	float r = lerp(0.05f, 0.25f, randomf());
	float m = 4.0f / 3.0f * PI * pow(r, 3) * 200.0f;
	SphereData sd;
	sd.geom = odeCreateSphere(r);
	sd.body = odeBodyCreate();
	sd.spawnFrame = frameIdx;
	odeGeomSetBody(sd.geom, sd.body);
	odeMassSetSphereTotal(sd.body, m, r);
	setCurrentOdeContext(masterContext);
	restoreOdeState(masterContext);
	Vector3f characterPos(odeBodyGetPosition(character.bones[0]->body));
	Vector3f throwDir(randomf() - 0.5f, randomf() - 0.5f, 0);
	throwDir.normalize();
	Vector3f vel;
	character.computeMeanVel(vel);
	Vector3f spawnPos = characterPos - 1.0f * throwDir;  //Add vel, as the ball will hit in 1 second
	spawnPos.z() = characterPos.z() + randomf() * 0.5f;
	odeBodySetPosition(sd.body, spawnPos.x(), spawnPos.y(), spawnPos.z());
	Vector3f spawnVel = throwDir * lerp(10.0f, 15.0f, randomf());
	spawnVel.z() = 0;
	odeBodySetLinearVel(sd.body, spawnVel.x(), spawnVel.y(), spawnVel.z());
	saveOdeState(masterContext, masterContext);
	spheres.push_back(sd);
	lastSphereLaunchTime = frameIdx;
}

static void write_vectors_to_file(const std::string& filename, const std::deque<Eigen::VectorXf>& data, const std::vector<std::string> comments = std::vector<std::string>()) {

	std::ofstream myfile;
	myfile.open(filename);

	for (const std::string& comment_line : comments) {
		myfile << "//" << comment_line << std::endl;
	}

	for (const Eigen::VectorXf& datum : data) {
		int size = datum.size();
		for (int i = 0; i < size; i++) {
			myfile << datum[i];
			if (i < size - 1) {
				myfile << ",";
			}
		}
		myfile << std::endl;
	}

	myfile.close();

}

static bool switchTargets = false; //if true, toggles target switching at next update
void EXPORT_API rcOnKeyDown(int key)
{
	if (key == 'p')
	{
		paused = !paused;
	}
	/*if (key == '1')
	{
		if (trainPPO)
		{
			//End the training
			zmqMessanger->Send(ParseIntoString('1'));
		}
	}*/
	/*if (key == '2')
	{
	}
	if (key == '3')
	{
	}
	if (key == '4')
	{
	}
	if (key == '5')
	{
	}
	if (key == 't')
	{
		realtimeMode = !realtimeMode;
	}
	if (key == ' ')
	{
		throwSphere();
	}*/
	/*if (key == 'i')
	{
		applyImpulse = true;
	}
	if (key == 'w')
	{
		switchTargets = true;
	}*/
	if (key == 'r')
	{
		saveOdeState(masterContext, resetSaveSlot);
		if (useAcceleration)
			targetSpeed *= 0;
	}
	if (key == 'v')
	{
		visualizePPO = !visualizePPO;
	}

	if (key == 27) { //Escape
		autoExitAt = frameIdx;
	}

	/*if (key == 'n') {
		run_on_neural_network = true;
	}
	if (key == 'm') {
		run_on_neural_network = false;
	}

	if (key == 'o') {
		write_vectors_to_file("controls.txt", control_sequence);
		write_vectors_to_file("ml_controls.txt", machine_learning_control_sequence);
		std::system("python density_plotter.py");
	}

	if (key == 'l') {
		write_vectors_to_file("controls.txt", control_sequence);
		std::system("python sequence_plotter.py");
	}*/
}

void EXPORT_API rcOnKeyUp(int key)
{
}

float computeStateCost(const TestRig &character, bool debugOutput = false)
{
	float result = 0;
	Vector3f com;
	character.computeCOM(com);

	Vector3f target_dir = get_target_dir(com);
	const Vector3f initialDir(0, -1, 0);
	Quaternionf targetRotation = Quaternionf::FromTwoVectors(initialDir, target_dir);

	//bone angle diff from initial
	for (size_t i = 0; i < character.bones.size(); i++)
	{
		OdeRig::Bone *b = character.bones[i];
		dQuaternion q;
		dQfromR((dReal *)&q, odeBodyGetRotation(b->body));
		Quaternionf curQ;
		curQ.x() = q[0];
		curQ.y() = q[1];
		curQ.z() = q[2];
		curQ.w() = q[3];
		result += squared(curQ.angularDistance(targetRotation*b->initialRotation) / character.angleSd[i]);
	}


	//com over feet
	if (scoreAngles)
	{
		Vector3f meanFeet = Vector3f::Zero();
		int num_feet = 0;

		float foot_max_height = -1000.0f;

		for (int i : feet_bones)
		{

			const Eigen::Vector3f footPos(odeBodyGetPosition(character.bones[i]->body));

			foot_max_height = max(foot_max_height, footPos.z());

			meanFeet += footPos;
			++num_feet;
		}

		meanFeet /= (float)num_feet;
		meanFeet.z() = foot_max_height;

		Vector3f comDiff = com - meanFeet;
		comDiff.z() = min(comDiff.z(), 0.0f); //only penalize vertical difference if a foot higher than com (prevents the cheat where character lies down and holds legs up over com)
		result += comDiff.squaredNorm() / squared(comDiffSd);
	}

	//COM vel difference
	Vector3f vel;
	character.computeMeanVel(vel);
	Vector3f targetVel = targetSpeed*target_dir;
	Vector3f velDiff = vel - targetVel;
	velDiff.z() = min(velDiff.z(), 0.0f);  //don't penalize upwards deviation
	result += velDiff.squaredNorm() / squared(velSd);
	return result;
}

bool fallen()
{
	for (int bone : non_ground_contact_bones)
	{	
		Vector3f pos, normal, vel;
		pos.setZero();
		normal.setZero();
		vel.setZero();
		bool contact = odeGetGeomContact(character.bones[bone]->geom, groundPlane, pos.data(), normal.data(), vel.data());

		if (contact) {
			return true;
		}
	}

	/*Vector3f meanFeet = Vector3f::Zero();
	int num_feet = 0;
	for (int i : feet_bones)
	{
		const Eigen::Vector3f footPos(odeBodyGetPosition(character.bones[i]->body));
		meanFeet += footPos;
		++num_feet;
	}

	meanFeet /= (float)num_feet;
	meanFeet.z() = 0;

	Vector3f com;
	character.computeCOM(com);
	com.z() = 0;

	if ((com - meanFeet).squaredNorm() > 0.16f)
	{
		return true;
	}*/
	return false;
}

bool recovered()
{
	return !fallen();
}


void applyControl(const float *control)
{
	float control2[256];
	for (int i = 0; i < character.controlDim; ++i)
		control2[i] = clampf(control[i], controlMin[i], controlMax[i]);
		//control2[i] = control[i];
	float currentAngles[256];
	character.getCurrentMotorAngles(currentAngles);
	for (int i = 0; i < character.nTotalDofs; i++)
	{
		control2[i] = (control[i] - currentAngles[i]) *  poseSpringConstant;
	}
	character.applyControl(control2);

	character.setFmaxForAllMotors(defaultFmax);
	character.setMotorSpringConstants(springKp, springDamping);

	if (enableRecoveryMode)
	{
		bool allow_changing_spring_constants = true;
		bool allow_changing_fmax = true;

		if (flc->sampling_mode_ == SCAControl::SamplingMode::KL) {
			allow_changing_spring_constants = false;
			allow_changing_fmax = false;
		}

		if (inRecoveryMode) {
			if (allow_changing_fmax) {
				character.setFmaxForAllMotors(recoveryModeFmax);
			}
			if (allow_changing_spring_constants) {
				character.setMotorSpringConstants(recoveryModeSpringKp, springDamping);
			}
		}
	}
}

void KeepCharacterCloseToOrigin(bool override = false, int contextIndex = -1)
{
	Vector3f com;
	character.computeCOM(com);
	if (override || com.norm() > maxDistanceFromOrigin)
	{
		for (size_t i = 0; i < character.bones.size(); ++i)
		{
			Vector3f pos(odeBodyGetPosition(character.bones[i]->body));
			odeBodySetPosition(character.bones[i]->body, pos.x() - com.x(), pos.y() - com.y(), pos.z());
		}
		if(contextIndex >= 0)
			saveOdeState(contextIndex, contextIndex);
	}
}

void DrawCharacterInUnity()
{
	LogFrameInfo(frameIdx, runningPPO, finishedPPO, nEpochs);
	for (int i = 0; i < TestRig::Bones::bCount; ++i)
	{
		float radius, length;
		odeGeomCapsuleGetParams(character.bones[i]->geom, radius, length);
		rcDrawCapsule(odeBodyGetPosition(character.bones[i]->body), odeBodyGetRotation(character.bones[i]->body), length, radius, true);
	}
}

static float previousCost = 10000.0f;
static int fallCount = 0;

void EXPORT_API rcOnMouse(float, float, float, float, float, float, int, int, int) {

}

void EXPORT_API rcUpdate()
{
	if (frameIdx == 0)
	{
		for (int i = 0; i < character.bones.size(); ++i)
		{
			float radius, length;
			odeGeomCapsuleGetParams(character.bones[i]->geom, radius, length);
			rcSetupBodyInUnity(i, odeBodyGetPosition(character.bones[i]->body), odeBodyGetRotation(character.bones[i]->body), length, radius);
		}
	}
	if (rigTestMode)
	{
		static bool setVP = true;
		if (setVP)
		{
			setVP = false;
			rcSetViewPoint(1, -2.0, 2.0, 0, 0, 1.5);
			rcSetLightPosition(-2.5, -1, 1.5);
		}
		setCurrentOdeContext(masterContext);
		poseEditor->Update(frameIdx);
		stepOde(timeStep, false);
	}
	else
	{
		if (!trainPPO || !runningPPO)
		{
			if (flc->sampling_mode_ == SCAControl::SamplingMode::KL) {
				use_external_prior = false;
			}

			//accelerated target velocity
			if (useAcceleration)
			{
				if (frameIdx == 0)
					targetSpeed *= 0;
				targetSpeed += acceleration * timeStep;
			}

			//walk targets
			if (useWalkTargets)
			{
				//pick next target location and velocity
				Vector3f com;
				character.computeCOM(com);
				com.z() = 0;

				auto random_walk_target = []() {
					Eigen::Vector3f target;

					target.setRandom();

					float dist_from_origin = 5.0f;

					target.z() = 0;
					target.normalize();
					target *= dist_from_origin;

					return target;

				};

				if (frameIdx % walk_time == 0 || frameIdx == 0 || switchTargets) {
					walkTarget = random_walk_target();
					switchTargets = false;
				}

				float distToTarget = (walkTarget - com).norm();
				while (distToTarget < 1.5f)
				{
					walkTarget = random_walk_target();
					distToTarget = (walkTarget - com).norm();
				}
			}

			//visualize target as a "pole"
			dMatrix3 R;
			dRSetIdentity(R);
			rcSetColor(0.5f, 1, 0.5f);
			//rcDrawCapsule(walkTarget.data(), R, 20.0f, 0.02f);
			rcSetColor(1, 1, 1);

			//multitasking with random velocity
			static float timeOnTask = 0;
			timeOnTask += timeStep;
			if (multiTask && (timeOnTask > 5.0f))
			{
				float r = (float)randInt(-1, 1);
				targetSpeed = r;
				printf("New target vel %f\n", targetSpeed);
				timeOnTask = 0;
			}

			//setup current character state
			setCurrentOdeContext(masterContext);
			restoreOdeState(masterContext);
			VectorXf startState(stateDim);
			computeStateVector(&startState[0]);

			int currentTrajectories = nTrajectories;
			int learning = true;

			if (realtimeMode) {
				currentTrajectories = nRealtimeTrajectories;
				learning = false;

				flc->machine_learning_samples_ = 2;
				flc->noisy_machine_learning_samples_ = 2;
				flc->nn_trajectories_ = 2;

			}
			else {

				flc->machine_learning_samples_ = orig_ml_trajectories;
				flc->noisy_machine_learning_samples_ = orig_noisy_ml_trajectories;
				flc->nn_trajectories_ = orig_nearest_neighbor_trajectories;

			}

			flc->setParams(previousCost*resampleThreshold, learning, currentTrajectories);

			flc->control_diff_std_ = controlSd;
			flc->static_prior_.mean[0] = init_motor_angles;

			endTimer = std::chrono::system_clock::now();

			std::chrono::duration<double> duration = endTimer - startTimer;
			int seconds = (int)duration.count() % 60;
			int minutes = (int)duration.count() / 60;

			//Amin
			VectorXf scaledControlSd = controlSd;
			controlDiffSd = controlSd;

			if (inRecoveryMode)
			{
				scaledControlSd *= recoveryModePoseSdMult;
				controlDiffSd *= recoveryModePoseSdMult;
			}
			else {
				flc->control_min_ = minControls;
				flc->control_max_ = maxControls;
			}

			flc->setSamplingParams(scaledControlSd.data(), controlDiffSd.data(), mutationSd);

			startPerfCount();

			float impulse[3];
			int impulseBone;
			bool drawImpulse = false;

			bool willFall = false;
			if (!rigTestMode)
			{

				if (!run_on_neural_network) {

					rcPrintString("Number of trajectories: %d %s", currentTrajectories, realtimeMode ? "(Realtime-mode)" : "");
					rcPrintString("Planning horizon: %.1f seconds", planningHorizonSeconds);
					rcPrintString("Simulation time %02d:%02d:%2d (%d frames)", frameIdx / (fps * 60), (frameIdx / fps) % 60, frameIdx % fps, frameIdx);
					rcPrintString("Pruning threshold %.1f", previousCost*resampleThreshold);

					flc->startIteration(!onlyAdvanceIfNotFalling || (fallCount == 0), &startState[0]);

					for (int step = 0; step < nTimeSteps; step++)
					{
						int nUsedTrajectories = flc->getNumTrajectories();
						flc->startPlanningStep(step);
						if (step == 0)
						{
							for (int i = 0; i < nUsedTrajectories; i++)
							{
								saveOdeState(contexts[i].trajectoryIdx, masterContext);
							}
						}
						else
						{
							for (int i = 0; i < nUsedTrajectories; i++)
							{
								saveOdeState(contexts[i].trajectoryIdx, contexts[i].trajectoryIdx);
							}

						}

						std::deque<std::future<void>> workers;

						for (int t = nUsedTrajectories - 1; t >= 0; t--)
						{
							//lambda to be executed in the thread of the simulation context
							auto controlStep = [step](int data) {
								if (frameIdx == 0 && step == 0 && useThreads)
								{
									allocateODEDataForThread();
									odeRandSetSeed(randomSeed);
								}
								SimulationContext &c = contexts[data];
								setCurrentOdeContext(c.trajectoryIdx);
								restoreOdeState(flc->getPreviousSampleIdx(c.trajectoryIdx));


								if (use_external_prior) {
									//pose prior (towards zero angles)
									character.getCurrentMotorAngles(c.priorMean.data());

									//In pose-based control we use the extra prior to limit acceleration.
									//We first compute the predicted pose based on current pose and motor angular velocities, and then
									//set the prior there.
									c.angleRates.setZero();
									character.getCurrentAngleRates(c.angleRates.data());
									c.priorMean = c.priorMean + c.angleRates / poseSpringConstant;
									c.priorMean = c.priorMean.cwiseMax(controlMin);
									c.priorMean = c.priorMean.cwiseMin(controlMax);
									c.priorSd.setConstant(controlAccSd * (inRecoveryMode ? recoveryModeAccSdMult : 1.0f));
								}

								//sample control
								if (use_external_prior) {
									flc->getControl(c.trajectoryIdx, &c.control[0], c.priorMean.data(), c.priorSd.data());
								}
								else {
									flc->getControl(c.trajectoryIdx, &c.control[0], nullptr, nullptr);
								}
								//step physics

								if (character_root_bone >= 0) {
									c.initialPosition = Eigen::Vector3f(odeBodyGetPosition(character.bones[character_root_bone]->body));
								}
								else {
									character.computeCOM(c.initialPosition);
								}

								bool broken = false;

								float controlCost = 0;
								applyControl(&c.control[0]);
								broken = !stepOde(timeStep, false);
								if (broken)
								{
									restoreOdeState(flc->getPreviousSampleIdx(c.trajectoryIdx));
								}

								controlCost += character.getAppliedSqJointTorques();
								controlCost /= squared(defaultFmax);

								if (character_root_bone >= 0) {
									c.resultPosition = Eigen::Vector3f(odeBodyGetPosition(character.bones[character_root_bone]->body));
								}
								else {
									character.computeCOM(c.resultPosition);
								}

								if (!broken)
								{
									float brokenDistanceThreshold = 0.25f;
									if ((c.resultPosition - c.initialPosition).norm() > brokenDistanceThreshold)
									{
										restoreOdeState(flc->getPreviousSampleIdx(c.trajectoryIdx));
										c.resultPosition = c.initialPosition;
										broken = true;
									}
								}

								//evaluate state cost
								float stateCost = computeStateCost(character);
								if (broken)
									stateCost += 1000000.0f;
								computeStateVector(&c.stateFeatures[0]);

								flc->updateResults(c.trajectoryIdx, c.control.data(), c.stateFeatures.data(), stateCost + controlCost);
								c.stateCost = stateCost;
								c.controlCost = controlCost;
							};
							if (!useThreads)
								controlStep(t);
							else
								workers.push_back(std::async(std::launch::async, controlStep, t));
						}
						if (useThreads)
						{
							for (std::future<void>& worker : workers) {
								worker.wait();
							}
						}

						flc->endPlanningStep(step);

						bool plot_trajectories = true;

						if (plot_trajectories) {
							//debug visualization
							for (int t = nUsedTrajectories - 1; t >= 0; t--)
							{
								SimulationContext &c = contexts[t];
								if (flc->experience_[step + 1][t].particleRole == ParticleRole::OLD_BEST) {
									rcSetColor(0, 1, 0, 1);
								}
								else if (flc->experience_[step + 1][t].particleRole == ParticleRole::NEAREST_NEIGHBOR)
								{
									rcSetColor(0, 0, 1, 1);
								}
								else
								{
									rcSetColor(1, 1, 1, 1.0f);
								}
								rcDrawLine(c.initialPosition.x(), c.initialPosition.y(), c.initialPosition.z(), c.resultPosition.x(), c.resultPosition.y(), c.resultPosition.z());
							}
						}
						rcSetColor(1, 1, 1, 1);
					}

					flc->endIteration();

					//print profiling info
					int controllerUpdateMs = getDurationMs();
					rcPrintString("Controller update time: %d ms", controllerUpdateMs);

					//check whether best trajectory will fall
					setCurrentOdeContext(contexts[flc->getBestSampleLastIdx()].trajectoryIdx);

					willFall = fallen();
					if (willFall)
						fallCount++;
					else
						fallCount = 0;
				}

				if (useAcceleration)
					rcPrintString("Target speed: %.2f m/s", fabs(targetSpeed));

				//step master context
				setCurrentOdeContext(masterContext);
				restoreOdeState(masterContext);

				if (run_on_neural_network) {
					bool on_the_ground = fallen();
					if (on_the_ground) {
						fallCount++;
					}
					else {
						fallCount = 0;
					}
				}

				Eigen::VectorXf machine_learning_control = contexts[masterContext].control;

				computeStateVector(contexts[masterContext].stateFeatures.data());
				flc->getMachineLearningControl(contexts[masterContext].stateFeatures.data(), machine_learning_control.data());

				if (run_on_neural_network) {
					contexts[masterContext].control = machine_learning_control;
				}
				else {
					flc->getBestControl(0, contexts[masterContext].control.data());
				}

				machine_learning_control_sequence.push_back(machine_learning_control);
				control_sequence.push_back(contexts[masterContext].control);

				float best_trajectory_cost = (float)flc->getBestTrajectoryCost();

				//apply control (and random impulses)
				applyControl(contexts[masterContext].control.data());
				if (!trainPPO && useRandomImpulses && !realtimeMode)
				{
					--randomImpulseTimer;
					if (randomImpulseTimer == 0)
					{
						for (int d = 0; d < 3; ++d)
							impulse[d] = lerp(randomImpulseMagnitudeMin, randomImpulseMagnitudeMax, randomf());
						impulse[2] /= 5.0f;
						impulseBone = randInt(0, TestRig::Bones::bCount - 1);
						drawImpulse = true;
						odeBodyAddForce(character.bones[impulseBone]->body, impulse);
						randomImpulseTimer = randInt(randomImpulseIntervalMin, randomImpulseIntervalMax);
					}
				}
				stepOde(timeStep, false);

				character.UpdateRunningAverageLinearVelocity();

				if (!onlyAdvanceIfNotFalling || (fallCount == 0)) //only progress if not fallen
					saveOdeState(masterContext, masterContext);

				float controlCost = character.getAppliedSqJointTorques();
				controlCost /= squared(defaultFmax);

				previousCost = contexts[flc->getBestSampleLastIdx()].stateCost;
				float state_cost = computeStateCost(character);
				rcPrintString("state cost %.2f, current control cost %.2f, end state cost %.2f\n", state_cost, controlCost, previousCost);

				std::vector<float> cost;
				cost.push_back(best_trajectory_cost);
				cost.push_back(state_cost + controlCost);

				flc->getMachineLearningChosenControlDiscrepancy();
				cost.push_back(flc->machine_learning_and_chosen_control_discrepancy_);

				float ml_mse = flc->getMachineLearningMSE();
				cost.push_back(ml_mse);

				costs.push_back(cost);
			} //!rigTestMode
			setCurrentOdeContext(masterContext);

			//in simple forward walking without targets, if moved too far from origin, move back
			if (!useWalkTargets)
			{
				KeepCharacterCloseToOrigin(false, masterContext);
			}

			const bool reset_on_fall = true;

			if (reset_on_fall) {
				bool on_the_ground = fallen();
				if (on_the_ground) {

					saveOdeState(masterContext, resetSaveSlot);
					run_on_neural_network = false;

					for (int i = 0; i <= nTrajectories; i++)
					{
						setCurrentOdeContext(contexts[i].trajectoryIdx);
						restoreOdeState(resetSaveSlot);
						saveOdeState(contexts[i].trajectoryIdx, contexts[i].trajectoryIdx);
					}

					fallCount = 0;
					if (useAcceleration)
						targetSpeed *= 0;
				}
			}
			else {
				//check for falling
				if (fallCount > (int)((inRecoveryMode ? recoveryModeTimeUntilReset : 5.0f) / timeStep))
				{
					saveOdeState(masterContext, resetSaveSlot);

					run_on_neural_network = false;


					for (int i = 0; i <= nTrajectories; i++)
					{
						setCurrentOdeContext(contexts[i].trajectoryIdx);
						restoreOdeState(resetSaveSlot);
						saveOdeState(contexts[i].trajectoryIdx, contexts[i].trajectoryIdx);
					}

					fallCount = 0;
					if (useAcceleration)
						targetSpeed *= 0;

				}
			}

			static bool setVP = true;
			if (setVP)
			{
				setVP = false;
				rcSetViewPoint(5.0f, -maxDistanceFromOrigin / 2.0f, 1.25f, 0, -maxDistanceFromOrigin / 2.0f, 0.9f);
				rcSetLightPosition(0, -10, 10);
			}
			DrawCharacterInUnity();
			if (drawImpulse)
			{
				rcDrawForceInUnity(0, impulse);
			}
			rcDrawAllObjects((dxSpace *)odeGetSpace());
			//character.debugVisualize();

			//state transitions
			if (willFall)
			{
				rcPrintString("Falling predicted!");
				if (enableRecoveryMode)
					inRecoveryMode = true;
			}
			if (!willFall && recovered())
				inRecoveryMode = false;
			if (inRecoveryMode) {
				if (flc->sampling_mode_ == SCAControl::SamplingMode::L1) {
					rcPrintString("Recovery mode with stronger movements and wider search distributions.");
				}

				if (flc->sampling_mode_ == SCAControl::SamplingMode::KL) {
					rcPrintString("Recovery mode with wider search distributions.");
				}
			}

			if (no_settings_exit) {
				exit(0);
			}

			//spheres
			if (useSpheres && (frameIdx > lastSphereLaunchTime + sphereInterval))
			{
				Vector3f com;
				character.computeCOM(com);
				if (useWalkTargets || (com.x() > -maxDistanceFromOrigin + 2.0f) && (com.x() < -0.5f))  //don't throw a sphere right before character is about to be teleported
					throwSphere();
			}
		}
		else
		{
			//Run PPO algorithm

			if (!finishedPPO)
			{
				if (curriculumLearningPPO)
					rewardThreshold = lerp(curriculumInitialThreshold, curriculumFinalThreshold, float(nEpochs) / maxIterationsPPO);
				else
					rewardThreshold = 0;
			}

			static VectorXf stateQueryArray = VectorXf::Zero((nTrajectories + 1) * stateDim);
			static VectorXf policyActionArray = VectorXf::Zero((nTrajectories + 1) * character.controlDim);
			static VectorXf rewardsAndCosts = VectorXf::Zero(2 * (nTrajectories + 1));
			static VectorXi episodesDone = VectorXi::Zero(nTrajectories + 1);
			static VectorXi episodesLen = VectorXi::Zero(nTrajectories + 1);
			static VectorXi randomImpulseTimers = -VectorXi::Ones(nTrajectories + 1);

			// Report whether each episode has been done(= 1) or not (= 0)
			if (!renderingPolicy)
				zmqMessanger->Send(ParseIntoString('D', episodesDone.data(), nTrajectories + 1));

			int nRunningEpisodes = nTrajectories + 1 - episodesDone.sum();
			if (renderingPolicy)
				nRunningEpisodes = 1;

			/*if (false && randomf() < 0.02f)
			{
				throwSphere();
			}*/

			//Compute state vectors for all trajectories
			for (int i = 0, counter = 0; i <= nTrajectories; ++i)
			{
				if ((!renderingPolicy && episodesDone[i] == 0) || (renderingPolicy && i == masterContext))
				{
					setCurrentOdeContext(i);
					restoreOdeState(i);
					computeStateVector(&stateQueryArray.data()[counter * stateDim]);
					++counter;
					stateQueryArray[counter * stateDim - 1] = float(referenceMotionSlot[i] - resetSaveSlot - 1) / float(nStateSamplesForPPO - 1);
				}
			}

			//Query policy network using stateQueryArray
			ParseStringIntoFloatArray(zmqMessanger->Send(ParseIntoString('A', stateQueryArray.data(), nRunningEpisodes * stateDim)), policyActionArray.data());

			//Execute actions and compute rewards
			std::deque<std::future<void>> workers;
			for (int i = 0, counter = 0; i <= nTrajectories; ++i)
			{
				auto controlStep = [](int contextIdx, int dataIdx)
				{
					setCurrentOdeContext(contextIdx);
					restoreOdeState(contextIdx);
					if (!renderingPolicy)
					{
						++referenceMotionSlot[contextIdx];
						if (referenceMotionSlot[contextIdx] == resetSaveSlot + nStateSamplesForPPO + 1)
						{
							referenceMotionSlot[contextIdx] = resetSaveSlot + 1;
							KeepCharacterCloseToOrigin(true, contextIdx);
						}
					}

					applyControl(&policyActionArray.data()[dataIdx * character.controlDim]);

					float impulse[3];
					int impulseBone;
					bool drawImpluse = false;

					if (useRandomImpulses)
					{
						if (randomImpulseTimers[contextIdx] < 0)
						{
							randomImpulseTimers[contextIdx] = randInt(randomImpulseIntervalMin, randomImpulseIntervalMax);
						}

						--randomImpulseTimers[contextIdx];

						if (randomImpulseTimers[contextIdx] == 0)
						{
							for (int d = 0; d < 3; ++d)
								impulse[d] = lerp(randomImpulseMagnitudeMin, randomImpulseMagnitudeMax, randomf());
							impulse[2] /= 5.0f;
							impulseBone = randInt(0, TestRig::Bones::bCount - 1);
							odeBodyAddForce(character.bones[impulseBone]->body, impulse);
							drawImpluse = renderingPolicy;

							randomImpulseTimers[contextIdx] = randInt(randomImpulseIntervalMin, randomImpulseIntervalMax);
						}
					}

					bool broken = !stepOde(timeStep, false);

					float taskReward = 0, imitationReward = 0;
					if (!renderingPolicy)
					{
						Vector3f com;
						character.computeCOM(com);

						/*
						static const bool useNegCostAsTaskReward = true;
						static const bool useImitationReward = true;
						*/

						rewardsAndCosts[2 * dataIdx + 1] = computeStateCost(character) + character.getAppliedSqJointTorques() / squared(defaultFmax);

						if (useNegCostAsTaskReward)
						{
							taskReward = (10000.0f - rewardsAndCosts[2 * dataIdx + 1]) / 10000.0f;
							if (taskReward < 0)
							{
								cout << "Negative reward, epoch = " << nEpochs << ", cost = " << rewardsAndCosts[2 * dataIdx + 1] << endl;
								taskReward = 0;
							}
							//taskReward = exp(-rewardsAndCosts[2 * dataIdx + 1] / 500.0f);
						}
						else
						{
							Vector3f meanVel;
							character.computeMeanVel(meanVel);
							meanVel.z() = 0;
							Vector3f targetVel = get_target_dir(com) * targetSpeed;
							Vector3f velDiff = targetVel - meanVel;
							taskReward = exp(-2.5f * velDiff.squaredNorm());
						}

						if (useImitationReward)
						{
							imitationReward = ComputeImitationReward(referenceMotionSlot[contextIdx]);
							rewardsAndCosts[2 * dataIdx] = 0.3f * taskReward + 0.7f * imitationReward;
						}
						else
						{
							
							rewardsAndCosts[2 * dataIdx] = taskReward;
						}
					}

					saveOdeState(contextIdx, contextIdx);

					if (renderingPolicy)
					{
						DrawCharacterInUnity();
						if (drawImpluse)
						{
							rcDrawForceInUnity(impulseBone, impulse);
						}
					}
					if (dataIdx == 0 && (renderingPolicy || visualizePPO || nEpochs % 100 == 0))
					{
						//Visualize current reference frame
						if (false)
						{
							restoreOdeState(referenceMotionSlot[contextIdx]);
							rcSetColor(0.0, 0.0, 0.15, 0.15);
							rcDrawAllObjects((dxSpace *)odeGetSpace(), false);
							restoreOdeState(contextIdx);
						}

						rcSetColor(1, 1, 1, 1);
						rcDrawAllObjects((dxSpace *)odeGetSpace(), false);

						rcPrintString("Episode Length = %d", episodesLen[contextIdx]);
						if (!renderingPolicy)
						{
							if (curriculumLearningPPO)
								rcPrintString("Minimum Reward Threshold = %.2f", rewardThreshold);
							rcPrintString("Context #%d:", contextIdx);
							if(useImitationReward)
								rcPrintString("Task reward = %.2f, Imitation reward = %.2f", taskReward, imitationReward);
							rcPrintString("Reward = %.2f", rewardsAndCosts[2 * dataIdx]);
						}
					}

					++episodesLen[contextIdx];

					bool forceEpisodeEndForPPO = false;
					if (renderingPolicy)
					{
						if (episodesLen[contextIdx] >= maxEpisodesLenRender)
						{
							forceEpisodeEndForPPO = true;
						}
					}
					else
					{
						if (episodesLen[contextIdx] >= maxEpLenPPO || (!evaluatingPolicy && curriculumLearningPPO && rewardsAndCosts[2 * dataIdx] < rewardThreshold))
						{
							forceEpisodeEndForPPO = true;
						}
					}

					if (forceEpisodeEndForPPO || broken || fallen())
					{
						//Episode over, reset the trajectory
						if (false)
						{
							referenceMotionSlot[contextIdx] = resetSaveSlot + 1;
						}
						else
						{
							referenceMotionSlot[contextIdx] = resetSaveSlot + randInt(1, nStateSamplesForPPO);
						}
						restoreOdeState(referenceMotionSlot[contextIdx]);
						saveOdeState(contextIdx, contextIdx);
						episodesDone[contextIdx] = 1;
						episodesLen[contextIdx] = 0;
						if(renderingPolicy)
						{
							renderedEpisodes++;
						}
					}
				};
				if ((!renderingPolicy && episodesDone[i] == 0) || (renderingPolicy && i == masterContext))
				{
					if (!useThreads)
						controlStep(i, counter);
					else
						workers.push_back(std::async(std::launch::async, controlStep, i, counter));
					++counter;
				}
			}
			if (useThreads)
			{
				for (std::future<void>& worker : workers) {
					worker.wait();
				}
			}

			if (!renderingPolicy)
				zmqMessanger->Send(ParseIntoString('R', rewardsAndCosts.data(), 2 * nRunningEpisodes));

			if ((renderingPolicy && episodesDone[masterContext]) || episodesDone.sum() == nTrajectories + 1)
			{
				episodesDone.setZero();
				episodesLen.setZero();
				if (evaluatingPolicy)
				{
					evaluatingPolicy = false;
					string reply = zmqMessanger->Send(ParseIntoString('1'));
					renderingPolicy = reply.compare("rendering") == 0;
				}
				else if (renderingPolicy)
				{
					if (renderedEpisodes == maxEpisodesRender)
					{
						renderingPolicy = false;
						if(finishedPPO)
							autoExitAt = frameIdx;
						else
							string reply = zmqMessanger->Send(ParseIntoString('2'));
						renderedEpisodes = 0;
					}
				}
				else
				{
					string reply = zmqMessanger->Send(ParseIntoString('T'));
					if (reply.compare("final_evaluating") == 0)
						finishedPPO = evaluatingPolicy = true;
					else if (reply.compare("evaluating") == 0)
						evaluatingPolicy = true;
					else if (reply.compare("rendering") == 0)
						renderingPolicy = true;
					if (reply.compare("skip") != 0)
						++nEpochs;
				}
			}
		}

		if (trainPPO && !runningPPO && nStateSamplesForPPO > 0)
		{
			if (visualizePPO || nEpochs % 100 == 0)
			{
				rcSetColor(0.0, 0.0, 0.15, 0.15);
				setCurrentOdeContext(resetSaveSlot);
				restoreOdeState(resetSaveSlot + 1);
				rcDrawAllObjects((dxSpace *)odeGetSpace(), false);
				if (runningPPO)
				{
					restoreOdeState(resetSaveSlot + nStateSamplesForPPO);
					rcDrawAllObjects((dxSpace *)odeGetSpace(), false);
				}
			}
			setCurrentOdeContext(masterContext);
			restoreOdeState(masterContext);
		}

		for (size_t i = 0; i < spheres.size(); i++)
		{
			SphereData &sd = spheres[i];
			if (sd.spawnFrame < frameIdx - 3 * fps)
			{
				setCurrentOdeContext(ALLTHREADS);
				odeGeomDestroy(sd.geom);
				odeBodyDestroy(sd.body);
				saveOdeState(masterContext, masterContext);
				spheres[i] = spheres[spheres.size() - 1];
				spheres.resize(spheres.size() - 1);
			}
		}

		if (trainPPO && !runningPPO && frameIdx >= startStateSamplingForPPOAt)
		{
			setCurrentOdeContext(masterContext);
			restoreOdeState(masterContext);

			if(nStateSamplesForPPO == 0)
				KeepCharacterCloseToOrigin(true, masterContext);

			bool cycleFinished = false;
			if (nStateSamplesForPPO >= 10)
			{
				MotionState *firstFrame, *currentFrame;
				firstFrame = motionStates[resetSaveSlot + 1];
				currentFrame = motionStates[resetSaveSlot + nStateSamplesForPPO];
				int simCount = 0;
				for (int i = 0; i < countEndEffectors; ++i)
				{
					if ((firstFrame->endEffectorsPos[i] - currentFrame->endEffectorsPos[i]).norm() < 0.05f)
					{
						simCount++;
					}
					if (firstFrame->endEffectorsVel[i].dot(currentFrame->endEffectorsVel[i]) > 0)
					{
						simCount++;
					}
				}
				//cout << simCount << endl;
				if (simCount >= 2 * countEndEffectors - 1)
				{
					cout << "Detected a cycle with " << nStateSamplesForPPO << " frames (similarity = " << simCount << ")" << endl;
					cycleFinished = true;
					runningPPO = true;
					//Initialize all trajectories to a valid state
					for (int i = 0; i <= nTrajectories; i++)
					{
						setCurrentOdeContext(contexts[i].trajectoryIdx);
						referenceMotionSlot[i] = resetSaveSlot + randInt(1, nStateSamplesForPPO);
						restoreOdeState(referenceMotionSlot[i]);
						saveOdeState(contexts[i].trajectoryIdx, contexts[i].trajectoryIdx);
					}
				}
			}

			if (!cycleFinished)
			{
				int slotIdx = resetSaveSlot + nStateSamplesForPPO + 1;
				saveOdeState(slotIdx, masterContext);
				SaveMotionState(slotIdx);
				++nStateSamplesForPPO;
			}
		}
	}

	//quit 
	if (frameIdx > autoExitAt)
	{
		/*std::deque<std::string> settings = flc->get_settings();

		for (std::string setting : settings) {
			comments.push_back(setting);
		}

		comments.push_back("Turns: " + std::to_string(useWalkTargets));

		cost_file_name = get_time_string() + "_costs.csv";
		write_vector_to_file(cost_file_name, costs, comments);*/

		if (zmqMessanger)
		{
			zmqMessanger->Send(ParseIntoString('E'));
			delete zmqMessanger;
		}
		rcSaveCommandLogs(character.commandLogFileName.data());
		exit(0);
	}
	frameIdx++;
}