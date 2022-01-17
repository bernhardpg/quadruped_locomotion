#pragma once

const int kNumPoseCoords = 7;
const int kNumTwistCoords = 6;
const int kNumJoints = 12;
const int kNumGenCoords = kNumPoseCoords + kNumJoints;
const int kNumGenVels = kNumTwistCoords + kNumJoints;
const int kNumLegs = 4;
const int kNumPosDims = 3; // TODO: change name to k3D
const int k2D = 2;
const int kNumFeetCoords = kNumLegs * kNumPosDims;

typedef Eigen::Matrix<double,kNumJoints,1> joint_vector_t;
typedef Eigen::Matrix<double,kNumGenCoords,1> gen_coord_vector_t;
typedef Eigen::Matrix<double,kNumGenVels,1> gen_vel_vector_t;
typedef Eigen::Matrix<double,kNumFeetCoords,1> feet_vector_t;

const std::vector<std::string> kFeetFrames = 
{
	"LF_FOOT",
	"RF_FOOT",
	"LH_FOOT",
	"RH_FOOT"
};

const std::vector<std::string> kJointNames =
{
	"LF_HAA",
	"RF_HAA",
	"LH_HAA",
	"RH_HAA",
	"LF_HFE",
	"RF_HFE",
	"LH_HFE",
	"RH_HFE",
	"LF_KFE",
	"RF_KFE",
	"LH_KFE",
	"RH_KFE"
};
 // Foot ordering: LF LH RF RH
const Eigen::MatrixXd initial_joint_config = (Eigen::MatrixXd(kNumJoints,1) << 
		0,0,0,0,
		2,2,-2,-2,
		-2.5,-2.5,2.5,2.5)
.finished();

