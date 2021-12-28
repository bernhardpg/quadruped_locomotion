#pragma once

const int kNumPoseCoords = 7;
const int kNumTwistCoords = 6;
const int kNumJoints = 12;
const int kNumGenCoords = kNumPoseCoords + kNumJoints;
const int kNumGenVels = kNumTwistCoords + kNumJoints;
const int kNumLegs = 4;
const int kNumPosDims = 3;
const int kNumFeetCoords = kNumLegs * kNumPosDims;

typedef Eigen::Matrix<double,kNumJoints,1> joint_vector_t;
typedef Eigen::Matrix<double,kNumGenCoords,1> gen_coord_vector_t;
typedef Eigen::Matrix<double,kNumGenVels,1> gen_vel_vector_t;
typedef Eigen::Matrix<double,kNumFeetCoords,1> feet_vector_t;

const std::vector<std::string> kFeetFrames = 
{
	"LF_FOOT",
	"LH_FOOT",
	"RF_FOOT",
	"RH_FOOT"
};

const std::vector<std::string> kJointNames =
{
	"LF_HAA",
	"LF_HFE",
	"LF_KFE",
	"LH_HAA",
	"LH_HFE",
	"LH_KFE",
	"RF_HAA",
	"RF_HFE",
	"RF_KFE",
	"RH_HAA",
	"RH_HFE",
	"RH_KFE"
};
 // Foot ordering: LF LH RF RH
const Eigen::MatrixXd initial_joint_config = (Eigen::MatrixXd(kNumJoints,1) << 
	0, 2, -2.5,
	0, -2, 2.5,
	0, 2, -2.5,
	0, -2, 2.5)
.finished();

