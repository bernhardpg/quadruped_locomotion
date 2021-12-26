#include "control/controller.hpp"

// TODO: remove all 'this' references

namespace control {
	Controller::Controller(int frequency)
		: loop_rate_(frequency)
	{
		ROS_INFO("Running controller at %d Hz", frequency);
		model_name_ = "anymal"; // TODO: Replace with node argument
		
		// TODO: Move into function?
		q_.setZero();
		q_(3) = 1; // quaternions, TODO: only temp
		u_.setZero();
		q_j_cmd_.setZero();
		q_j_dot_cmd_.setZero();

		InitRos();
		SetStartTime();
		SpinRosThreads();

		// Wait for state to get published
		//while (q_.isZero(0) || u_.isZero(0))

		InitController();

		CreatePreStandupTraj();
		controller_ready_ = true; // TODO: cleanup
	}

	Controller::~Controller()
	{
			ros_node_.shutdown();

			ros_process_queue_.clear();
			ros_process_queue_.disable();
			ros_process_queue_thread_.join();

			ros_publish_queue_.clear();
			ros_publish_queue_.disable();
			ros_publish_queue_thread_.join();
	}

	void Controller::InitController()
	{
		feet_pos_error_.setZero();
		feet_vel_ff_.setZero();

		q_j_cmd_.setZero();
		q_j_dot_cmd_.setZero();

		q_j_dot_cmd_integrator_ = Integrator(kNumJoints_);
		q_j_dot_cmd_integrator_.Reset();
	}

	// **************** //
	// STANDUP SEQUENCE //
	// **************** //

	void Controller::CreatePreStandupTraj()
	{
		Eigen::MatrixXd curr_config(12,1); // LF LH RF RH
		curr_config = q_.block<12,1>(7,0);

		// TODO: Clean up here
		pre_standup_config_.resize(12,1);
		pre_standup_config_ <<
			0, 2, -2.5,
			0, -2, 2.5,
			0, 2, -2.5,
			0, -2, 2.5;

		q_j_dot_cmd_integrator_.SetIntegral(pre_standup_config_); // TODO: Move this to some state handler

		const std::vector<double> breaks = {
			standup_start_time_, standup_end_time_
		};

		std::vector<Eigen::MatrixXd> samples;
		samples.push_back(curr_config);
		samples.push_back(pre_standup_config_);

		q_j_ref_traj_ =
			drake::trajectories::PiecewisePolynomial<double>
			::FirstOrderHold(
					breaks, samples
					);
		q_j_dot_ref_traj_ = q_j_ref_traj_.derivative(1);
	}

	void Controller::CreateStandupTraj()
	{
		const std::vector<double> breaks = {
			standup_end_time_ + 2, standup_final_time_ + 2
		};

		Eigen::MatrixXd start_conf(12,1);
		start_conf = robot_dynamics_.GetFeetPositions(q_);

		Eigen::MatrixXd standing_conf = start_conf;
		for (int leg_i = 0; leg_i < n_legs_; ++leg_i)
			standing_conf(2 + leg_i * n_dims_) = standing_height_;

		std::cout << "start_conf:" << std::endl;
		std::cout << start_conf.transpose() << std::endl << std::endl;

		std::cout << "standing_conf:" << std::endl;
		std::cout << standing_conf.transpose() << std::endl << std::endl;

		std::vector<Eigen::MatrixXd> samples;
		samples.push_back(start_conf);
		samples.push_back(standing_conf);

		// Use drake piecewise polynomials for easy trajectory construction
		standup_pos_traj_ =
			drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
					breaks, samples
					);
		standup_vel_traj_ = standup_pos_traj_.derivative(1);

		for (double t = standup_end_time_; t < standup_final_time_; t+=0.5)
		{
			std::cout << std::fixed << std::setprecision(2)
				<< standup_pos_traj_.value(t).transpose() << std::endl << std::endl;
		}
		created_standup_traj_ = true;
	}

	void Controller::JacobianController()
	{
		t_ = GetElapsedTimeSince(start_time_);

		feet_pos_error_ =
			standup_pos_traj_.value(t_)
			- robot_dynamics_.GetFeetPositions(q_);

		feet_vel_ff_ = standup_vel_traj_.value(t_);

		J_feet_pos_ = robot_dynamics_
			.GetStackedFeetJacobianPos(q_)
			.block(0,6,12,12); // Get rid of jacobian for body
		Eigen::MatrixXd J_inv = CalcPseudoInverse(J_feet_pos_);
		
		// Position controller
		q_j_dot_cmd_ = J_inv
			* (k_pos_p_ * feet_pos_error_ + feet_vel_ff_);

		q_j_dot_cmd_integrator_.Integrate(q_j_dot_cmd_);
		q_j_cmd_ = q_j_dot_cmd_integrator_.GetIntegral();
		std::cout << std::fixed << std::setprecision(2)
			<< feet_pos_error_.transpose() << std::endl 
			<< feet_vel_ff_.transpose() << std::endl
			<< q_j_cmd_.transpose() << std::endl
			<< q_j_dot_cmd_.transpose() << std::endl << std::endl;

		SetStateToCmd();
	}

	// TODO: only for testing
	void Controller::SetStateToCmd()
	{
		q_.block<3,1>(0,0).setZero();
		q_(3) = 1;
		q_.block<3,1>(4,0).setZero();
		q_.block<12,1>(7,0) = q_j_cmd_;

		u_.block<3,1>(0,0).setZero();
		u_.block<3,1>(3,0).setZero();
		u_.block<12,1>(6,0) = q_j_dot_cmd_;
	}

	void Controller::CalcJointCmd()
	{
		t_ = GetElapsedTimeSince(start_time_);
		// TODO: clean up this
		if (t_ < standup_start_time_)
		{
			q_j_cmd_ = q_j_ref_traj_.value(standup_start_time_);
			q_j_dot_cmd_.setZero();
		}
		else if (t_ > standup_end_time_ + 2) // TODO: wait two seconds
		{
			if (!created_standup_traj_)
			{
				CreateStandupTraj(); 
			}
			if (created_standup_traj_)	
			{
				JacobianController();
			}
		} 
		else
		{
			q_j_cmd_ = q_j_ref_traj_.value(t_);
			q_j_dot_cmd_ = q_j_dot_ref_traj_.value(t_);
		}
	}

	// *** //
	// ROS //
	// *** //

	void Controller::InitRos()
	{
		ROS_INFO("Initialized controller node");

		// TODO: All of these constructions can be moved into their own functions, this is essentially just repeated code.
		// TODO: Move this joint_state publisher into its own node
		ros::AdvertiseOptions joint_state_ao =
			ros::AdvertiseOptions::create<sensor_msgs::JointState>(
					"/joint_states",
					1,
					ros::SubscriberStatusCallback(),
					ros::SubscriberStatusCallback(),
					ros::VoidPtr(),
					&this->ros_publish_queue_
					);

		joint_state_pub_ = ros_node_.advertise(joint_state_ao);

		// Set up advertisements
		ros::AdvertiseOptions q_j_cmd_ao =
			ros::AdvertiseOptions::create<std_msgs::Float64MultiArray>(
					"/q_j_cmd",
					1,
					ros::SubscriberStatusCallback(),
					ros::SubscriberStatusCallback(),
					ros::VoidPtr(),
					&this->ros_publish_queue_
					);

		ros::AdvertiseOptions q_j_dot_cmd_ao =
			ros::AdvertiseOptions::create<std_msgs::Float64MultiArray>(
					"/q_j_dot_cmd",
					1,
					ros::SubscriberStatusCallback(),
					ros::SubscriberStatusCallback(),
					ros::VoidPtr(),
					&this->ros_publish_queue_
					);

		q_j_cmd_pub_= ros_node_.advertise(q_j_cmd_ao);
		q_j_dot_cmd_pub_= ros_node_.advertise(q_j_dot_cmd_ao);

		// Set up subscriptions
		ros::SubscribeOptions gen_coord_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model_name_ + "/gen_coord",
					1,
					boost::bind(&Controller::OnGenCoordMsg, this, _1),
					ros::VoidPtr(),
					&this->ros_process_queue_
					);

		ros::SubscribeOptions gen_vel_so =
			ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
					"/" + this->model_name_ + "/gen_vel",
					1,
					boost::bind(&Controller::OnGenVelMsg, this, _1),
					ros::VoidPtr(),
					&this->ros_process_queue_
					);

		gen_coord_sub_ = ros_node_.subscribe(gen_coord_so);
		gen_vel_sub_ = ros_node_.subscribe(gen_vel_so);

		ROS_INFO("Finished setting up ros topics");
	}

	void Controller::SpinRosThreads()
	{
		this->ros_process_queue_thread_ = std::thread(
				std::bind(&Controller::ProcessQueueThread, this)
				);
		this->ros_publish_queue_thread_ = std::thread(
				std::bind(&Controller::PublishQueueThread, this)
				);
	}

	void Controller::ProcessQueueThread()
	{
		static const double timeout = 0.01; // TODO: I should check this number
		while (ros_node_.ok())
		{
			ros_process_queue_.callAvailable(
					ros::WallDuration(timeout)
					);
		}
	}

	void Controller::PublishQueueThread()
	{
		while (ros_node_.ok())
		{
			if (!controller_ready_) continue;

			//CalcJointCmdStandup();
			CalcJointCmd();

			std_msgs::Float64MultiArray q_j_cmd_msg;
			tf::matrixEigenToMsg(q_j_cmd_, q_j_cmd_msg);
			q_j_cmd_pub_.publish(q_j_cmd_msg);

			std_msgs::Float64MultiArray q_j_dot_cmd_msg;
			tf::matrixEigenToMsg(q_j_dot_cmd_, q_j_dot_cmd_msg);
			q_j_dot_cmd_pub_.publish(q_j_dot_cmd_msg);

			PublishJointState();
			loop_rate_.sleep();	

			// TODO: move to its own node
		}
	}

	void Controller::PublishJointState()
	{
		sensor_msgs::JointState joint_state_msg;

		joint_state_msg.header.stamp = ros::Time::now();
		for(int i = 0; i < 12; i++)
		{
				joint_state_msg.name.push_back(joint_names_[i]);
				joint_state_msg.position.push_back(q_j_cmd_(i));
		}

		joint_state_pub_.publish(joint_state_msg);
	}


	void Controller::OnGenCoordMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		SetGenCoords(msg->data);
	}

	void Controller::OnGenVelMsg(
			const std_msgs::Float64MultiArrayConstPtr &msg
			)
	{
		SetGenVels(msg->data);
	}


	// ***************** //
	// SETTERS & GETTERS //
	// ***************** //

	void Controller::SetGenCoords(const std::vector<double> &gen_coords)
	{
		for (int i = 0; i < kNumGenCoords_; ++i)
			q_(i) = gen_coords[i];
	}

	void Controller::SetGenVels(const std::vector<double> &gen_vels)
	{
		for (int i = 0; i < 18; ++i)
			u_(i) = gen_vels[i];
	}

	// **************** //
	// HELPER FUNCTIONS //
	// **************** //

	void Controller::SetStartTime()
	{
		// Wait for /clock to get published
		while (ros::Time::now().toSec() == 0.0); 
		start_time_ = ros::Time::now();
		ROS_INFO("Start time: %f", start_time_.toSec());
	}

	double Controller::GetElapsedTimeSince(ros::Time t)
	{
		double elapsed_time = (ros::Time::now() - t).toSec();
		return elapsed_time; 
	}

	Eigen::MatrixXd Controller::CalcPseudoInverse(Eigen::MatrixXd A)
	{
		Eigen::MatrixXd eye =
			Eigen::MatrixXd::Identity(A.rows(), A.rows()); // same size as A*A^t
		double damping = 1;

		// Moore-Penrose right inverse: A^t (A A^t)
		Eigen:: MatrixXd pseudo_inverse =
			A.transpose() * (A * A.transpose() + damping * eye).inverse();
		return pseudo_inverse;
	}
}
