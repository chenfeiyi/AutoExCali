#include "checkerbor_ctrl.h"
namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(CheckerborCtrl);
void CheckerborCtrl::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  this->model = _parent;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  checker_pos_.x = 2;
  checker_pos_.y = 0;
  checker_pos_.z = 1;
  checker_pos_.roll = 0;
  checker_pos_.pitch = 1.57;
  checker_pos_.yaw = 0;

  if (!ros::isInitialized()) {
    int argc;
    char **argv = NULL;

    ros::init(argc, argv, "checkerbor_ctrl",
              ros::init_options::NoSigintHandler);
  }
  this->rosnode_.reset(new ros::NodeHandle("checkerbor_ctrl"));
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
          "/checkerctrl", 1,
          boost::bind(&CheckerborCtrl::ROSCallback, this, _1), ros::VoidPtr(),
          &this->rosQueue_);
  this->sub_ = this->rosnode_->subscribe(so);
  this->rosQueueThread_ =
      std::thread(std::bind(&CheckerborCtrl::QueueThread, this));
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CheckerborCtrl::OnUpdate, this));
}

// Called by the world update start event
void CheckerborCtrl::OnUpdate() {
  // Apply a small linear velocity to the model.
  this->model->SetWorldPose(gazebo::math::Pose(
      checker_pos_.x, checker_pos_.y, checker_pos_.z, checker_pos_.roll,
      checker_pos_.pitch, checker_pos_.yaw));
}
void CheckerborCtrl::ROSCallback(
    const std_msgs::Float32MultiArrayConstPtr msg) {
  checker_pos_.x = msg->data.at(0);
  checker_pos_.y = msg->data.at(1);
  checker_pos_.z = msg->data.at(2);
  checker_pos_.roll = msg->data.at(3);
  checker_pos_.pitch = msg->data.at(4);
  checker_pos_.yaw = msg->data.at(5);
}

void CheckerborCtrl::QueueThread() {
  while (this->rosnode_->ok()) {
    this->rosQueue_.callAvailable(ros::WallDuration(0.01));
  }
}

}  // namespace gazebo
