#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float32MultiArray.h>
#include <thread>
#include <functional>
#include <memory>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
namespace gazebo {
typedef struct CheckerPos {
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
} CheckerPos;
class CheckerborCtrl : public ModelPlugin {
 public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

  void OnUpdate();
  void QueueThread();
  // Pointer to the model
  // Register this plugin with the simulator
  void ROSCallback(const std_msgs::Float32MultiArrayConstPtr msg);

 private:
  physics::ModelPtr model;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
  ros::CallbackQueue rosQueue_;
  std::thread rosQueueThread_;
  /// \brief pointer to ros node
  std::unique_ptr<ros::NodeHandle> rosnode_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  CheckerPos checker_pos_;
};
}  // namespace gazebo
