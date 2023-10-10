

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "controller_manager/controller_manager.h"

namespace controller_ns
{

  class CTestVelocityController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
  public:
    bool init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &n)
    {
      // get joint name from the parameter server
      std::string l_joint_a = "test_joint_A";
      if (!n.getParam("joint", l_joint_a))
      {
        ROS_ERROR("Could not find joint name");
        return false;
      }

      std::string l_joint_b = "test_joint_B";
      if (!n.getParam("joint", l_joint_b))
      {
        ROS_ERROR("Could not find joint name");
        return false;
      }

      // get the joint object to use in the realtime loop
      joint_a = hw->getHandle(l_joint_a); // throws on failure
      joint_b = hw->getHandle(l_joint_b); // throws on failure
      return true;
    }

    void update(const ros::Time &time, const ros::Duration &period)
    {
      double error = setpoint_ - joint_a.getVelocity();
      joint_a.setCommand(error * gain_);

      double error_b = setpoint_b - joint_b.getVelocity();
      joint_b.setCommand(error_b * gain_);
    }

    void starting(const ros::Time &time) {}
    void stopping(const ros::Time &time) {}

  private:
    hardware_interface::JointHandle joint_a;
    hardware_interface::JointHandle joint_b;
    double gain_ = 1.25;
    double setpoint_ = 3.00;

    double setpoint_b = 3.00;
  };
  // PLUGINLIB_DECLARE_CLASS(package_name, PositionController, controller_ns::PositionController, controller_interface::ControllerBase);

   PLUGINLIB_DECLARE_CLASS(test_ros_control, VelocityController, controller_ns::CTestVelocityController, controller_interface::ControllerBase);
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_controller");
  ros::NodeHandle nh;

  // Create the controller manager
  controller_manager::ControllerManager cm(&robot, nh);

  // Setup a separate thread that will be used to service ROS callbacks.
  ros::CallbackQueue ros_queue;
  ros::AsyncSpinner ros_spinner(1, &ros_queue);
  ros_spinner.start();

  // Setup a timer that will update the RobotHW and publish the joint states
  // at the desired frequency.
  ros::TimerOptions control_timer(
      ros::Duration(1.0 / control_frequency),
      boost::bind(controlLoop, boost::ref(cm)),
      &ros_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  // Run until shutdown.
  ros::waitForShutdown();
  return 0;
}