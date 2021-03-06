#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>


namespace my_controller_ns 
{	class MyControllerClass: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
	{
	public:
		bool init(hardware_interface::VelocityJointInterface *hw); //, ros::NodeHandle &n
		void update(const ros::Time& time, const ros::Duration& period);
		void starting(const ros::Time& time);
		void stopping(const ros::Time& time);

	protected:
		hardware_interface::JointHandle joint_l;
		hardware_interface::JointHandle joint_r;
	};
}