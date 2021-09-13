#include "my_controller.h"
#include "ros/ros.h"
#include "std_msgs/Twist.h"

actual_msg = Twist(); // global velocity ss
point_msg = Twist();
ros::NodeHandle nh;
double error = 0 
double integral = 0 
double kp = 100
double kd = 1
double ki = 0.01

namespace my_controller_ns
{
	
	//Controller initialization
  	bool MyControllerClass::init(hardware_interface::VelocityJointInterface* hw) //, ros::NodeHandle &nh)
  	{
		ros::init(argc, argv, "/controller");

		ros::Subscriber sub1 = nh.subscribe("/cmd_vel", 1000, actualCallback);	// get actual velocity of bot
		ros::Subscriber sub2 = nh.subscribe("/point_vel", 1000, pointCallback); // get velocity from point algo
		//Retrieve the joint object to control
		std::string left_motor;
		std::string right_motor;
		if (!nh.getParam("left_motor", left_motor) || !nh.getParam("right_motor", right_motor))
		{
			ROS_ERROR("No joint_name specified");
			return false;
		}
    	joint_l = hw->getHandle(left_motor); 
    	joint_r = hw->getHandle(right_motor); 
    	return true;
	}

	void actualCallback(const geometry_msgs::Twist &msg)
	{
		if (msg.angular.z > 0)
		{	
			actual_msg = msg;
			ROS_INFO_STREAM("Subscriber velocities:"
							<< " linear=" << msg.linear.x << " angular=" << msg.angular.z);
		}
	}

	void pointCallback(const geometry_msgs::Twist &msg)
	{
		if (msg.angular.z > 0)
		{
			point_msg = msg;
			ROS_INFO_STREAM("Subscriber velocities:"
							<< " linear=" << msg.linear.x << " angular=" << msg.angular.z);
		}
	}

	//Controller startup
  	void MyControllerClass::starting(const ros::Time& time) 
	{
		//Get initial position to use in the control procedure
		// vl = joint_l.getVelocoty()
		// vr = joint_r.getVelocoty()
		vl = vr = 0;
	}

	//Controller running
   void MyControllerClass::update(const ros::Time& time, const ros::Duration& period)
   { 
	  	double l = 0.128;
	  	double r = 0.0215;

	  	// int vl = point_msg.angular.z * (r - l / 2);
	  	// int vr = point_msg.angular.z * (r + l / 2);

		vl = vr = point_msg.linear.y;
		if (point_msg.angular.z == 0)
		{
			double error = (point_msg.linear.y - actual_msg.linear.y) + (point_msg.angular.z - actual_msg.angular.z);
			double diff = error - prev_error; 
			integral += error;
			double PID = error * kp + diff * kd + integral * ki;
			prev_error = error;
			vr += PID;
			vl -= PID;
			// vl = vr = PID + actual_msg.linear.y;
		}

		if (vr > 255) 
			vr = 255;
		else if (vr < 0) 
			vr = 0;

		if (vl > 255)
			vl = 255;
		else if (vl < 0)
			vl = 0;

		joint_l.setCommand(vl);
		joint_r.setCommand(vr);

	  	//---Perform a sinusoidal motion for joint shoulder_pan_joint
	  	// double dpos = init_pos_ + 10 * sin(ros::Time::now().toSec());
	  	// double cpos = joint_.getPosition();
	  	// joint_.setCommand(-10 * (cpos - dpos)); //Apply command to the selected joint
	  	//---

	}

	//Controller exiting
	void MyControllerClass::stopping(const ros::Time& time) { }

}

//Register the plugin: PLUGINLIB_EXPORT_CLASS(my_namespace::MyPlugin, base_class_namespace::PluginBaseClass)
PLUGINLIB_EXPORT_CLASS(my_controller_ns::MyControllerClass, controller_interface::ControllerBase);