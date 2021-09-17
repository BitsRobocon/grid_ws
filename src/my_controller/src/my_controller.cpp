#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <my_controller/template.h>

namespace my_controller_ns
{
	geometry_msgs::Twist actual_msg; // global velocity ss
	geometry_msgs::Twist point_msg;
	ros::NodeHandle nh;
	double l = 0.128;  // wheel separation
	double r = 0.0215; // wheel radius

	double vl = 0.0, vr = 0.0;
	double prev_error_linear = 0, prev_error_ang = 0;
	double integral_linear = 0, integral_ang = 0;
	double kp_l = 100, kd_l = 1.0, ki_l = 0.01;
	double kp_a = 100, kd_a = 1, ki_a = 0.01;
	long counter = 0;

	void actualCallback(const geometry_msgs::Twist &msg1)
	{
		actual_msg = msg1;
	}

	void pointCallback(const geometry_msgs::Twist &msg2)
	{
		point_msg = msg2;
	}
	//Controller initialization
  	bool MyControllerClass::init(hardware_interface::VelocityJointInterface* hw) //, ros::NodeHandle &nh)
  	{
		// ros::init(argc, argv, "/controller");

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

	//Controller startup
  	void MyControllerClass::starting(const ros::Time& time) 
	{
		//Get initial position to use in the control procedure
		// vl = joint_l.getVelocity()
		// vr = joint_r.getVelocity()
		vr = point_msg.linear.y + point_msg.angular.z * (l / 2.0);
		vl = point_msg.linear.y - point_msg.angular.z * (l / 2.0);
	}

	//Controller running
   void MyControllerClass::update(const ros::Time& time, const ros::Duration& period)
   { 
		vr = point_msg.linear.y + point_msg.angular.z * (l/2.0);
		vl = point_msg.linear.y - point_msg.angular.z * (l/2.0);

		double error_linear = (point_msg.linear.y - actual_msg.linear.y);
		double diff_linear = error_linear - prev_error_linear; 
		integral_linear += error_linear;
		double linear_pid = kp_l*error_linear + kd_l*diff_linear + ki_l*integral_linear;
			
		double error_ang = (point_msg.angular.z - actual_msg.angular.z);
		double diff_ang = error_ang - prev_error_ang;
		integral_ang += error_ang;
		double ang_pid = kp_l*error_ang + kd_l*diff_ang + ki_l*integral_ang;

		vr = vr + linear_pid + ang_pid;
		vl = vl + linear_pid - ang_pid;

		prev_error_linear = error_linear;
		prev_error_ang = error_ang;

		if (vr > 255)
			vr = 255;
		else if (vr < 0)
			vr = 0;

		if (vl > 255) 
			vl = 255;
		else if (vl < 0)
			vl = 0;

		joint_l.setCommand((int)vl);
		joint_r.setCommand((int)vr);

		counter++;	// integral windup
		if (counter % 50 == 0)
		{
			integral_ang = 0;
			integral_linear = 0;
		}

	}

	//Controller exiting
	void MyControllerClass::stopping(const ros::Time& time) { }

}

//Register the plugin: PLUGINLIB_EXPORT_CLASS(my_namespace::MyPlugin, base_class_namespace::PluginBaseClass)
PLUGINLIB_EXPORT_CLASS(my_controller_ns::MyControllerClass, controller_interface::ControllerBase);