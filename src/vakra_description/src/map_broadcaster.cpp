#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "map_broadcaster");
    ros::NodeHandle nh;

	int32_t publish_rate_ = 100;
	tf::TransformBroadcaster tf_br_;
	tf::StampedTransform tf_map_to_odom_;

	// set up parent and child frames
	tf_map_to_odom_.frame_id_ = std::string("map");
	tf_map_to_odom_.child_frame_id_ = std::string("odom");

	// set up publish rate
	ros::Rate loop_rate(publish_rate_);

	// main loop
	while (ros::ok())
	{
	// time stamp
	tf_map_to_odom_.stamp_ = ros::Time::now();

	// specify actual transformation vectors from odometry
	// NOTE: zeros have to be substituted with actual variable data
	tf_map_to_odom_.setOrigin(tf::Vector3(1.0, 1.0, 0.0));
	tf_map_to_odom_.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

	// broadcast transform
	tf_br_.sendTransform(tf_map_to_odom_);

	ros::spinOnce();
	loop_rate.sleep();
	}
}

// int main(int argc, char** argv){
//     ros::init(argc, argv, "map_broadcaster");
//     ros::NodeHandle nh;
//     ros::Rate r(100);

//     tf::TransformBroadcaster br;

//     while(nh.ok()){
//         br.sendTransform(
//             tf::StampedTransform(
//                 tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.0,0.0,0.0)),
//                 ros::Time::now(),"map", "odom"));

//         r.sleep();
//     }
// }
