#include "ros/ros.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include <mav_msgs/default_topics.h>
#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>

#include <sstream>
#include <cstdlib>
#include <fstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_broadcaster");
  ros::NodeHandle n("~");
  ros::Rate loop_rate(10);
  std::string rotors_ns;
  if (!n.getParam("rotors_ns", rotors_ns)) {
    std::cout << "Couldn't read rotors namespace. Please provide as a ros param (rotors_ns)" << std::endl;
    exit(1);
  }
  ros::NodeHandle nh(rotors_ns);
  ros::Publisher ctrl_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory> (
    mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  
  trajectory_msgs::MultiDOFJointTrajectory ctrl_msg;
  ctrl_msg.header.stamp = ros::Time::now();
  Eigen::Vector3d desired_position;
  double desired_yaw;

  std::ifstream ifs;
  std::string filename;
  if (!n.getParam("file_name", filename)) {
    std::cout << "Couldn't read file name. Please provide as a ros param (file_name)" << std::endl;
    exit(1);
  }
  ifs.open(filename, std::ios::in);
  std::string line, s_line;

  while (ros::ok())
  {  
    std::getline(ifs, line);
    std::stringstream ss1(line);
    std::getline(ss1, s_line, ';');
    std::stringstream ss(s_line);
    std::string entry;
    std::vector<double> v_entry;
    while (std::getline(ss, entry, ',')) {
      v_entry.push_back(std::stod(entry));
    }  

    desired_position.x() = v_entry[0];
    desired_position.y() = v_entry[1];
    desired_position.z() = v_entry[2];
    desired_yaw = v_entry[5];
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
        desired_yaw, &ctrl_msg);

    ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f, %f].",
             n.getNamespace().c_str(),
             desired_position.x(),
             desired_position.y(),
             desired_position.z(),
             desired_yaw);

    ctrl_pub.publish(ctrl_msg);
  
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
