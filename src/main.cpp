#include "api.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TobbyAPI");
  ros::NodeHandle nh;
  Api api(&nh);
  api.Run();

  return 0;
}
