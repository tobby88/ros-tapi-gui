#include "api.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TobbyAPI");
  ros::NodeHandle nodehandle("Tobby/API");
  Api api(&nodehandle);
  api.Run();

  return 0;
}
