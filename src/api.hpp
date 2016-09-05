#ifndef API_H
#define API_H

// Intervals in ms
#define CHECK_INTERVAL 1000L

#include <map>
#include <string>
#include <vector>
#include "connection.hpp"
#include "device.hpp"
#include "ros/ros.h"
#include "std_msgs/Time.h"

namespace Tapi
{
class Api
{
public:
  // Constructor/Destructor
  explicit Api(ros::NodeHandle* nh);

  // Public member variables
  ros::NodeHandle* nh;
};
}

#endif  // API_H
