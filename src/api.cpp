#include "api.hpp"
#include "ros/ros.h"

#define STANDARD_HEARTBEAT_INTERVAL 10000L

using namespace ros;

Api::Api(NodeHandle *nodehandle)
{
  nh = nodehandle;
  ServiceServer helloServ = nh->advertiseService("Hello", &Api::hello, this);
  ROS_INFO("Started Hello-Service, ready for API-connections.");
}

Api::~Api()
{
  ROS_INFO("Hello-Service has been stopped.");
}

bool Api::hello(tobby::hello::Request &helloReq, tobby::hello::Response &helloResp)
{
  string uuid = helloReq.uuid;
  if(devices.empty())
  {
    // New device:
    unsigned long last_seq = helloReq.header.seq;
    Time last_seen_timestamp = helloReq.header.stamp;
    unsigned long heartbeat_interval = STANDARD_HEARTBEAT_INTERVAL;
    string name = helloReq.product;

    // WORK IN PROGRESS
    /*helloReq.features.empty();
    unordered_map<string, Feature> features;
    unordered_map<string, Feature> decode_features(string features);*/
  }
  return true;
}

void Api::Run()
{
  spin();
}
