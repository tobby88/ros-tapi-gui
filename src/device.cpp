#include "device.hpp"
#include "ros/ros.h"

using namespace ros;

Device::Device(unsigned long last_seq, Time last_seen_timestamp, unsigned long heartbeat_interval, string name, string features)
{
  this->last_seq = last_seq;
  this->last_seen_timestamp = last_seen_timestamp;
  this->heartbeat_interval = heartbeat_interval;
  this->name = name;
  this->features = decode_features(features);
}

Device::~Device()
{ }

unordered_map<string, Feature> Device::decode_features(string features)
{
  // TODO: Decode the feature-String to features
  unordered_map<string, Feature> temp;
  return temp;
}
