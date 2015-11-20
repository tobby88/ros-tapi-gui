#ifndef DEVICE_H
#define DEVICE_H

#include "feature.hpp"
#include "ros/ros.h"
#include <string>
#include <unordered_map>

using namespace ros;
using namespace std;

class Device
{
private:
  string uuid; // Not necessary as this is saved in the hash-table (unordered_map)
  unsigned long last_seq;
  Time last_seen_timestamp;
  unsigned long heartbeat_interval;
  string name;
  unordered_map<string, Feature> features;
  unordered_map<string, Feature> decode_features(string features);

public:
  Device(unsigned long last_seq, Time last_seen_timestamp, unsigned long heartbeat_interval, string name, string features);
  ~Device();
};

#endif // API_H
