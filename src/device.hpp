#ifndef DEVICE_H
#define DEVICE_H

#include "enums.hpp"
#include "feature.hpp"
#include "ros/ros.h"
#include <map>
#include <string>

using namespace ros;
using namespace std;

class Device
{
private:
  DeviceType type;
  string name;
  string
      uuid; // Not necessary as this is saved in the hash-table (unordered_map)
  unsigned long lastSeq;
  Time lastSeen;
  unsigned long heartbeat;
  map<string, Feature> features;

public:
  Device(DeviceType type, string name, string uuid, unsigned long lastSeq,
         Time lastSeen, unsigned long heartbeat);
  ~Device();
  void addFeature(Feature feature);
  map<string, Feature> getFeatureMap();
  DeviceType getType();
  string getName();
  string getUUID();
  unsigned long getLastSeq();
  Time getLastSeen();
  unsigned long getHeartbeat();
  void Update(DeviceType type, string name, unsigned long lastSeq,
              Time lastSeen, unsigned long heartbeat);
};

#endif // DEVICE_H
