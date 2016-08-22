#ifndef DEVICE_H
#define DEVICE_H

#include "enums.hpp"
#include "feature.hpp"
#include "ros/ros.h"
#include <map>
#include <string>
#include <vector>

using namespace ros;
using namespace std;

class Device
{
public:
  // Constructor/Destructor
  Device(DeviceType type, string name, string uuid, unsigned long lastSeq,
         Time lastSeen, unsigned long heartbeat);
  ~Device();

  // Public member functions
  void addFeature(Feature feature);
  static bool compareFeatureNames(const Feature* first, const Feature* second);
  Feature* getFeatureByUUID(string uuid);
  map<string, Feature> getFeatureMap();
  unsigned long getHeartbeat();
  Time getLastSeen();
  unsigned long getLastSeq();
  string getName() const;
  vector<Feature*> GetSortedFeatures();
  DeviceType getType();
  string getUUID();
  void Update(DeviceType type, string name, unsigned long lastSeq,
              Time lastSeen, unsigned long heartbeat);

private:
  // Private member variables
  map<string, Feature> features;
  unsigned long heartbeat;
  Time lastSeen;
  unsigned long lastSeq;
  string name;
  DeviceType type;
  string uuid;
};

#endif // DEVICE_H
