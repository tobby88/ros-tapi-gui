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
  void AddFeature(Feature feature);
  static bool CompareFeatureNames(const Feature* first, const Feature* second);
  Feature* GetFeatureByUUID(string uuid);
  unsigned long GetHeartbeat();
  Time GetLastSeen();
  unsigned long GetLastSeq();
  string GetName() const;
  vector<Feature*> GetSortedFeatures();
  DeviceType GetType();
  string GetUUID();
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
