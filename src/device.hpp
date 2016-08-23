#ifndef DEVICE_H
#define DEVICE_H

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
  Device(uint8_t type, string name, string uuid, unsigned long lastSeq,
         Time lastSeen, unsigned long heartbeat);
  ~Device();

  // Public member functions
  void AddFeature(Feature feature);
  Feature* GetFeatureByUUID(string uuid);
  unsigned long GetHeartbeat();
  Time GetLastSeen();
  unsigned long GetLastSeq();
  string GetName() const;
  vector<Feature*> GetSortedFeatures();
  uint8_t GetType();
  string GetUUID();
  void Update(uint8_t type, string name, unsigned long lastSeq, Time lastSeen,
              unsigned long heartbeat);

private:
  // Private member variables
  map<string, Feature> features;
  unsigned long heartbeat;
  Time lastSeen;
  unsigned long lastSeq;
  string name;
  uint8_t type;
  string uuid;

  // Private member functions
  static bool compareFeatureNames(const Feature* first, const Feature* second);
};

#endif // DEVICE_H
