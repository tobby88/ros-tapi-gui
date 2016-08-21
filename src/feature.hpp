#ifndef FEATURE_H
#define FEATURE_H

#include "enums.hpp"
#include <string>

using namespace std;

class Feature
{
private:
  FeatureType type;
  string name;
  string description;
  string
      uuid; // Not necessary as this is saved in the hash-table (unordered_map)?
  int connections;

public:
  Feature(FeatureType type, string name, string description, string uuid);
  ~Feature();

  string getName() const;
  string getDescription();
  string getUUID();
  FeatureType getType();
  void incrementConnections();
  void decrementConnections();
  int getConnectionCount();
};

#endif // FEATURE_H
