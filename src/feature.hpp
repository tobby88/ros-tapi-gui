#ifndef FEATURE_H
#define FEATURE_H

#include "enums.hpp"
#include <string>

using namespace std;

class Feature
{
public:
  // Constructor/Destructor
  Feature(FeatureType type, string name, string description, string uuid);
  ~Feature();

  // Public member functions
  void decrementConnections();
  int getConnectionCount();
  string getDescription();
  string getName() const;
  FeatureType getType();
  string getUUID();
  void incrementConnections();

private:
  // Private member variables
  FeatureType type;
  string name;
  string description;
  string uuid;
  int connections;
};

#endif // FEATURE_H
