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
  string uuid; // Not necessary as this is saved in the hash-table (unordered_map)?
  //string msg_type; // TODO: Right choice to save message type? How to subscribe later on?

public:
  Feature(FeatureType type, string name, string description, string uuid);
  ~Feature();

  string getName();
  string getDescription();
  string getUUID();
  FeatureType getType();
};

#endif // FEATURE_H
