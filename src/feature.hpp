#ifndef FEATURE_H
#define FEATURE_H

#include "enums.hpp"
#include <string>

using namespace std;

class Feature
{
private:
  string feature_name; // Not necessary as this is saved in the hash-table (unordered_map)?
  unsigned long count;
  Feature_Type type;
  string msg_type; // TODO: Right choice to save message type? How to subscribe later on?

public:
  Feature(string feature_name, unsigned long count, Feature_Type type, string msg_type);
  ~Feature();
};

#endif // FEATURE_H
