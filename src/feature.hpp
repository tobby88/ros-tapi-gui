#ifndef FEATURE_H
#define FEATURE_H

#include "enums.hpp"
#include <string>

using namespace std;

class Feature
{
private:
  Feature_Type type;
  string feature_name; // Not necessary as this is saved in the hash-table (unordered_map)?
  unsigned long id;
  //string msg_type; // TODO: Right choice to save message type? How to subscribe later on?

public:
  Feature(Feature_Type type, string feature_name, unsigned long id);
  ~Feature();

  unsigned long getID();
};

#endif // FEATURE_H
