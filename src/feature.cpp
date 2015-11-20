#include "feature.hpp"

Feature::Feature(string feature_name, unsigned long count, Feature_Type type, string msg_type)
{
  this->feature_name = feature_name;
  this->count = count;
  this->type = type;
  this->msg_type = msg_type;
}

Feature::~Feature()
{ }
