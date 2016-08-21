#include "assignment.hpp"

Assignment::Assignment(string senderUUID, string senderFeatureUUID,
                       string receiverUUID, string receiverFeatureUUID,
                       double coefficient)
{
  this->senderUUID = senderUUID;
  this->senderFeatureUUID = senderFeatureUUID;
  this->receiverUUID = receiverUUID;
  this->receiverFeatureUUID = receiverFeatureUUID;
  this->coefficient = coefficient;
}

Assignment::~Assignment() {}
