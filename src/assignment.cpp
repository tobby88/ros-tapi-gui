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

string Assignment::getSenderUUID() { return senderUUID; }

string Assignment::getSenderFeatureUUID() { return senderFeatureUUID; }

string Assignment::getReceiverUUID() { return receiverUUID; }

string Assignment::getReceiverFeatureUUID() { return receiverFeatureUUID; }

double Assignment::getCoefficient() { return coefficient; }
