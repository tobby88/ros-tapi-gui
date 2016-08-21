#include "assignment.hpp"

using namespace std;

// Public member functions

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

double Assignment::getCoefficient() { return coefficient; }

string Assignment::getReceiverFeatureUUID() { return receiverFeatureUUID; }

string Assignment::getReceiverUUID() { return receiverUUID; }

string Assignment::getSenderFeatureUUID() { return senderFeatureUUID; }

string Assignment::getSenderUUID() { return senderUUID; }
