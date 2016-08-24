#include "assignment.hpp"

using namespace std;

// Constructor/Destructor

Assignment::Assignment(string senderUUID, string senderFeatureUUID, string receiverUUID, string receiverFeatureUUID,
                       double coefficient)
{
  this->senderUUID = senderUUID;
  this->senderFeatureUUID = senderFeatureUUID;
  this->receiverUUID = receiverUUID;
  this->receiverFeatureUUID = receiverFeatureUUID;
  this->coefficient = coefficient;
}

Assignment::~Assignment()
{
}

// Public member functions

double Assignment::GetCoefficient()
{
  return coefficient;
}

string Assignment::GetReceiverFeatureUUID()
{
  return receiverFeatureUUID;
}

string Assignment::GetReceiverUUID()
{
  return receiverUUID;
}

string Assignment::GetSenderFeatureUUID()
{
  return senderFeatureUUID;
}

string Assignment::GetSenderUUID()
{
  return senderUUID;
}
