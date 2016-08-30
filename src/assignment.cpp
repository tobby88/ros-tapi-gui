#include "assignment.hpp"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

Assignment::Assignment(string senderUUID, string senderFeatureUUID, string receiverUUID, string receiverFeatureUUID,
                       double coefficient)
  : senderUUID(senderUUID)
  , senderFeatureUUID(senderFeatureUUID)
  , receiverUUID(receiverUUID)
  , receiverFeatureUUID(receiverFeatureUUID)
  , coefficient(coefficient)
{
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
}
