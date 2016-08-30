#include "connection.hpp"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

Connection::Connection(string senderUUID, string senderFeatureUUID, string receiverUUID, string receiverFeatureUUID,
                       double coefficient)
  : senderUUID(senderUUID)
  , senderFeatureUUID(senderFeatureUUID)
  , receiverUUID(receiverUUID)
  , receiverFeatureUUID(receiverFeatureUUID)
  , coefficient(coefficient)
{
}

Connection::~Connection()
{
}

// Public member functions

double Connection::GetCoefficient()
{
  return coefficient;
}

string Connection::GetReceiverFeatureUUID()
{
  return receiverFeatureUUID;
}

string Connection::GetReceiverUUID()
{
  return receiverUUID;
}

string Connection::GetSenderFeatureUUID()
{
  return senderFeatureUUID;
}

string Connection::GetSenderUUID()
{
  return senderUUID;
}
}
