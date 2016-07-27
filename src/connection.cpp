#include "connection.hpp"

Connection::Connection(string receiverUUID, string publisherUUID, string publisherFeatureUUID, string receiverFeatureUUID, double coefficient)
{
  this->receiverUUID = receiverUUID;
  this->publisherUUID = publisherUUID;
  this->publisherFeatureUUID = publisherFeatureUUID;
  this->receiverFeatureUUID = receiverFeatureUUID;
  this->coefficient = coefficient;
}

Connection::~Connection()
{ }

