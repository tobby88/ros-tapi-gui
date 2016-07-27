#ifndef CONNECTION_H
#define CONNECTION_H

#include <string>

using namespace std;

class Connection
{
private:
  string receiverUUID;
  string publisherUUID;
  string publisherFeatureUUID;
  string receiverFeatureUUID;
  double coefficient;

public:
  Connection(string receiverUUID, string publisherUUID, string publisherFeatureUUID, string receiverFeatureUUID, double coefficient);
  ~Connection();

};

#endif // CONNECTION_H
