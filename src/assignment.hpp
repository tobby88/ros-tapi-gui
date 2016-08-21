#ifndef ASSIGNMENT_H
#define ASSIGNMENT_H

#include <string>

using namespace std;

class Assignment
{
private:
  string senderUUID;
  string senderFeatureUUID;
  string receiverUUID;
  string receiverFeatureUUID;
  double coefficient;

public:
  Assignment(string senderUUID, string senderFeatureUUID, string receiverUUID,
             string receiverFeatureUUID, double coefficient);
  ~Assignment();
};

#endif // ASSIGNMENT_H
