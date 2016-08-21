#ifndef ASSIGNMENT_H
#define ASSIGNMENT_H

#include <string>

using namespace std;

class Assignment
{
public:
  // Public member functions
  Assignment(string senderUUID, string senderFeatureUUID, string receiverUUID,
             string receiverFeatureUUID, double coefficient);
  ~Assignment();
  double getCoefficient();
  string getReceiverFeatureUUID();
  string getReceiverUUID();
  string getSenderFeatureUUID();
  string getSenderUUID();

private:
  // Private member variables
  double coefficient;
  string receiverFeatureUUID;
  string receiverUUID;
  string senderFeatureUUID;
  string senderUUID;
};

#endif // ASSIGNMENT_H
