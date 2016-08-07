#include "api.hpp"
#include "apigui.hpp"
#include "ros/ros.h"
#include <QApplication>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "TobbyAPI");
  ros::NodeHandle nh;
  Api api(&nh);
  QApplication a(argc, argv);
  ApiGui w;
  w.show();

  return a.exec();
  api.Run();

  return 0;
}
