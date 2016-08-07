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
  ApiGui w(&api);
  w.show();
  api.Run();
  a.exec();

  return 0;
}
