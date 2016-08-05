#include "api.hpp"
#include "mainwindow.hpp"
#include "ros/ros.h"
#include <QApplication>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TobbyAPI");
  ros::NodeHandle nh;
  Api api(&nh);
  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  return a.exec();
  api.Run();

  return 0;
}
