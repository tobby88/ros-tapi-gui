#ifndef APIGUI_HPP
#define APIGUI_HPP

#include <QPoint>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <map>
#include <vector>
#include "guidevice.hpp"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/service_client.h"
#include "ros/spinner.h"
#include "ros/subscriber.h"
#include "ros/timer.h"
#include "std_msgs/Time.h"
#include "tapi_lib/connection.hpp"
#include "tapi_lib/device.hpp"

#define CHECK_INTERVAL 1000L

namespace Ui
{
class ApiGui;
}

namespace Tapi
{
class ApiGui : public QWidget
{
  Q_OBJECT

public:
  // Constructor/Destructor
  ApiGui(ros::NodeHandle* nh, QWidget* parent = 0);
  ~ApiGui();

protected:
  // Protected member functions
  void paintEvent(QPaintEvent*) Q_DECL_OVERRIDE;

private:
  // Private member variables
  ros::Publisher clearPub;
  std::map<std::string, QColor> colorKeys;
  std::map<std::string, Tapi::Connection> connections;
  ros::ServiceClient conListClient;
  ros::Publisher conPub;
  ros::Publisher delPub;
  std::map<std::string, Tapi::GuiDevice*> devices;
  ros::ServiceClient devListClient;
  QTimer* guitimer;
  ros::ServiceClient helloClient;
  ros::Time lastUpdated;
  ros::Subscriber lastUpdatedSub;
  QVBoxLayout* layoutPublisher;
  QVBoxLayout* layoutSubscriber;
  QPoint mousePosition;
  ros::NodeHandle* nh;
  QWidget* parent;
  bool pendingChanges;
  std::map<std::string, Tapi::GuiDevice*> publisherGuiDevices;
  Tapi::Feature* selectedFeature;
  Tapi::GuiDevice* selectedGuiDevice;
  ros::AsyncSpinner* spinner;
  std::map<std::string, Tapi::GuiDevice*> subscriberGuiDevices;
  int timerInterval;
  Ui::ApiGui* ui;
  ros::Timer updateTimer;

  // Private member functions
  void addDeviceToApi(uint8_t type, std::string name, std::string uuid, std::map<std::string, Tapi::Feature> features);
  static bool compareDeviceNames(const Tapi::GuiDevice* first, const Tapi::GuiDevice* second);
  void connectFeatures(std::string feature1UUID, std::string feature2UUID, double coefficient);
  void deleteConnection(std::string subscriberFeatureUUID);
  Tapi::GuiDevice* getDeviceByFeatureUUID(std::string uuid);
  std::vector<Tapi::GuiDevice*> getDevicesSorted();
  void timer(const ros::TimerEvent& e);
  void updateAvailable(const std_msgs::Time::ConstPtr& time);
  void updateData();

private slots:
  // Slot functions
  void checkForGuiUpdate();
  void clear();
  void featureClicked(Tapi::GuiDevice* guidevice, Tapi::Feature* feature);
  void loadButtonClicked();
  void saveButtonClicked();
};
}

#endif  // APIGUI_HPP
