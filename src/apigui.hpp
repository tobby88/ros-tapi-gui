#ifndef APIGUI_HPP
#define APIGUI_HPP

#include <QPoint>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <map>
#include <vector>
#include "api.hpp"
#include "device.hpp"
#include "guidevice.hpp"
#include "tapi_msgs/Connection.h"
#include "tapi_msgs/Device.h"

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
  Tapi::Api* api;
  ros::Publisher clearPub;
  std::map<std::string, QColor> colorKeys;
  std::map<std::string, Tapi::Connection> connections;
  ros::ServiceClient conListClient;
  ros::Publisher conPub;
  ros::Publisher delPub;
  std::map<std::string, Tapi::Device> devices;
  ros::ServiceClient devListClient;
  QTimer* guitimer;
  ros::ServiceClient helloClient;
  ros::Time lastUpdated;
  ros::Subscriber lastUpdatedSub;
  QVBoxLayout* layoutReceiver;
  QVBoxLayout* layoutSender;
  QPoint mousePosition;
  ros::NodeHandle* nh;
  bool pendingChanges;
  std::vector<Tapi::GuiDevice*> receiverGuiDevices;
  Tapi::Feature* selectedFeature;
  Tapi::GuiDevice* selectedGuiDevice;
  std::vector<Tapi::GuiDevice*> senderGuiDevices;
  ros::AsyncSpinner* spinner;
  int timerInterval;
  Ui::ApiGui* ui;
  ros::Timer updateTimer;

  // Private member functions
  void addDevice(Tapi::Device* device);
  void addDevice(uint8_t type, std::string name, std::string uuid, std::map<std::string, Tapi::Feature> features);
  void changed();
  bool checkPending();
  void clear();
  static bool compareDeviceNames(const Tapi::Device* first, const Tapi::Device* second);
  bool connectFeatures(std::string feature1UUID, std::string feature2UUID, double coefficient);
  bool deleteConnection(std::string receiverFeatureUUID);
  void done();
  std::vector<Tapi::Connection*> getConnections();
  Tapi::Device* getDeviceByFeatureUUID(std::string uuid);
  std::vector<Tapi::Device*> getDevicesSorted();
  void run();
  void timer(const ros::TimerEvent& e);
  void updateAvailable(const std_msgs::Time::ConstPtr& time);
  void updateData();

private slots:
  // Slot functions
  void checkApiForUpdate();
  void clearButtonClicked();
  void featureClicked(Tapi::GuiDevice* guidevice, Tapi::Feature* feature);
  void loadButtonClicked();
  void saveButtonClicked();
};
}

#endif  // APIGUI_HPP
