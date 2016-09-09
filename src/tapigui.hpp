/******************************************************************************
*  This file is part of tapi_gui.                                             *
*                                                                             *
*  tapi_gui is free software: you can redistribute it and/or modify           *
*  it under the terms of the GNU General Public License as published by       *
*  the Free Software Foundation, either version 3 of the License, or          *
*  (at your option) any later version.                                        *
*                                                                             *
*  tapi_gui is distributed in the hope that it will be useful,                *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of             *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
*  GNU General Public License for more details.                               *
*                                                                             *
*  You should have received a copy of the GNU General Public License          *
*  along with tapi_gui.  If not, see <http://www.gnu.org/licenses/>.          *
*                                                                             *
*  Diese Datei ist Teil von tapi_gui.                                         *
*                                                                             *
*  tapi_gui ist Freie Software: Sie können es unter den Bedingungen           *
*  der GNU General Public License, wie von der Free Software Foundation,      *
*  Version 3 der Lizenz oder (nach Ihrer Wahl) jeder späteren                 *
*  veröffentlichten Version, weiterverbreiten und/oder modifizieren.          *
*                                                                             *
*  tapi_gui wird in der Hoffnung, dass es nützlich sein wird, aber            *
*  OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite         *
*  Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK  *
*  Siehe die GNU General Public License für weitere Details.                  *
*                                                                             *
*  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem  *
*  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>. *
*******************************************************************************/

#ifndef TAPIGUI_HPP
#define TAPIGUI_HPP

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
class TapiGui;
}

namespace Tapi
{
class TapiGui : public QWidget
{
  Q_OBJECT

public:
  // Constructor/Destructor
  TapiGui(ros::NodeHandle* nh, QWidget* parent = 0);
  ~TapiGui();

protected:
  // Protected member functions
  void paintEvent(QPaintEvent*) Q_DECL_OVERRIDE;

private:
  // Private member variables
  ros::Publisher clearAllPub;
  ros::Publisher clearInactivePub;
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
  Ui::TapiGui* ui;
  ros::Timer updateTimer;

  // Private member functions
  void addDeviceToTapi(uint8_t type, std::string name, std::string uuid, std::map<std::string, Tapi::Feature> features);
  static bool compareDeviceNames(const Tapi::GuiDevice* first, const Tapi::GuiDevice* second);
  void connectFeatures(std::string feature1UUID, std::string feature2UUID, double coefficient);
  void deleteConnection(std::string subscriberFeatureUUID);
  std::vector<Tapi::GuiDevice*> getDevicesSorted();
  void timer(const ros::TimerEvent& e);
  void updateAvailable(const std_msgs::Time::ConstPtr& time);
  void updateData();

private slots:
  // Slot functions
  void checkForGuiUpdate();
  void clearAllButtonClicked();
  void clearInactiveButtonClicked();
  void featureClicked(Tapi::GuiDevice* guidevice, Tapi::Feature* feature);
  void loadButtonClicked();
  void saveButtonClicked();
};
}

#endif  // APIGUI_HPP
