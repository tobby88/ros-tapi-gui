/******************************************************************************
 *  Copyright (C) 2016 by Tobias Holst                                        *
 *                                                                            *
 *  This file is part of tapi_gui.                                            *
 *                                                                            *
 *  tapi_gui is free software: you can redistribute it and/or modify          *
 *  it under the terms of the GNU General Public License as published by      *
 *  the Free Software Foundation, either version 3 of the License, or         *
 *  (at your option) any later version.                                       *
 *                                                                            *
 *  tapi_gui is distributed in the hope that it will be useful,               *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
 *  GNU General Public License for more details.                              *
 *                                                                            *
 *  You should have received a copy of the GNU General Public License         *
 *  along with tapi_gui.  If not, see <http://www.gnu.org/licenses/>.         *
 *                                                                            *
 *  Diese Datei ist Teil von tapi_gui.                                        *
 *                                                                            *
 *  tapi_gui ist Freie Software: Sie können es unter den Bedingungen          *
 *  der GNU General Public License, wie von der Free Software Foundation,     *
 *  Version 3 der Lizenz oder (nach Ihrer Wahl) jeder späteren                *
 *  veröffentlichten Version, weiterverbreiten und/oder modifizieren.         *
 *                                                                            *
 *  tapi_gui wird in der Hoffnung, dass es nützlich sein wird, aber           *
 *  OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite        *
 *  Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK *
 *  Siehe die GNU General Public License für weitere Details.                 *
 *                                                                            *
 *  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem *
 *  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.*
 ******************************************************************************/

/*!
 * \file tapigui.hpp
 * \ingroup tapi_gui
 * \author Tobias Holst
 * \date 20 Nov 2016
 * \brief Declaration of the Tapi::TapiGui-class and definition of its member variables
 */

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
/*!
 * \brief Main widget in the main window
 *
 * This class is the main part of tapi_gui, it's the main widet in embedded in the main windows (Tapi::MainGui). It
 * communicates with the core to get its data and controls the core by sending it messages about connections
 * (new/delete) or about clearing inactive/all devices in the core.
 * \author Tobias Holst
 * \version 4.0.1
 */
class TapiGui : public QWidget
{
  Q_OBJECT

public:
  // Constructor/Destructor

  /*!
   * \brief Create a TapiGui-object (main widget of tapi_gui)
   * \param nh Pointer to a \c ros::NodeHandle created outside of this class
   * \param parent Pointer to the parent widget of this widget - has to be set to the Tapi::MainGui widget/window
   * \see Tapi::MainGui
   */
  TapiGui(ros::NodeHandle* nh, QWidget* parent = 0);

  //! Shutdown all publishers and subscribers, free memory allocated by this class (devices, spinner, guitimer, ui)
  ~TapiGui();

protected:
  // Protected member functions

  /*!
   * \brief Overrides the paintEvent of Qt to draw the connections between devices in the main widget
   *
   * It iterates through all connections, searches for both GuiDevice objects and then it tries to find both
   * Tapi::Feature belonging to the connection. Then it gets their positions of the feature boxes and draws line between
   * them.
   * If there is a current selection pending, it additionally draws a line from the start to the current mouse position.
   * \see \c Tapi::Feature in the \c tapi_lib package
   * \see Tapi::GuiDevice::FeatureBoxPosition
   */
  void paintEvent(QPaintEvent*) Q_DECL_OVERRIDE;

private:
  // Private member variables

  /*!
   * \brief Publisher to the core to clear all connections and devices, publishes on /Tapi/ClearAll
   * \see Tapi::TapiGui::clearAllButtonClicked
   */
  ros::Publisher clearAllPub;

  /*!
   * \brief Publisher to remove inactive devices and its connections in the core, publishes on /Tapi/ClearInactive
   * \see Tapi::TapiGui::clearInactiveButtonClicked
   */
  ros::Publisher clearInactivePub;

  /*!
   * \brief Saves the colors belonging to each topic type generated by Tapi::GuiDevice::stringToColor
   * \see Tapi::GuiDevice::stringToColor
   */
  std::map<std::string, QColor> colorKeys;

  /*!
   * \brief Stores all connections between the devices. Filled (and updated) with data from the core when updateData is
   * called
   * \see Tapi::TapiGui::updateData
   * \see Tapi::TapiGui::conListClient
   */
  std::map<std::string, Tapi::Connection> connections;

  /*!
   * \brief ServiceClient to call the core and get a list of all current connections
   * \see Tapi::TapiGui::connections
   * \see Tapi::TapiGui::updateData
   */
  ros::ServiceClient conListClient;

  /*!
   * \brief Publisher to tell the core to connect two devices, publishes on /Tapi/ConnectFeatures
   * \see Tapi::TapiGui::featureClicked
   */
  ros::Publisher conPub;

  /*!
   * \brief Publisher to tell the core to delete a connection from a Subscriber/ServiceClient, publishes on
   * /Tapi/DeleteConnection
   * \see Tapi::TapiGui::featureClicked
   */
  ros::Publisher delPub;

  /*!
   * \brief Stores all devices linked to the core. Filled (and updated) with data from the core when updateData is
   * called. Calls /Tapi/GetConnectionList
   * \see Tapi::TapiGui::updateData
   * \see Tapi::TapiGui::devListClient
   */
  std::map<std::string, Tapi::GuiDevice*> devices;

  /*!
   * \brief ServiceClient to call the core and get a list of all linked devices. Calls /Tapi/GetDeviceList
   * \see Tapi::TapiGui::devices
   * \see Tapi::TapiGui::updateData
   */
  ros::ServiceClient devListClient;

  /*!
   * \brief Timer to refresh the gui every 15 ms (60 Hz), stopped when a message box is shown
   * \see Tapi::TapiGui::checkForGuiUpdate
   */
  QTimer* guitimer;

  /*!
   * \brief Client to connect to Hello-service to add devices to the core. Used when a configuration file is loaded.
   * Calls /Tapi/HelloServ
   * \see Tapi::TapiGui::loadButtonClicked
   * \see Tapi::TapiGui::addDeviceToApi
   */
  ros::ServiceClient helloClient;

  /*!
   * \brief Time of last update from the core
   * \see Tapi::TapiGui::updateAvailable
   * \see Tapi::TapiGui::updateData
   */
  ros::Time lastUpdated;

  /*!
   * \brief Subscribes /Tapi/LastChanged to get information from the core about pending changes
   * \see Tapi::TapiGui::lastUpdated
   * \see Tapi::TapiGui::updateAvailable
   */
  ros::Subscriber lastUpdatedSub;

  /*!
   * \brief Left column where all Publisher/ServiceServer devices are shown
   * \see Tapi::TapiGui::publisherGuiDevices
   * \see Tapi::TapiGui::updateData
   */
  QVBoxLayout* layoutPublisher;

  /*!
   * \brief Right column where all Publisher/ServiceServer devices are shown
   * \see Tapi::TapiGui::subscriberGuiDevices
   * \see Tapi::TapiGui::updateData
   */
  QVBoxLayout* layoutSubscriber;

  /*!
   * \brief Position of the mouse during a pending connection to draw the line from the start to the current mouse
   * position
   * \see Tapi::TapiGui::paintEvent
   * \see Tapi::TapiGui::checkForGuiUpdate
   */
  QPoint mousePosition;

  //! NodeHandle-pointer necessary to create subscribers, publishers and services.
  ros::NodeHandle* nh;

  /*!
   * \brief Pointer to parent widget (should be a Tapi::MainGui object)
   * \see Tapi::MainGui
   */
  QWidget* parent;

  /*!
   * \brief Synchronize gui thread(s) with ros thread whether there are pending changes to be drawn
   *
   * No mutex necessary since this is a bool and it's only written in one thread anyways.
   * \see Tapi::TapiGui::checkForGuiUpdates
   * \see Tapi::TapiGui::updateData
   * \see Tapi::TapiGui::updateAvailable
   * \see Tapi::TapiGui::timer
   */
  bool pendingChanges;

  /*!
   * \brief Map of all Publisher/ServiceServer GuiDevices
   *
   * Map is filled and updated by the updateData function, which sorts the devices by name and sorts them by type
   * (tapi_lib::Device::Type_Publisher for Publisher and ServiceServer or tapi_lib::Device::Type_Subscriber for
   * Subscriber and ServiceClients)
   * \see \c Device.msg of the \c tapi_lib package
   * \see Tapi::GuiDevice
   */
  std::map<std::string, Tapi::GuiDevice*> publisherGuiDevices;

  /*!
   * \brief Pointer to the selected feature for the current pending connection
   * \see Tapi::TapiGui::featureClicked
   */
  Tapi::Feature* selectedFeature;

  /*!
   * \brief Pointer to the selected GuiDevice for the current pending connection
   * \see Tapi::TapiGui::featureClicked
   */
  Tapi::GuiDevice* selectedGuiDevice;

  //! Asynchronous spinner (-thread) to let ros do its callbacks without interfering with the gui-thread(s)
  ros::AsyncSpinner* spinner;

  /*!
   * \brief Map of all Subscriber/ServiceClient GuiDevices
   *
   * Map is filled and updated by the updateData function, which sorts the devices by name and sorts them by type
   * (tapi_lib::Device::Type_Publisher for Publisher and ServiceServer or tapi_lib::Device::Type_Subscriber for
   * Subscriber and ServiceClients)
   * \see \c Device.msg of the \c tapi_lib package
   * \see Tapi::GuiDevice
   */
  std::map<std::string, Tapi::GuiDevice*> subscriberGuiDevices;

  /*!
   * \brief Timer interval of the guitimer to start/stop it later with the same time
   * \see Tapi::TapiGui::guitimer
   */
  int timerInterval;

  //! Pointer to the ui object, generated from the ui-designer and created in the constructor
  Ui::TapiGui* ui;

  /*!
   * \brief This timer is used to regularly check the core for updates, even if there was no call of updateAvailable.
   *
   * Calls function "timer" on a timer event. Checks every CHECK_INTERVAL milliseconds.
   * \see Tapi::TapiGui::timer
   */
  ros::Timer updateTimer;

  // Private member functions

  /*!
   * \brief Add a device to the core, even if the device is not connected. Necessary to be able to load a config file
   * which may include inactive devices
   * \param type Type of the eevice (tapi_lib::Device::Type_Publisher for Publisher and ServiceServer or
   * tapi_lib::Device::Type_Subscriber for Subscriber and ServiceClients)
   * \param name Name of the device
   * \param uuid Unique ID of the device
   * \param features Map of the device's features
   * \see \c Device.msg in the \c tapi_lib package
   * \see \c Tapi::Feature in the \c tapi_lib package
   */
  void addDeviceToTapi(uint8_t type, std::string name, std::string uuid, std::map<std::string, Tapi::Feature> features);

  /*!
   * \brief To sort GuiDevices there has to be a compare function, comparing their names.
   * \param first Pointer to the first GuiDevice to compare
   * \param second Pointer to the second GuiDevice to compare
   * \return \c true if the first GuiDevice's name has to be above the second's GuiDevice name in an alphabetically
   * sort, \c false when it's vice versa
   * \see Tapi::GuiDevice
   */
  static bool compareDeviceNames(const Tapi::GuiDevice* first, const Tapi::GuiDevice* second);

  /*!
   * \brief Send the command to the core that it shall connect two features
   *
   * When the user has selected two features of the same type on compatible device types (Subscriber to Publisher or
   * ServiceClient to ServiceServer), this function is called and sends a message to the core with the two uuids which
   * shahll be connected
   * \param feature1UUID Unique ID of one of the two features which schall be connected
   * \param feature2UUID Unique ID of the other of the two features which schall be connected
   * \param coefficient Optional, if set and it's a compatible tpye (simple number type of std_msgs like \c UInt32 or \c
   * Float64) there can be a coefficient and all values have to be multiplied with this value. But has to be implemented
   * on the client side!
   * \see Tapi::TapiGui::featureClicked
   * \see Tapi::TapiGui::conPub
   */
  void connectFeatures(std::string feature1UUID, std::string feature2UUID, double coefficient = 1.0);

  /*!
   * \brief Delete the connection of two devices by sending this as a command to the core
   * \param subscriberFeatureUUID Unique ID of the feature of the Subscriber/ServiceClient which shall be disconnected
   * \see Tapi::TapiGui::featureClicked
   * \see Tapi::TapiGui::delPub
   */
  void deleteConnection(std::string subscriberFeatureUUID);

  /*!
   * \brief Get a \c vector of pointers to all elements in the Tapi::TapiGui::devices map, alphabetically sorted.
   * \return \c vector of pointers to all elements in the Tapi::TapiGui::devices map, alphabetically sorted.
   * \see Tapi::GuiDevice
   * \see Tapi::compareDeviceNames
   */
  std::vector<Tapi::GuiDevice*> getDevicesSorted();

  /*!
   * \brief "timer" is called by a ros callback when the timer event occurs, it then checks if the gui should do an
   * update of its data from the core, because the core didn't send new data for a too long time
   * \param e Unused
   * \see Tapi::TapiGui::updateTimer
   */
  void timer(const ros::TimerEvent& e);

  /*!
   * \brief Called by a ros callback when a new timestamp on /Tapi/LastChanged is got
   * \param time The message waiting in the ros message queue where the timestamp of the last change is stored
   * \see Tapi::TapiGui::lastUpdatedSub
   */
  void updateAvailable(const std_msgs::Time::ConstPtr& time);

  /*!
   * \brief Actually update all the data in the gui
   *
   * It calls the services of the core to get all data and then compares it with the data already stored in the gui. If
   * there is new data, the data is updated/added.
   * \see Tapi::TapiGui::checkForGuiUpdates
   */
  void updateData();

private slots:
  // Slot functions

  /*!
   * \brief Slot function connected to guitimer. Checks for pending Changes and redraws the gui.
   * First it checks, whether there are changes the core already mentioned or when it didn't "say" anything for a long
   * time. If yes it calls updateData and the data is updated/added to the gui members and the gui is redrawn.
   * \see Tapi::TapiGui::guitimer
   * \see Tapi::TapiGui::updateData
   * \see Tapi::TapiGui::pendingChanges
   */
  void checkForGuiUpdate();

  /*!
   * \brief Slot function connected to the click-event of the clear all button
   *
   * It then sends the command to the core that it should delete all of its content and demarkates a pending connection
   * in the gui, since there is no device anymore then.
   * \see Tapi::TapiGui::clearAllPub
   */
  void clearAllButtonClicked();

  /*!
   * \brief Slot function connected to the click event of the clear inactive button
   *
   * Just send a message to the core that is shall delete all inactive devices and its connections
   * \see Tapi::TapiGui::clearInactivePub
   */
  void clearInactiveButtonClicked();

  /*!
   * \brief Slot function connected to the "featureClicked" event of all GuiDevice objects, emitted by their
   * mousReleaseEvent
   *
   * When a feature is clicked and this event is called it checks if its the second click to this feature. If yes, the
   * feature is demarcated.
   * Then it checks, if it's the second selection an whether both devices can be connected or not. If yes, the
   * "connectFeatures" function is called. If not, the old selection will be overwritten by the last selected feature.
   * If a feature of a Subscriber or ServiceClient is clicked, which is alreads connected, it asks the user, if the
   * connection shall be deleted (if there is currently no selection) or replaced (if there is currently a selection and
   * the devices are compatible)
   * \param guidevice A pointer to the guidevice who emitted the signal and "called" this slot
   * \param feature A pointer to the feature the user clicked on
   * \see Tapi::GuiDevice::featureClicked
   */
  void featureClicked(Tapi::GuiDevice* guidevice, Tapi::Feature* feature);

  /*!
   * \brief Slot function connected to the click event of the load button
   *
   * It opens a file picker to choose a file, deletes every device and connection in the gui and the core and tries to
   * read the file. Its data is then sent to the core
   * \see Tapi::TapiGui::addDeviceToTapi
   * \see Tapi::TapiGui::clearAllButtonClicked
   */
  void loadButtonClicked();

  /*!
   * \brief Slot function connected to the click event of the save button
   *
   * It opens a file picker to choose an output file. Then all data of the gui (which should be the same as in the core)
   * is saved into this file
   */
  void saveButtonClicked();
};
}

#endif  // APIGUI_HPP
