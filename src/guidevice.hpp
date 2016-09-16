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
 * \file guidevice.hpp
 * \ingroup tapi_gui
 * \author Tobias Holst
 * \date 10 Aug 2016
 * \brief Declaration of the Tapi::GuiDevice-class and definition of its member variables
 */

#ifndef GUIDEVICE_H
#define GUIDEVICE_H

#include <QColor>
#include <QMouseEvent>
#include <QWidget>
#include <vector>
#include "tapi_lib/device.hpp"
#include "tapi_lib/feature.hpp"

namespace Tapi
{
/*!
 * \brief GuiDevice inherits from Device and stores additional data and functions to draw, show and control the devices
 * in the gui
 * \author Tobias Holst
 * \version 4.0.1
 */
class GuiDevice : public QWidget, public Device
{
  Q_OBJECT

public:
  // Constructor/Destructor

  /*!
   * \brief Create a GuiDevice object
   * \param parent Parent widget of the GuiDevice, should be a pointer to the Tapi::TapiGui object
   * \param type Type of the device (tapi_lib::Device::Type_Publisher for Publisher and ServiceServer or
   * tapi_lib::Device::Type_Subscriber for Subscriber and ServiceClients)
   * \param name Name of the device
   * \param uuid Unique ID of the device
   * \param lastSeq Sequence number of the header of the device when it has been seen for the last time
   * \param lastSeen Timestamp when the device has been seen for the last time
   * \param heartbeat Heartbeat (delay the device shall wait between two Hello-calls) configured for this device
   * \param features Map of Tapi::Feature objects to describe the features of the device
   * \see \c Tapi::Device in the \c tapi_lib package
   * \see \c Device.msg in the \c tapi_lib package
   */
  GuiDevice(QWidget* parent, uint8_t type, std::string name, std::string uuid, unsigned long lastSeq,
            ros::Time lastSeen, unsigned long heartbeat, std::map<std::string, Feature> features);

  //! Empty destructor
  ~GuiDevice();

  // Public member functions
  /*!
   * \brief Calculate the position of the feature box where the drawing of a connection has to start/end
   * \param feature Pointer to the feature whose position has to be calculated
   * \return QPoint with x = left end of the box for type = tapi_lib::Device::Type_Publisher or x = right end of the box
   * for type = tapi_lib::Device::Type_Subscriber), y = half of the height of this box
   * \see \c Device.msg in the \c tapi_lib package
   * \see Tapi::TapiGui::paintEvent
   */
  QPoint FeatureBoxPosition(Tapi::Feature* feature);

  /*!
   * \brief Calculate a color from a string to create unique colors for every ros topic
   *
   * It takes the string and calculates a cryptographic hash (MD5) of it. Then the first byte of the MD5-sum is used as
   * the value for red, the second for green and the third for blue. From this three color values a QColor object is
   * created to color the keys per Topic in Tapi::TapiGui and the feature boxes of the GuiDevice
   * \param messagetype String to build the color, should be the topic type like \c "std_msgs/Bool"
   * \return (Hopefully) unique (if there is no hash collision) color for this string/message-type
   * \see Tapi::TapiGui::checkForGuiUpdate
   * \see Tapi::GuiDevice::paintEvent
   */
  static QColor stringToColor(std::string messagetype);

  /*!
   * \brief Update the data of the device when there is new data from the core
   * \param type Type of the device (tapi_lib::Device::Type_Publisher for Publisher and ServiceServer or
   * tapi_lib::Device::Type_Subscriber for Subscriber and ServiceClients)
   * \param name Name of the device
   * \param uuid Unique ID of the device
   * \param lastSeq Sequence number of the header of the device when it has been seen for the last time
   * \param lastSeen Timestamp when the device has been seen for the last time
   * \param heartbeat Heartbeat (delay the device shall wait between two Hello-calls) configured for this device
   * \param features Map of Tapi::Feature objects to describe the features of the device
   * \see \c Tapi::Device in the \c tapi_lib package
   * \see \c Device.msg in the \c tapi_lib package
   */
  void Update(uint8_t type, std::string name, unsigned long lastSeq, ros::Time lastSeen, unsigned long heartbeat,
              std::map<std::string, Feature> features);

protected:
  // Protected member functions

  /*!
   * \brief Track mouse clicks (or button releases, to be more precise) in the gui
   *
   * Track mouse input of the user to create new connections or delete them. When the event occurs it already means that
   * the click happened on exactly this device object. Then the position of the event is used to calculate on which
   * feature the click happened. Then it emits a featureClicked event where the Tapi::TapiGui object conects to.
   * \param event Information about the click event like its position.
   * \see Tapi::TapiGui::featureClicked
   */
  void mouseReleaseEvent(QMouseEvent* event) Q_DECL_OVERRIDE;

  /*!
   * \brief Overrides the paintEvent of Qt to draw the widget for this device
   *
   * It calculates the necessary size of the outer box for the device, draws thix box with round edges, writes the name
   * of the device and the name for every feature in it and draws connection boxes to the side of the object.
   */
  void paintEvent(QPaintEvent*) Q_DECL_OVERRIDE;

private:
  // Private member variables

  //! Square size of the connection boxes
  int connectbox_size;

  /*!
   * \brief Vector to all elements in the Tapi::Device::features-map of this device, alphabetically sorted
   * \see \c Tapi::Device in the \c tapi_lib package
   * \see \c Tapi::Feature in the \c tapi_lib package
   */
  std::vector<Tapi::Feature*> features;

  //! Height of the footer of the device rectangle
  int footer_height;

  //! Upper coordinate where the footer starts
  int footer_start;

  //! Lower coordinate where the header ends
  int header_end;

  //! Number of items to draw inside of the device rectangle (= number of features)
  int items;

  //! Right end of a feature-"entry" in the device rectangle
  int line_end;

  //! Height of a feature-"entry" in the device rectangle
  int line_height;

  //! Left end of a feature-"entry" in the device rectangle
  int line_start;

  //! Width of a feature-"entry" in the device rectangle
  int line_width;

signals:
  // Signals without implementation

  /*!
   * \brief featureClicked event, emitted when a feature of this GuiDevice has been clicked
   * \param guidevice A pointer to this GuiDevice is forwarded
   * \param feature A Pointer to the clicked Tapi::Feature is forwarded
   * \see \c Tapi::Feature in the \c tapi_lib package
   */
  void featureClicked(Tapi::GuiDevice* guidevice, Tapi::Feature* feature);
};
}

#endif  // GuiDevice_H
