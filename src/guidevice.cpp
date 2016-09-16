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
 * \file guidevice.cpp
 * \ingroup tapi_gui
 * \author Tobias Holst
 * \date 10 Aug 2016
 * \brief Definition of the Tapi::GuiDevice-class and its member functions
 */

#include "guidevice.hpp"
#include <QCryptographicHash>
#include <QFont>
#include <QPainter>
#include <QPoint>
#include <QRect>
#include <QSize>
#include <QString>
#include "tapi_lib/Device.h"
#include "tapi_lib/HelloRequest.h"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

GuiDevice::GuiDevice(QWidget* parent, uint8_t type, string name, string uuid, unsigned long lastSeq, ros::Time lastSeen,
                     unsigned long heartbeat, map<string, Feature> features)
  : QWidget(parent), Device(type, name, uuid, lastSeq, lastSeen, heartbeat, features)
{
  // Define some standard values
  connectbox_size = 10;
  header_end = 30;
  footer_height = 10;
  line_height = 20;
  line_start = connectbox_size;
  this->features = GetSortedFeatures();
  items = this->features.size();

  // Set the height of this widget to a fixed size
  this->setMinimumHeight(header_end + line_height * items + footer_height);
  this->setMaximumHeight(header_end + line_height * items + footer_height);

  // Prevent uninitialized usage of some member variables
  footer_start = 0;
  line_end = 0;
  line_width = 0;
}

GuiDevice::~GuiDevice()
{
}

// Public member functions

QPoint GuiDevice::FeatureBoxPosition(Tapi::Feature* feature)
{
  int x, y, i;

  // Calculate y-coordinate (left end or right end of this widget, depending on the device type)
  y = 0;
  if (GetType() == tapi_lib::Device::Type_Subscriber)
    x = 0;
  else
    x = this->width() - 1;

  // Calculate x-coordinate of the feature (middle of it) by searching from top to bottom through the features
  i = 0;
  for (auto it = features.begin(); it != features.end(); ++it)
  {
    if (*it == feature)
      y = header_end + i * line_height + line_height / 2;
    i++;
  }
  return QPoint(x, y);
}

// Public member functions
QColor GuiDevice::stringToColor(string messagetype)
{
  // Use MD5 method of Qt to calculate the MD5 of the given string
  QCryptographicHash hasher(QCryptographicHash::Md5);
  hasher.addData(messagetype.c_str());
  QByteArray hash = hasher.result();

  // Take the first three bytes of the MD5 to generate the color
  uint8_t red, green, blue;
  red = hash.at(0);
  green = hash.at(1);
  blue = hash.at(2);
  QColor color;
  color = QColor(red, green, blue);
  return color;
}

void GuiDevice::Update(uint8_t type, string name, unsigned long lastSeq, ros::Time lastSeen, unsigned long heartbeat,
                       map<string, Feature> features)
{
  // Update the Device object we are inheriting from and update our own alphabetically sorted features-vector
  this->features.clear();
  Device::Update(type, name, lastSeq, lastSeen, heartbeat, features);
  this->features = GetSortedFeatures();
  items = this->features.size();

  // Set the new height for this widget
  this->setMinimumHeight(header_end + line_height * items + footer_height);
  this->setMaximumHeight(header_end + line_height * items + footer_height);
}

// Protected member functions

void GuiDevice::mouseReleaseEvent(QMouseEvent* event)
{
  // Check if the click happened to a feature, emit the featureClicked event then
  int i = 0;
  QPoint p = event->pos();
  for (auto it = features.begin(); it != features.end(); ++it)
  {
    int y = header_end + i * line_height;

    if (p.y() >= y && p.y() < y + line_height)
    {
      emit featureClicked(this, *it);
    }
    i++;
  }
}

void GuiDevice::paintEvent(QPaintEvent*)
{
  QPainter painter(this);

  // Calculate the necessary values for drawing from the size of this widget
  footer_start = this->height() - footer_height;
  line_width = this->width() - 2 * connectbox_size - 1;
  line_end = line_start + line_width;

  // Draw main rectangle with round edges
  if (Active())
    painter.setBrush(Qt::white);
  else
    painter.setBrush(Qt::gray);
  painter.setPen(Qt::black);
  QSize a(line_width, this->height() - 1);
  painter.drawRoundedRect(QRect(QPoint(line_start, 0), a), 10, 10);

  // Print heading
  painter.setFont(QFont("Arial", 11));
  painter.drawText(QRect(QPoint(line_start, 0), QPoint(line_end, header_end)), Qt::AlignCenter,
                   QString::fromStdString(GetName()));

  // Draw Features
  int i = 0;
  for (auto it = features.begin(); it != features.end(); ++it)
  {
    int line_y = header_end + i * line_height;

    QColor color;
    color = stringToColor((*it)->GetType());
    if (!Active())
      color = color.darker();
    painter.setBrush(color);

    painter.drawLine(QPoint(line_start, line_y), QPoint(line_end, line_y));

    // Draw connection box
    QRect connect_box(QPoint(0, line_y + line_height / 2 - connectbox_size / 2),
                      QSize(connectbox_size, connectbox_size));

    if (GetType() == tapi_lib::Device::Type_Publisher)
      connect_box.moveLeft(line_end);

    painter.drawRect(connect_box);

    // Draw the name of the feature
    painter.setFont(QFont("Arial", 10));
    painter.drawText(QRect(QPoint(line_start, line_y), QPoint(line_end, line_y + line_height)), Qt::AlignCenter,
                     QString::fromStdString((*it)->GetName()));
    i++;
  }

  // Draw footer
  painter.drawLine(QPoint(line_start, footer_start), QPoint(line_end, footer_start));
}
}
