/******************************************************************************
*  Copyright (C) 2016 by Tobias Holst                                         *
*                                                                             *
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
class GuiDevice : public QWidget, public Device
{
  Q_OBJECT

public:
  // Constructor/Destructor
  GuiDevice(QWidget* parent, uint8_t type, std::string name, std::string uuid, unsigned long lastSeq,
            ros::Time lastSeen, unsigned long heartbeat, std::map<std::string, Feature> features);
  ~GuiDevice();

  // Public member functions
  QPoint FeatureBoxPosition(Tapi::Feature* feature);
  static QColor stringToColor(std::string messagetype);
  void Update(uint8_t type, std::string name, unsigned long lastSeq, ros::Time lastSeen, unsigned long heartbeat,
              std::map<std::string, Feature> features);

protected:
  // Protected member functions
  void mouseReleaseEvent(QMouseEvent* event) Q_DECL_OVERRIDE;
  void paintEvent(QPaintEvent*) Q_DECL_OVERRIDE;

private:
  // Private member variables
  int connectbox_size;
  std::vector<Tapi::Feature*> features;
  int footer_height;
  int footer_start;
  int header_end;
  int items;
  int line_end;
  int line_height;
  int line_start;
  int line_width;

signals:
  // Signals without implementation
  void featureClicked(Tapi::GuiDevice* guidevice, Tapi::Feature* feature);
};
}

#endif  // GuiDevice_H
