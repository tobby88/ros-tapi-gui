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
 * \defgroup tapi_gui tapi_gui
 * \file main.cpp
 * \ingroup tapi_gui
 * \author Tobias Holst
 * \date 19 Nov 2015
 * \brief Main function of tapi_gui
 */

#include <QApplication>
#include "maingui.hpp"
#include "ros/init.h"
#include "ros/node_handle.h"

/*!
 * \brief Main function of tapi_gui
 *
 * Main function of tapi_gui to initialize ROS, its NodeHandle and then create the Qt-based GUI
 * \param argc Number of arguments when started from the console
 * \param argv \c char pointer to the \c char arrays of the given arguments
 * \return Returns the return value of the QApplication-object
 * \see Tapi::MainGui
 * \see Tapi::TapiGui
 */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Tapi_GUI");
  ros::NodeHandle nh;
  QApplication a(argc, argv);
  Tapi::MainGui w(&nh);
  w.show();
  return a.exec();
}
