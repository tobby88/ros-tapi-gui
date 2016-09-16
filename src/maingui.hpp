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
 * \file maingui.hpp
 * \ingroup tapi_gui
 * \author Tobias Holst
 * \date 20 Aug 2016
 * \brief Declaration of the Tapi::MainGui-class and definition of its member variables
 */

#ifndef MAINGUI_HPP
#define MAINGUI_HPP

#include <QMainWindow>
#include "ros/node_handle.h"
#include "tapigui.hpp"

namespace Ui
{
class MainGui;
}

namespace Tapi
{
/*!
 * \brief Main window of tapi_gui
 *
 * This class wraps the main windows arund the Tapi::TapiGui class/widget. This makes it easier to control the main
 * window and the widgets for every device independently
 * \author Tobias Holst
 * \version 4.0.1
 */
class MainGui : public QMainWindow
{
  Q_OBJECT

public:
  // Constructor/Destructor

  /*!
   * \brief Create a MainGui-object (main window of tapi_gui)
   * \param nh Pointer to a \c ros::NodeHandle created outside of this class
   * \param parent Pointer to the parent widget of this window - defaults to 0 and shouldn't be changed (or set to 0) on
   * normal execution
   */
  MainGui(ros::NodeHandle* nh, QWidget* parent = 0);

  //! Delete the ui-object of this window
  ~MainGui();

private:
  // Private member variables

  //! Pointer to the Tapi::TapiGui object inside the main window, created when creating the Tapi::MainGui object
  Tapi::TapiGui* tapiGui;

  //! Pointer to the ui object, generated from the ui-designer and created in the constructor
  Ui::MainGui* ui;
};
}

#endif  // MAINGUI_HPP
