/**
 * @file   main.cpp
 * @brief  Main to launch the graphic interface.
 * @author Gennaro Raiola
 *
 * This file is part of virtual-fixtures, a set of libraries and programs to create
 * and interact with a library of virtual guides.
 * Copyright (C) 2014-2016 Gennaro Raiola, ENSTA-ParisTech
 *
 * virtual-fixtures is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * virtual-fixtures is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with virtual-fixtures.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "main_window.h"

/// QT
#include <QApplication>

/// ROS
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vf_gui");

    ros::NodeHandle nh;

    QApplication a(argc, argv);
    MainWindow w(nh);

    w.setWindowTitle("Virtual Fixtures - GUI");
    w.show();

    a.exec();

    return 0;
}
