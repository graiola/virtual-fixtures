/**
 * @file   main_window.h
 * @brief  View associated to the list of guides.
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

#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

/// QT
#include <QMainWindow>
//#include <QStringListModel>

/// ROS
#include <ros/ros.h>
//#include <mechanism_manager/MechanismManagerServices.h>

#include "guides_model.h"

namespace Ui {
class MainWindow;
class button;
class label;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(ros::NodeHandle& nh, QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_quitButton_clicked();
    void on_deleteButton_clicked();
    void on_insertButton_clicked();
    void on_refreshButton_clicked();
    void on_saveButton_clicked();
protected:
    void timerEvent(QTimerEvent *event);
    GuidesModel* guides_model_;

private:
    Ui::MainWindow *ui;
};

#endif // MAIN_WINDOW_H
