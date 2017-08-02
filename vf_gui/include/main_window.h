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
#include <QColorDialog>
#include <QThread>

/// ROS
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>

#include "guides_model.h"

namespace Ui {
class MainWindow;
class button;
class label;
class radioButton;
class slider;
class textEdit;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(ros::NodeHandle& nh, QWidget *parent = 0);
    ~MainWindow();
    void loggerCallback(const rosgraph_msgs::Log::ConstPtr& msg);

signals:
    void requestUpdateConsole(const QString& data , int level);

private slots:

    void updateConsole(const QString& data , int level);

    void on_quitButton_clicked();
    void on_deleteButton_clicked();
    void on_insertButton_clicked();
    void on_refreshButton_clicked();
    void on_saveButton_clicked();
    void on_softRadioButton_clicked();
    void on_hardRadioButton_clicked();
    void on_clearButton_clicked();
    void on_recordButton_toggled(bool checked);

protected:
    void timerEvent(QTimerEvent *event);
    GuidesModel* guides_model_;
    ros::Subscriber* sub_;
    ros::AsyncSpinner* spinner_ptr_; // Used to keep alive the ros callbacks

private:
    Ui::MainWindow *ui;
};

#endif // MAIN_WINDOW_H
