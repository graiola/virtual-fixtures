#include "mainwindow.h"

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
    w.show();
    return a.exec();

    return 0;
}
