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
