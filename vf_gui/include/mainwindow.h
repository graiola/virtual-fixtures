#ifndef MAINWINDOW_H
#define MAINWINDOW_H

/// QT
#include <QMainWindow>
#include <QStringListModel>

/// ROS
#include <ros/ros.h>
#include <mechanism_manager/MechanismManagerServices.h>


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
    void UpdateList();
    ros::ServiceClient sc_;
    QStringListModel* NamesModel_;
    QStringList NamesList_;

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
