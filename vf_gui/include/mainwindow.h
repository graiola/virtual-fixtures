#ifndef MAINWINDOW_H
#define MAINWINDOW_H

/// QT
#include <QMainWindow>

/// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mechanism_manager/MechanismManagerAction.h>


namespace Ui {
class MainWindow;
class button;
class label;
}

typedef actionlib::SimpleActionClient<mechanism_manager::MechanismManagerAction> ac_t;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_quitButton_clicked();
    void on_deleteButton_clicked();
    void on_insertButton_clicked();

protected:
    void timerEvent(QTimerEvent *event);
    ac_t ac_;

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
