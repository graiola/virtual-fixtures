#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
//#include <mechanism_manager/mechanism_manager_interface.h>

namespace Ui {
class MainWindow;
class button;
class label;
}

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

private:
    Ui::MainWindow *ui;
    //org::VirtualFixture::MechanismManager::Interface* mm_;
};

#endif // MAINWINDOW_H
