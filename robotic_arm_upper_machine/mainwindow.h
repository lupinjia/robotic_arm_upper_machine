#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <Qt3DCore>
#include <Qt3DRender>
#include <Qt3DExtras>
#include <QDebug>
#include <QTimer>

#include "arm_entity.h"
#include "serialportconfigurationwindow.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_serialPortConfigButton_clicked();
    void on_serialSendCheckBox_stateChanged(int checked);
    void serialSendData();
    //void serialRevData();

private:
    Ui::MainWindow *ui;
    SerialPortConfigurationWindow* m_spConfigWindow;
    bool m_isSerialSendChecked;
    ArmEntity* m_armEntity;
    QTimer* m_serialSendTimer;
};
#endif // MAINWINDOW_H
