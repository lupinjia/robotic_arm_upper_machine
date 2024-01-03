#ifndef SERIALPORTCONFIGURATIONWINDOW_H
#define SERIALPORTCONFIGURATIONWINDOW_H

#include <QWidget>
#include <QMessageBox>
#include <QDebug>
/*--------------------Serial Port-------------------*/
#include <QSerialPort>         // 提供访问串口的功能
#include <QSerialPortInfo>     // 提供系统中存在的串口信息

namespace Ui {
class SerialPortConfigurationWindow;
}

class SerialPortConfigurationWindow : public QWidget
{
    Q_OBJECT

public:
    explicit SerialPortConfigurationWindow(QWidget *parent = nullptr);
    ~SerialPortConfigurationWindow();
    bool getSerialIsOpen();
    void setJointAngles(QList<float> jointAngles);
    QList<float> getJointAngles();
    void serialSendFrame();
    void serialRevFrame();

private slots:
    void on_closeWindowButton_clicked();
    void on_portOpenButton_clicked();

private:
    /*---------- 函数 ----------*/
    void serialPortInit();
    void refreshSerialPort(int index);
    QByteArray encodeJointAngles();
    void decodeJointAngles();
    unsigned short calCrc(QByteArray frameData);
    /*---------- 变量 ----------*/
    Ui::SerialPortConfigurationWindow *ui;
    QSerialPort* m_serial;
    bool m_isOpen;
    QList<float> m_jointAngles;
    QByteArray m_sendFrame;
    QByteArray m_revFrame;
};

#endif // SERIALPORTCONFIGURATIONWINDOW_H
