#include "serialportconfigurationwindow.h"
#include "ui_serialportconfigurationwindow.h"

SerialPortConfigurationWindow::SerialPortConfigurationWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SerialPortConfigurationWindow)
{
    ui->setupUi(this);
    serialPortInit();
    /*---------- initialize joint angles list ----------*/
    m_jointAngles.clear();
    // first append, then you can acess corresponding indices
    for(int i = 0; i < 6; i++)
    {
        m_jointAngles.append(0);
    }

}

SerialPortConfigurationWindow::~SerialPortConfigurationWindow()
{
    delete ui;
}

bool SerialPortConfigurationWindow::getSerialIsOpen()
{
    return m_isOpen;
}

void SerialPortConfigurationWindow::setJointAngles(QList<float> jointAngles)
{
    m_jointAngles = jointAngles;
}

QList<float> SerialPortConfigurationWindow::getJointAngles()
{
    return m_jointAngles;
}
// send data of 1 frame via serial port
void SerialPortConfigurationWindow::serialSendFrame()
{
    m_sendFrame.resize(16); //长为16字节
    m_sendFrame[0] = 0xA8; //帧头
    QByteArray frameData = encodeJointAngles(); //得到数据
    // 将数据放入数据帧中指定位置
    for(int i = 0; i < frameData.size(); i++)
    {
        m_sendFrame[1+i] = frameData[i];
    }
    // 计算crc校验位
    unsigned short crc = calCrc(frameData);
    // 校验位放入数据帧中
    m_sendFrame[13] = crc & 0xFF; //低8位
    m_sendFrame[14] = (crc >> 8) & 0xFF; //高8位
    // 帧尾
    m_sendFrame[15] = 0xFE;
    // 调试信息
    qDebug() << "send bytes" << m_sendFrame;
    // 发送数据
    m_serial->write(m_sendFrame);
}

//void SerialPortConfigurationWindow::serialRevFrame()
//{
//    // 串口接收数据
//    m_revFrame.resize(14);
//    // 测试解码,先用发送的
//    m_revFrame = m_sendFrame;
//    // 调试信息
//    qDebug() << "receive bytes" << m_revFrame;
//    // 解码数据
//    decodeJointAngles();
//}

void SerialPortConfigurationWindow::serialPortInit()
{
    m_serial = new QSerialPort;
    // if opened, close it
    if(m_serial->isOpen())
    {
        m_serial->clear();
        m_serial->close();
    }
    //串口为关闭状态
    m_isOpen = false;

    // 初始参数配置
    ui->baudComboBox->setCurrentIndex(5);// 初始波特率    115200
    ui->parityComboBox->setCurrentIndex(0);// 初始校验位  0
    ui->dataBitComboBox->setCurrentIndex(3);// 初始数据位 8
    ui->stopBitComboBox->setCurrentIndex(0);//初始停止位 1

    // 刷新串口
    refreshSerialPort(0);
}

void SerialPortConfigurationWindow::refreshSerialPort(int index)
{
    QStringList portNameList;                                        // 存储所有串口名
    if(index != 0)
    {
        m_serial->setPortName(ui->portComboBox->currentText());             //设置串口号
    }
    else
    {
        ui->portComboBox->clear();                                           //关闭串口号
        ui->portComboBox->addItem("刷新");                                    //添加刷新
        foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts()) //添加新串口
        {
            portNameList.append(info.portName());
        }
        ui->portComboBox->addItems(portNameList);
        ui->portComboBox->setCurrentIndex(1);                               // 当前串口号为刷新下的第一个
        m_serial->setPortName(ui->portComboBox->currentText());             //设置串口号
    }
}
// 将关节角度进行编码,转换成放入数据帧中的12个字节
QByteArray SerialPortConfigurationWindow::encodeJointAngles()
{
    QByteArray frameData;
    frameData.resize(12); //数据帧中数据长度为12个字节
    for(int i = 0; i < 6; i++)
    {
        // 将浮点的角度乘以100后转换为int
        unsigned int angle = (unsigned int)(m_jointAngles[i]);
        // 取低8位
        frameData[i*2] = angle & 0xFF;
        // 取高8位
        frameData[i*2+1] = (angle >> 8) & 0xFF;
    }
    return frameData;
}

void SerialPortConfigurationWindow::decodeJointAngles()
{
    QByteArray frameData;
    frameData.resize(12);
    // 从接收到的数据帧中提取数据
    for(int i = 0; i < frameData.size(); i++)
    {
        frameData[i] = m_revFrame[1+i];
    }
    // 解码得关节角度
    for(int i = 0; i < 6; i++)
    {
        unsigned int angle = frameData[i*2] + (frameData[i*2+1] << 8);
        float jointAngle = (float)angle;
        m_jointAngles[i] = jointAngle;
    }
}

unsigned short SerialPortConfigurationWindow::calCrc(QByteArray frameData)
{
    int k = 0;
    int len = frameData.size();
    unsigned short crc = 0xFFFF;  //crc16位寄存器初始值

    while(len--)
    {
        crc ^= frameData[k++];
        for (int i = 0; i < 8; i++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001; //多项式 POLY（0x8005)的高低位交换值，这是由于其模型的一些参数决定的
            else
                crc = (crc >> 1);
        }
    }

    return crc;
}

void SerialPortConfigurationWindow::on_closeWindowButton_clicked()
{
    this->close();
}


void SerialPortConfigurationWindow::on_portOpenButton_clicked()
{
    if(m_serial->isOpen()) //如果串口打开了，给他关闭
    {
        //设置串口的通讯配置
        m_serial->clear();
        m_serial->close();

        //设置UI
        ui->portOpenButton->setText("Open Serial Port");// 关闭状态，按钮显示“打开串口”
        ui->baudComboBox->setEnabled(true);      // 关闭状态，允许用户操作
        ui->parityComboBox->setEnabled(true);// 关闭状态，允许用户操作
        ui->dataBitComboBox->setEnabled(true);// 关闭状态，允许用户操作
        ui->stopBitComboBox->setEnabled(true);// 关闭状态，允许用户操作
        m_isOpen = false;
    }
    else //如果串口关闭了，给他打开
    {
        //设置串口的通讯配置
        m_serial->setPortName(ui->portComboBox->currentText());// 当前选择的串口名字
        m_serial->setBaudRate(ui->baudComboBox->currentText().toInt());// 波特率

            //parity 设置校验位
            switch (ui->parityComboBox->currentIndex()) {
            case 0:     //无校验
                m_serial->setParity(QSerialPort::NoParity);
                break;
            case 1:     //偶校验
                m_serial->setParity(QSerialPort::EvenParity);
                break;
            case 2:    //奇校验
                m_serial->setParity(QSerialPort::OddParity);
                break;
            case 3:    //奇校验
                m_serial->setParity(QSerialPort::MarkParity);
                break;
            default:    //奇校验
                m_serial->setParity(QSerialPort::NoParity);
                break;
            }

            //data bits 设置数据位
            switch (ui->dataBitComboBox->currentText().toInt()) {
            case 8:
                m_serial->setDataBits(QSerialPort::Data8);
                break;
            case 7:
                m_serial->setDataBits(QSerialPort::Data7);
                break;
            case 6:
                m_serial->setDataBits(QSerialPort::Data6);
                break;
            case 5:
                m_serial->setDataBits(QSerialPort::Data5);
                break;
            default:
                m_serial->setDataBits(QSerialPort::Data8);
                break;
            }

            //stop bits 设置停止位
            switch (ui->stopBitComboBox->currentIndex()) {
            case 0:
                m_serial->setStopBits(QSerialPort::OneStop);
                break;
            case 1:
                m_serial->setStopBits(QSerialPort::OneAndHalfStop);
                break;
            case 2:
                m_serial->setStopBits(QSerialPort::TwoStop);
                break;
            default:
                m_serial->setStopBits(QSerialPort::OneStop);
                break;
            }

            //流控制
            m_serial->setFlowControl(QSerialPort::NoFlowControl);

            //用ReadWrite 的模式尝试打开串口，无法收发数据时，发出警告
            if(!m_serial->open(QIODevice::ReadWrite))
            {
                QMessageBox::warning(this,tr("提示"),tr("串口打开失败!"),QMessageBox::Ok);
                return;
            }

            //设置UI
            ui->portOpenButton->setText("Close Serial Port");// 打开状态，按钮显示“关闭串口”
            ui->baudComboBox->setEnabled(false);// 打开状态，禁止用户操作
            ui->parityComboBox->setEnabled(false);// 打开状态，禁止用户操作
            ui->dataBitComboBox->setEnabled(false);// 打开状态，禁止用户操作
            ui->stopBitComboBox->setEnabled(false);// 打开状态，禁止用户操作
            m_isOpen = true;

            }
}

