#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    /*---------- variable initialization ----------*/
    m_isSerialSendChecked = false;
    /*---------- Qt3D configurations ----------*/
    //创建3d窗口
    Qt3DExtras::Qt3DWindow *view = new Qt3DExtras::Qt3DWindow();
    view->defaultFrameGraph()->setClearColor(QColor(QRgb(0x4d4d4f)));
    QWidget *container = QWidget::createWindowContainer(view);
    QSize screenSize = view->screen()->size();
    container->setMinimumSize(QSize(200, 100));
    container->setMaximumSize(screenSize);
    ui->hLayout->addWidget(container,1);
    // 1.define Root entity
    Qt3DCore::QEntity *rootEntity = new Qt3DCore::QEntity();
    // 2.define Camera
    Qt3DRender::QCamera *cameraEntity = view->camera();
    cameraEntity->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    cameraEntity->setPosition(QVector3D(-0.04, 0.7f, 0.08f)); //set the camera's position
    cameraEntity->setUpVector(QVector3D(0, 0, 1)); // set where the upside of the camera points at
    cameraEntity->setViewCenter(QVector3D(-0.04, 0, 0.08)); //where the camera looks at
    // light
    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(rootEntity);
    Qt3DRender::QPointLight *light = new Qt3DRender::QPointLight(lightEntity); // light is a component, needs to be added to an entity
    light->setColor("white");
    light->setIntensity(1);
    lightEntity->addComponent(light);
    Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity); // transform is a component of an entity
    lightTransform->setTranslation(cameraEntity->position());
    lightEntity->addComponent(lightTransform);
    // For camera controls
    Qt3DExtras::QOrbitCameraController *camController = new Qt3DExtras::QOrbitCameraController(rootEntity);
    camController->setCamera(cameraEntity);
    // Set root object of the scene
    view->setRootEntity(rootEntity);
    // create arm entity
    m_armEntity = new ArmEntity(rootEntity);

    /*---------- Serial Port Configuration Window ----------*/
    m_spConfigWindow = new SerialPortConfigurationWindow();
    /*---------- Timer Initialization ----------*/
    m_serialSendTimer = new QTimer(this);

    /*---------- connect signals and slots ----------*/
    QObject::connect(ui->horizontalSlider1, &QSlider::valueChanged, m_armEntity, &ArmEntity::onSlider1ValueChanged);
    QObject::connect(ui->horizontalSlider2, &QSlider::valueChanged, m_armEntity, &ArmEntity::onSlider2ValueChanged);
    QObject::connect(ui->horizontalSlider3, &QSlider::valueChanged, m_armEntity, &ArmEntity::onSlider3ValueChanged);
    QObject::connect(ui->horizontalSlider4, &QSlider::valueChanged, m_armEntity, &ArmEntity::onSlider4ValueChanged);
    QObject::connect(ui->horizontalSlider5, &QSlider::valueChanged, m_armEntity, &ArmEntity::onSlider5ValueChanged);
    QObject::connect(m_serialSendTimer, &QTimer::timeout, this, &MainWindow::serialSendData);

    //qDebug() << "sizeof unsigned short" << sizeof(unsigned short);

}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_serialPortConfigButton_clicked()
{
    //显示串口设置窗口
    m_spConfigWindow->show();
}

// 发送checkBox槽函数,使能时通过串口定时发送每个关节的角度数据
void MainWindow::on_serialSendCheckBox_stateChanged(int checked)
{
    m_isSerialSendChecked = checked;
    //select current checkbox
    if(m_isSerialSendChecked)
    {
        //串口未开启,报错
        if(!m_spConfigWindow->getSerialIsOpen())
        {
            QMessageBox::warning(this,tr("错误"),tr("未开启串口,请先开启串口"),QMessageBox::Ok);
            return;
        }
        //没有错误,正常运行
        else
        {
            // 5Hz
            m_serialSendTimer->start(100);
        }
    }
    //unselect
    else
    {
        m_serialSendTimer->stop();
    }


}

void MainWindow::serialSendData()
{
    // debug info
    qDebug() <<"";
    qDebug() << "timeout slot serialSendData";
    // refresh joint angles and send 1 frame of data
    m_spConfigWindow->setJointAngles(m_armEntity->getJointAngles());
    qDebug() << "send joint angles" << m_armEntity->getJointAngles();
    m_spConfigWindow->serialSendFrame();
}

//void MainWindow::serialRevData()
//{
//    //debug info
//    qDebug() <<"";
//    qDebug() << "timeout slot serialRevData";
//    //receive 1 frame of data and update joint angles
//    m_spConfigWindow->serialRevFrame();
//    QList<float> jointAngles = m_spConfigWindow->getJointAngles();
//    qDebug() << "receive joint angles" << jointAngles;
//}

