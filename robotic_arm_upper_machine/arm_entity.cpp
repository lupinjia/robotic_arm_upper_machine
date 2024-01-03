#include "arm_entity.h"

ArmEntity::ArmEntity(Qt3DCore::QEntity* rootEntity, QObject *parent)
    : QObject{parent}
{
    /*---------- initialize entities and components ----------*/
    m_rootEntity = rootEntity;
    initEntities();
    /*---------- initialize entities and components ----------*/
    saveInitialRotationZ();
    /*---------- initialize joint angles list ----------*/
    m_jointAngles.clear();
    // first append, then you can acess corresponding indices
    for(int i = 0; i < 6; i++)
    {
        m_jointAngles.append(0);
    }


}

ArmEntity::~ArmEntity()
{

}

QList<float> ArmEntity::getJointAngles()
{
    //get current joint angles(except gripper joint)
    m_joint1Angle = m_link1Transform->rotationZ() - m_initialRotationZ.at(0);
    m_joint2Angle = m_link2Transform->rotationZ() - m_initialRotationZ.at(1);
    m_joint3Angle = m_link3Transform->rotationZ() - m_initialRotationZ.at(2);
    m_joint4Angle = -(m_link4Transform->rotationZ() - m_initialRotationZ.at(3));
    m_joint5Angle = m_link5Transform->rotationZ() - m_initialRotationZ.at(4);
    //update joint angles in list
    m_jointAngles.replace(0, m_joint1Angle);
    m_jointAngles.replace(1, m_joint2Angle);
    m_jointAngles.replace(2, m_joint3Angle);
    m_jointAngles.replace(3, m_joint4Angle);
    m_jointAngles.replace(4, m_joint5Angle);
    m_jointAngles.replace(5, m_joint6Angle);

    return m_jointAngles;
}

// use float for compatibility
void ArmEntity::onSlider1ValueChanged(int value)
{
    float angleDeg = value;// convert to deg joint angle, [0,90]
    float targetAng = m_initialRotationZ.at(0) + angleDeg;
    m_link1Transform->setRotationZ(targetAng);
}

void ArmEntity::onSlider2ValueChanged(int value)
{
    float angleDeg = value;// convert to deg joint angle, [0,90]
    float targetAng = m_initialRotationZ.at(1) + angleDeg;
    m_link2Transform->setRotationZ(targetAng);
}

void ArmEntity::onSlider3ValueChanged(int value)
{
    float angleDeg = value;// convert to deg joint angle, [0,90]
    float targetAng = m_initialRotationZ.at(2) + angleDeg;
    m_link3Transform->setRotationZ(targetAng);
}

void ArmEntity::onSlider4ValueChanged(int value)
{
    float angleDeg = value;// convert to deg joint angle, [0,90]
    float targetAng = m_initialRotationZ.at(3) + (-angleDeg); // wrist1的正方向与实物相反
    m_link4Transform->setRotationZ(targetAng);
}

void ArmEntity::onSlider5ValueChanged(int value)
{
    float angleDeg = value;// convert to deg joint angle, [0,90]
    float targetAng = m_initialRotationZ.at(4) + angleDeg;
    m_link5Transform->setRotationZ(targetAng);
}

// gripper is different from others
void ArmEntity::onSlider6ValueChanged(int value)
{
    m_joint6Angle = (float)value;
}



void ArmEntity::initEntities()
{
    /*---------- link0 ----------*/
    // mesh
    Qt3DRender::QMesh* link0Mesh = new Qt3DRender::QMesh();
    QUrl data = QUrl::fromLocalFile("D:/develop/qt/projects/robotic_arm_upper_machine/robotic_arm_upper_machine/mesh/link0.STL");
    link0Mesh->setSource(data);
    // transform
    m_link0Transform = new Qt3DCore::QTransform();
    m_link0Transform->setScale(1.0f);
    m_link0Transform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(0.0f, 0.0f, 1.0f), 0.0f));
    m_link0Transform->setTranslation(QVector3D(0.0f, 0.0f, 0.0f));
    // material
    Qt3DExtras::QPhongMaterial* link0Material = new Qt3DExtras::QPhongMaterial();
    link0Material->setDiffuse(QColor(QRgb(0xbeb32b)));
    // entity
    m_link0Entity = new Qt3DCore::QEntity(m_rootEntity);
    m_link0Entity->addComponent(link0Mesh);
    m_link0Entity->addComponent(link0Material);
    m_link0Entity->addComponent(m_link0Transform);

    /*---------- link1 ----------*/
    // mesh
    Qt3DRender::QMesh* link1Mesh = new Qt3DRender::QMesh();
    data = QUrl::fromLocalFile("D:/develop/qt/projects/robotic_arm_upper_machine/robotic_arm_upper_machine/mesh/link1.STL");
    link1Mesh->setSource(data);
    // transform
    m_link1Transform = new Qt3DCore::QTransform();
    m_link1Transform->setScale(1.0f);
    //m_link1Transform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1.0f, 0.0f, 0.0f), 90.0f));
    m_link1Transform->setTranslation(QVector3D(0.0f, 0.0f, 0.0f));
    // material
    Qt3DExtras::QPhongMaterial* link1Material = new Qt3DExtras::QPhongMaterial();
    link1Material->setDiffuse(QColor(QRgb(0xbeb32b)));
    // entity
    m_link1Entity = new Qt3DCore::QEntity(m_link0Entity);
    m_link1Entity->addComponent(link1Mesh);
    m_link1Entity->addComponent(link1Material);
    m_link1Entity->addComponent(m_link1Transform);
    /*---------- link2 ----------*/
    // mesh
    Qt3DRender::QMesh* link2Mesh = new Qt3DRender::QMesh();
    data = QUrl::fromLocalFile("D:/develop/qt/projects/robotic_arm_upper_machine/robotic_arm_upper_machine/mesh/link2.STL");
    link2Mesh->setSource(data);
    // transform
    m_link2Transform = new Qt3DCore::QTransform();
    m_link2Transform->setScale(1.0f);
    m_link2Transform->setRotationX(-90.0);
    m_link2Transform->setRotationZ(-16.0); // joint2往下转是负的方向,往上转是正方向 [-10,90]
    m_link2Transform->setTranslation(QVector3D(0, -0.0241400064300461, 0.043));
    // material
    Qt3DExtras::QPhongMaterial* link2Material = new Qt3DExtras::QPhongMaterial();
    link2Material->setDiffuse(QColor(QRgb(0xbeb32b)));
    // entity
    m_link2Entity = new Qt3DCore::QEntity(m_link1Entity);
    m_link2Entity->addComponent(link2Mesh);
    m_link2Entity->addComponent(link2Material);
    m_link2Entity->addComponent(m_link2Transform);
    /*---------- link3 ----------*/
    // mesh
    Qt3DRender::QMesh* link3Mesh = new Qt3DRender::QMesh();
    data = QUrl::fromLocalFile("D:/develop/qt/projects/robotic_arm_upper_machine/robotic_arm_upper_machine/mesh/link3.STL");
    link3Mesh->setSource(data);
    // transform
    m_link3Transform = new Qt3DCore::QTransform();
    m_link3Transform->setScale(1.0f);
    //m_link3Transform->setRotation(QQuaternion::fromEulerAngles(180.0f, 0.0f, (-0.26277/3.14159)*180.0));
    m_link3Transform->setRotationX(180.0f);
    //m_link3Transform->setRotationZ((0.26277/3.14159)*180.0);
    m_link3Transform->setRotationZ(30.0f); // [-10,90]
    m_link3Transform->setTranslation(QVector3D(-0.17844f, -0.048814f, 0.0084f));
    // material
    Qt3DExtras::QPhongMaterial* link3Material = new Qt3DExtras::QPhongMaterial();
    link3Material->setDiffuse(QColor(QRgb(0xbeb32b)));
    // entity
    m_link3Entity = new Qt3DCore::QEntity(m_link2Entity);
    m_link3Entity->addComponent(link3Mesh);
    m_link3Entity->addComponent(link3Material);
    m_link3Entity->addComponent(m_link3Transform);
    /*---------- link4 ----------*/
    // mesh
    Qt3DRender::QMesh* link4Mesh = new Qt3DRender::QMesh();
    data = QUrl::fromLocalFile("D:/develop/qt/projects/robotic_arm_upper_machine/robotic_arm_upper_machine/mesh/link4.STL");
    link4Mesh->setSource(data);
    // transform
    m_link4Transform = new Qt3DCore::QTransform();
    m_link4Transform->setScale(1.0f);
    //m_link4Transform->setRotationZ(-0.47421/3.14159*180.0);
    m_link4Transform->setRotationZ(105.0f);
    m_link4Transform->setTranslation(QVector3D(0.1175f, 0.0f, 0.005f));
    // material
    Qt3DExtras::QPhongMaterial* link4Material = new Qt3DExtras::QPhongMaterial();
    link4Material->setDiffuse(QColor(QRgb(0xbeb32b)));
    // entity
    m_link4Entity = new Qt3DCore::QEntity(m_link3Entity);
    m_link4Entity->addComponent(link4Mesh);
    m_link4Entity->addComponent(link4Material);
    m_link4Entity->addComponent(m_link4Transform);
    /*---------- link5 ----------*/
    // mesh
    Qt3DRender::QMesh* link5Mesh = new Qt3DRender::QMesh();
    data = QUrl::fromLocalFile("D:/develop/qt/projects/robotic_arm_upper_machine/robotic_arm_upper_machine/mesh/link5.STL");
    link5Mesh->setSource(data);
    // transform
    m_link5Transform = new Qt3DCore::QTransform();
    m_link5Transform->setScale(1.0f);
    //m_link5Transform->setRotation(QQuaternion::fromEulerAngles(-90.0, 1.4814/3.14159*180.0, -90.0));
    m_link5Transform->setRotationY(91.0);
    m_link5Transform->setRotationZ(80.0);
    m_link5Transform->setTranslation(QVector3D(0.08f, -0.00025f, -0.015859f));
    // material
    Qt3DExtras::QPhongMaterial* link5Material = new Qt3DExtras::QPhongMaterial();
    link5Material->setDiffuse(QColor(QRgb(0xbeb32b)));
    // entity
    m_link5Entity = new Qt3DCore::QEntity(m_link4Entity);
    m_link5Entity->addComponent(link5Mesh);
    m_link5Entity->addComponent(link5Material);
    m_link5Entity->addComponent(m_link5Transform);
}

void ArmEntity::saveInitialRotationZ()
{
    m_initialRotationZ.append(m_link1Transform->rotationZ());
    m_initialRotationZ.append(m_link2Transform->rotationZ());
    m_initialRotationZ.append(m_link3Transform->rotationZ());
    m_initialRotationZ.append(m_link4Transform->rotationZ());
    m_initialRotationZ.append(m_link5Transform->rotationZ());
}


