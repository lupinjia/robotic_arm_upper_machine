#ifndef ARMENTITY_H
#define ARMENTITY_H

#include <QObject>
#include <Qt3DCore>
#include <Qt3DRender>
#include <Qt3DExtras>
#include <Qt3DRender/QMesh>

class ArmEntity : public QObject
{
    Q_OBJECT
public:
    explicit ArmEntity(Qt3DCore::QEntity* rootEntity, QObject *parent = nullptr);
    ~ArmEntity();
    QList<float> getJointAngles();

signals:

public slots:
    void onSlider1ValueChanged(int value);
    void onSlider2ValueChanged(int value);
    void onSlider3ValueChanged(int value);
    void onSlider4ValueChanged(int value);
    void onSlider5ValueChanged(int value);

private:
    void initEntities();
    void saveInitialRotationZ();
    QList<float> m_initialRotationZ; // save the initial quaternion
    /*---------- link entity ----------*/
    Qt3DCore::QEntity* m_rootEntity;
    Qt3DCore::QEntity* m_link0Entity;
    Qt3DCore::QEntity* m_link1Entity;
    Qt3DCore::QEntity* m_link2Entity;
    Qt3DCore::QEntity* m_link3Entity;
    Qt3DCore::QEntity* m_link4Entity;
    Qt3DCore::QEntity* m_link5Entity;
    /*---------- link transform ----------*/
    Qt3DCore::QTransform* m_link0Transform;
    Qt3DCore::QTransform* m_link1Transform;
    Qt3DCore::QTransform* m_link2Transform;
    Qt3DCore::QTransform* m_link3Transform;
    Qt3DCore::QTransform* m_link4Transform;
    Qt3DCore::QTransform* m_link5Transform;
    /*---------- joint angle ----------*/
    float m_joint1Angle;
    float m_joint2Angle;
    float m_joint3Angle;
    float m_joint4Angle;
    float m_joint5Angle;
    QList<float> m_jointAngles;

};

#endif // ARMENTITY_H
