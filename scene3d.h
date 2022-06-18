#ifndef SCENE3D_H
#define SCENE3D_H

#include <QObject>
#include <Qt3DCore/qentity.h>
#include <Qt3DExtras/QText2DEntity>
#include "robotarm.h"
#include <vector>

using namespace std;

class Scene3D : public QObject
{
    Q_OBJECT
public:
    explicit Scene3D(Qt3DCore::QEntity *rootEntity);
    void textView(int value);
    void setText(QString str);
    ~Scene3D();

private:
    Qt3DExtras::QCylinderMesh *cylinderZ, *cylinderY, *cylinderX;
    Qt3DCore::QEntity *m_cylinderZEntity, *m_cylinderYEntity, *m_cylinderXEntity;
    Qt3DCore::QEntity *m_rootEntity;
    RobotArm *arm;
    vector<Qt3DCore::QEntity*> pointEntity;
    vector<Qt3DCore::QTransform*> pointTF;
    Qt3DExtras::QSphereMesh *sphereMesh;
    Qt3DExtras::QPhongMaterial *sphereMaterial;
    Qt3DCore::QTransform *textTF;
    Qt3DExtras::QText2DEntity *text2d;

    int angle;

    void drawLine(const QVector3D& start, const QVector3D& end, const QColor& color, Qt3DCore::QEntity *_rootEntity);

public slots:
    void updateAngle(int joint, float *angle, float *pos);
    void updatePos(float *pos, float *joint, bool *valid);
    void updateAll(float *joints, float *pos);
    void axisShow(bool en);
    void gripper(bool value);
    void addPoint(POINT p);
};

#endif
