#ifndef ROBOTARM_H
#define ROBOTARM_H

#include <Qt3DCore/qentity.h>
#include <Qt3DCore/qtransform.h>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DRender/QMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QCylinderMesh>
#include "robotic.h"

#define a1    12
#define a2    94
#define a3    89.98
#define a4    27.2
#define d1    61.5
#define d5    46.66
#define d6    90//50
#define JOINT 6

enum {
    J1 = 0,
    J2,
    J3,
    J4,
    J5,
    J6
};

typedef struct _point
{
    float x;
    float y;
    float z;
}POINT;

class RobotArm
{
public:
    RobotArm(Qt3DCore::QEntity *rootEntity);
    ~RobotArm();

    void setJoint(int joint, float *angle);
    void setJoints(float *joints);
    void setPos(float *pos);
    void updateFK(float *pos);
    bool updateIK(float *joint);
    void axisShow(bool en);
    void gripper(bool value);

private:
    Qt3DCore::QEntity *m_rootEntity;
    Qt3DCore::QEntity *m_base, *m_link1, *m_link2, *m_link3, *m_gripper, *m_gripper_open;
    Qt3DCore::QEntity *m_servo1, *m_servo2, *m_servo3, *m_servo4_5, *m_servo6;
    Qt3DCore::QEntity *m_position, *m_axis_x, *m_axis_y, *m_axis_z;
    Qt3DCore::QTransform *m_base_TF, *m_link2_TF, *m_link3_TF, *m_gripper_TF, *m_gripper_open_TF;
    Qt3DCore::QTransform *m_servo1_TF, *m_servo2_TF, *m_servo3_TF, *m_servo4_5_TF, *m_servo6_TF;
    Qt3DCore::QTransform *m_position_TF, *m_axis_x_TF, *m_axis_y_TF, *m_axis_z_TF;
    Qt3DRender::QMesh *m_mesh_base, *m_mesh_link1, *m_mesh_link2, *m_mesh_link3;
    Qt3DRender::QMesh *m_mesh_gripper, *m_mesh_gripper_open;
    Qt3DRender::QMesh *m_mesh_servo1, *m_mesh_servo2, *m_mesh_servo3, *m_mesh_servo4_5, *m_mesh_servo6;
    Qt3DRender::QMesh *m_mesh_axis_x, *m_mesh_axis_y, *m_mesh_axis_z;

    float m_theta[6] = {0};
    float m_theta_d4[6];
    bool isGripperOpen;
    mat m_ori[6];
    mat m_pos[6];
    Robotic *m_robotic;
};

#endif // ROBOTARM_H
