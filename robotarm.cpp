#include "robotarm.h"

double dh_table[6][3] = { {a1, 90, d1},
                         {a2,  0,  0},
                         {a3,  0,  0},
                         {a4, 90,  0},
                         { 0,  0, d5},
                         { 0,  0, d6}
                       };

double limit[6][2] = { { -90.0,  90.0},
                      {   0.0, 180.0},
                      {-135.0,   0.0},
                      {   0.0,  90.0},
                      { -90.0,  90.0},
                      {   0.0,   1.0}
                    };

RobotArm::RobotArm(Qt3DCore::QEntity *rootEntity)
    : m_rootEntity(rootEntity)
{
    m_robotic = new Robotic(JOINT, dh_table, limit);
    m_theta[1] = 90;
    m_theta[2] = -90;
    m_theta[3] = 90;
    m_theta[4] = 0;
    m_theta_d4[1] = 90;
    m_theta_d4[2] = -90;
    m_theta_d4[3] = 90;
    m_theta_d4[4] = 0;

    for (int i = 0; i < JOINT; i++)
    {
        m_pos[i] = zeros<mat>(3,1);
        m_ori[i] = zeros<mat>(3,3);
    }
    isGripperOpen = false;
    m_robotic->forward(m_theta, m_ori, m_pos);
//    qDebug()<<m_pos[0][0]<<", "<<m_pos[0][1]<<", "<<m_pos[0][2];
    qDebug()<<m_pos[5](0)<<", "<<m_pos[5](1)<<", "<<m_pos[5](2);
    // Base
    m_mesh_base = new Qt3DRender::QMesh();
    m_mesh_base->setSource(QUrl::fromLocalFile("./resource/model/base.obj"));
    m_base_TF = new Qt3DCore::QTransform();
    m_base_TF->setTranslation(QVector3D(0.0f, 0.0f, 0.0f));
    Qt3DExtras::QPhongMaterial *joint1Material = new Qt3DExtras::QPhongMaterial();
    joint1Material->setDiffuse(QColor(QRgb(0x4d4d4f)));
    m_base = new Qt3DCore::QEntity(m_rootEntity);
    m_base->addComponent(m_mesh_base);
    m_base->addComponent(joint1Material);
    m_base->addComponent(m_base_TF);

    // Servo1
    m_mesh_servo1 = new Qt3DRender::QMesh();
    m_mesh_servo1->setSource(QUrl::fromLocalFile("./resource/model/servo1.obj"));
    m_servo1_TF = new Qt3DCore::QTransform();
    m_servo1_TF->setTranslation(QVector3D(0.0f, 0.0f, 22.0f));
    Qt3DExtras::QPhongMaterial *joint1Materia2 = new Qt3DExtras::QPhongMaterial();
    joint1Materia2->setDiffuse(QColor(QRgb(0x303030)));
    m_servo1 = new Qt3DCore::QEntity(m_rootEntity);
    m_servo1->addComponent(m_mesh_servo1);
    m_servo1->addComponent(joint1Materia2);
    m_servo1->addComponent(m_servo1_TF);

    // Link1
    m_mesh_link1 = new Qt3DRender::QMesh();
    m_mesh_link1->setSource(QUrl::fromLocalFile("./resource/model/link1.obj"));
    Qt3DExtras::QPhongMaterial *link1Materia = new Qt3DExtras::QPhongMaterial();
    link1Materia->setDiffuse(QColor(QRgb(0x0080FF)));
    m_link1 = new Qt3DCore::QEntity(m_rootEntity);
    m_link1->addComponent(m_mesh_link1);
    m_link1->addComponent(link1Materia);

    // Servo2
    m_mesh_servo2 = new Qt3DRender::QMesh();
    m_mesh_servo2->setSource(QUrl::fromLocalFile("./resource/model/servo2.obj"));
    m_servo2_TF = new Qt3DCore::QTransform();
    m_servo2_TF->setTranslation(QVector3D(m_pos[0](0), m_pos[0](1), m_pos[0](2)));
    Qt3DExtras::QPhongMaterial *servo2Materia = new Qt3DExtras::QPhongMaterial();
    servo2Materia->setDiffuse(QColor(QRgb(0x303030)));
    m_servo1 = new Qt3DCore::QEntity(m_rootEntity);
    m_servo1->addComponent(m_mesh_servo2);
    m_servo1->addComponent(servo2Materia);
    m_servo1->addComponent(m_servo2_TF);

    // Link2
    m_mesh_link2 = new Qt3DRender::QMesh();
    m_mesh_link2->setSource(QUrl::fromLocalFile("./resource/model/link2.obj"));
    m_link2_TF = new Qt3DCore::QTransform();
    m_link2_TF->setTranslation(QVector3D(m_pos[0](0), m_pos[0](1), m_pos[0](2)));
    Qt3DExtras::QPhongMaterial *link2Materia = new Qt3DExtras::QPhongMaterial();
    link2Materia->setDiffuse(QColor(QRgb(0x0080FF)));
    m_link2 = new Qt3DCore::QEntity(m_rootEntity);
    m_link2->addComponent(m_mesh_link2);
    m_link2->addComponent(link2Materia);
    m_link2->addComponent(m_link2_TF);

    // Servo3
    m_mesh_servo3 = new Qt3DRender::QMesh();
    m_mesh_servo3->setSource(QUrl::fromLocalFile("./resource/model/servo3.obj"));
    m_servo3_TF = new Qt3DCore::QTransform();
    m_servo3_TF->setTranslation(QVector3D(m_pos[1](0), m_pos[1](1), m_pos[1](2)));
    Qt3DExtras::QPhongMaterial *servo3Materia = new Qt3DExtras::QPhongMaterial();
    servo3Materia->setDiffuse(QColor(QRgb(0x303030)));
    m_servo3 = new Qt3DCore::QEntity(m_rootEntity);
    m_servo3->addComponent(m_mesh_servo3);
    m_servo3->addComponent(servo3Materia);
    m_servo3->addComponent(m_servo3_TF);

    // Link3
    m_mesh_link3 = new Qt3DRender::QMesh();
    m_mesh_link3->setSource(QUrl::fromLocalFile("./resource/model/link3.obj"));
    m_link3_TF = new Qt3DCore::QTransform();
    m_link3_TF->setTranslation(QVector3D(m_pos[1](0), m_pos[1](1), m_pos[1](2)));
    Qt3DExtras::QPhongMaterial *link3Materia = new Qt3DExtras::QPhongMaterial();
    link3Materia->setDiffuse(QColor(QRgb(0x0080FF)));
    m_link3 = new Qt3DCore::QEntity(m_rootEntity);
    m_link3->addComponent(m_mesh_link3);
    m_link3->addComponent(link3Materia);
    m_link3->addComponent(m_link3_TF);

    // Servo4-5
    m_mesh_servo4_5 = new Qt3DRender::QMesh();
    m_mesh_servo4_5->setSource(QUrl::fromLocalFile("./resource/model/servo4-5.obj"));
    m_servo4_5_TF = new Qt3DCore::QTransform();
    m_servo4_5_TF->setTranslation(QVector3D(m_pos[2](0), m_pos[2](1), m_pos[2](2)));
    Qt3DExtras::QPhongMaterial *servo45Materia = new Qt3DExtras::QPhongMaterial();
    servo45Materia->setDiffuse(QColor(QRgb(0x303030)));
    m_servo4_5 = new Qt3DCore::QEntity(m_rootEntity);
    m_servo4_5->addComponent(m_mesh_servo4_5);
    m_servo4_5->addComponent(servo45Materia);
    m_servo4_5->addComponent(m_servo4_5_TF);

    // Servo6
    m_mesh_servo6 = new Qt3DRender::QMesh();
    m_mesh_servo6->setSource(QUrl::fromLocalFile("./resource/model/servo6.obj"));
    m_servo6_TF = new Qt3DCore::QTransform();
//    m_servo6_TF->setTranslation(QVector3D(-89.98f-12-46.66, 0, 155.5f+22));
    m_servo6_TF->setTranslation(QVector3D(m_pos[4](0), m_pos[4](1), m_pos[4](2)));
    Qt3DExtras::QPhongMaterial *servo6Materia = new Qt3DExtras::QPhongMaterial();
    servo6Materia->setDiffuse(QColor(QRgb(0x303030)));
    m_servo6 = new Qt3DCore::QEntity(m_rootEntity);
    m_servo6->addComponent(m_mesh_servo6);
    m_servo6->addComponent(servo6Materia);
    m_servo6->addComponent(m_servo6_TF);

    // Gripper
    m_mesh_gripper = new Qt3DRender::QMesh();
    m_mesh_gripper->setSource(QUrl::fromLocalFile("./resource/model/gripper.obj"));
    m_gripper_TF = new Qt3DCore::QTransform();
//    m_gripper_TF->setTranslation(QVector3D(-89.98f-12-46.66, 0, 155.5f+22));
    m_gripper_TF->setTranslation(QVector3D(m_pos[4](0), m_pos[4](1), m_pos[4](2)));
    Qt3DExtras::QPhongMaterial *gripperMateria = new Qt3DExtras::QPhongMaterial();
    gripperMateria->setDiffuse(QColor(QRgb(0x808080)));
    m_gripper = new Qt3DCore::QEntity(m_rootEntity);
    m_gripper->addComponent(m_mesh_gripper);
    m_gripper->addComponent(gripperMateria);
    m_gripper->addComponent(m_gripper_TF);

    // Gripper-Open
    m_mesh_gripper_open = new Qt3DRender::QMesh();
    m_mesh_gripper_open->setSource(QUrl::fromLocalFile("./resource/model/gripper_open.obj"));
    m_gripper_open_TF = new Qt3DCore::QTransform();
    m_gripper_open_TF->setTranslation(QVector3D(m_pos[4](0), m_pos[4](1), m_pos[4](2)));
    m_gripper_open = new Qt3DCore::QEntity(m_rootEntity);
    m_gripper_open->addComponent(m_mesh_gripper_open);
    m_gripper_open->addComponent(m_gripper_open_TF);
    m_gripper_open->removeComponent(m_mesh_gripper_open);

    // Position
    Qt3DExtras::QSphereMesh *sphereMesh = new Qt3DExtras::QSphereMesh();
    sphereMesh->setRings(20);
    sphereMesh->setSlices(20);
    sphereMesh->setRadius(10);
    m_position_TF = new Qt3DCore::QTransform();
    m_position_TF->setTranslation(QVector3D(m_pos[5](0)-(d5+d6)*m_ori[5](0,2)-a4*m_ori[5](0,0)*0,
                                            m_pos[5](1)-(d5+d6)*m_ori[5](1,2)-a4*m_ori[5](1,0)*0,
                                            m_pos[5](2)-(d5+d6)*m_ori[5](2,2)-a4*m_ori[5](2,0)*0));
    Qt3DExtras::QPhongMaterial *sphereMaterial = new Qt3DExtras::QPhongMaterial();
    sphereMaterial->setDiffuse(QColor(QRgb(0xff00ff)));
    m_position = new Qt3DCore::QEntity(m_rootEntity);
//    m_position->addComponent(sphereMesh);
    m_position->addComponent(sphereMaterial);
    m_position->addComponent(m_position_TF);

    // Axis
    m_mesh_axis_z = new Qt3DRender::QMesh();
    m_mesh_axis_z->setSource(QUrl::fromLocalFile("./resource/model/axis-z.obj"));
    m_axis_z_TF = new Qt3DCore::QTransform();
    QMatrix4x4 matrix;
    matrix = m_axis_z_TF->matrix();
    matrix.setToIdentity();
    matrix.rotate(m_theta[J1], QVector3D(0.0f, 0.0f, 1.0f));
    matrix.rotate(-(m_theta[J2]+m_theta[J3]+m_theta[J4]+180),
                  QVector3D(0.0f, 1.0f, 0.0f));
    matrix.rotate(m_theta[J5], QVector3D(-1.0f, 0.0f, 0.0f));
    m_axis_z_TF->setMatrix(matrix);
    m_axis_z_TF->setTranslation(QVector3D(m_pos[5](0), m_pos[5](1), m_pos[5](2)));
    Qt3DExtras::QPhongMaterial *axixZMaterial = new Qt3DExtras::QPhongMaterial();
    axixZMaterial->setDiffuse(QColor(QRgb(0x0000ff)));
    m_axis_z = new Qt3DCore::QEntity(m_rootEntity);
    m_axis_z->addComponent(m_mesh_axis_z);
    m_axis_z->addComponent(axixZMaterial);
    m_axis_z->addComponent(m_axis_z_TF);
    m_axis_z->removeComponent(m_mesh_axis_z);

    m_mesh_axis_y = new Qt3DRender::QMesh();
    m_mesh_axis_y->setSource(QUrl::fromLocalFile("./resource/model/axis-y.obj"));
    m_axis_y_TF = new Qt3DCore::QTransform();
    matrix = m_axis_y_TF->matrix();
    matrix.setToIdentity();
    matrix.rotate(m_theta[J1], QVector3D(0.0f, 0.0f, 1.0f));
    matrix.rotate(-(m_theta[J2]+m_theta[J3]+m_theta[J4]+180),
                  QVector3D(0.0f, 1.0f, 0.0f));
    matrix.rotate(m_theta[J5], QVector3D(-1.0f, 0.0f, 0.0f));
    m_axis_y_TF->setMatrix(matrix);
    m_axis_y_TF->setTranslation(QVector3D(m_pos[5](0), m_pos[5](1), m_pos[5](2)));
    Qt3DExtras::QPhongMaterial *axixYMaterial = new Qt3DExtras::QPhongMaterial();
    axixYMaterial->setDiffuse(QColor(QRgb(0x00ff00)));
    m_axis_y = new Qt3DCore::QEntity(m_rootEntity);
    m_axis_y->addComponent(m_mesh_axis_y);
    m_axis_y->addComponent(axixYMaterial);
    m_axis_y->addComponent(m_axis_y_TF);
    m_axis_y->removeComponent(m_mesh_axis_y);

    m_mesh_axis_x = new Qt3DRender::QMesh();
    m_mesh_axis_x->setSource(QUrl::fromLocalFile("./resource/model/axis-x.obj"));
    m_axis_x_TF = new Qt3DCore::QTransform();
    matrix = m_axis_x_TF->matrix();
    matrix.setToIdentity();
    matrix.rotate(m_theta[J1], QVector3D(0.0f, 0.0f, 1.0f));
    matrix.rotate(-(m_theta[J2]+m_theta[J3]+m_theta[J4]+180),
                  QVector3D(0.0f, 1.0f, 0.0f));
    matrix.rotate(m_theta[J5], QVector3D(-1.0f, 0.0f, 0.0f));
    m_axis_x_TF->setMatrix(matrix);
    m_axis_x_TF->setTranslation(QVector3D(m_pos[5](0), m_pos[5](1), m_pos[5](2)));
    Qt3DExtras::QPhongMaterial *axixXMaterial = new Qt3DExtras::QPhongMaterial();
    axixXMaterial->setDiffuse(QColor(QRgb(0xff0000)));
    m_axis_x = new Qt3DCore::QEntity(m_rootEntity);
    m_axis_x->addComponent(m_mesh_axis_x);
    m_axis_x->addComponent(axixXMaterial);
    m_axis_x->addComponent(m_axis_x_TF);
    m_axis_x->removeComponent(m_mesh_axis_x);
}

RobotArm::~RobotArm()
{

}

void RobotArm::setJoint(int joint, float *angle)
{
    if (*angle > limit[joint][1])
        *angle = limit[joint][1];
    else if (*angle < limit[joint][0])
        *angle = limit[joint][0];

    m_theta[joint] = *angle;
}

void RobotArm::setJoints(float *joints)
{
    m_theta[J1] = joints[J1];
    m_theta[J2] = joints[J2];
    m_theta[J3] = joints[J3];
    m_theta[J4] = joints[J4];
    m_theta[J5] = joints[J5];
}

void RobotArm::gripper(bool value)
{
    isGripperOpen = value;

    QMatrix4x4 matrix = m_gripper_TF->matrix();
    matrix.setToIdentity();
    matrix.rotate(m_theta[J1], QVector3D(0.0f, 0.0f, 1.0f));
    matrix.rotate(-(m_theta[J2]+m_theta[J3]+m_theta[J4]-m_theta_d4[J2]-m_theta_d4[J3]-m_theta_d4[J4]),
                  QVector3D(0.0f, 1.0f, 0.0f));
    matrix.rotate(m_theta[J5], QVector3D(-1.0f, 0.0f, 0.0f));
    m_gripper_TF->setMatrix(matrix);
    m_gripper_TF->setTranslation(QVector3D(m_pos[4](0), m_pos[4](1), m_pos[4](2)));
    if (isGripperOpen)
    {
        m_gripper->addComponent(m_mesh_gripper_open);
        m_gripper->removeComponent(m_mesh_gripper);
    }
    else {
        m_gripper->addComponent(m_mesh_gripper);
        m_gripper->removeComponent(m_mesh_gripper_open);
    }
}

void RobotArm::setPos(float *pos)
{
    m_pos[JOINT-1](0) = pos[0];
    m_pos[JOINT-1](1) = pos[1];
    m_pos[JOINT-1](2) = pos[2];
}

bool RobotArm::updateIK(float *joint)
{
#if 0
    mat pc(3,1);

    float s234 = sqrt(pow(m_ori[5](0,2), 2) + pow(m_ori[5](1,2), 2));
    float theta1 = atan2(m_ori[5](1,2), m_ori[5](0,2));
    if (theta1*R2D < limit[0][0])
        theta1 += M_PI;
    else if (theta1*R2D > limit[0][1])
        theta1 -= M_PI;
    float c1c234 = cos(theta1)*(-m_ori[5](2,2));
    float s1c234 = sin(theta1)*(-m_ori[5](2,2));
    pc(0) = m_pos[5](0)-(d5+d6)*m_ori[5](0,2)-a4*c1c234;
    pc(1) = m_pos[5](1)-(d5+d6)*m_ori[5](1,2)-a4*s1c234;
    pc(2) = m_pos[5](2)-(d5+d6)*m_ori[5](2,2)-a4*s234;
//    m_position_TF->setTranslation(QVector3D(m_pos[5](0)-(d5+d6)*m_ori[5](0,2)-a4*c1c234,
//                                            m_pos[5](1)-(d5+d6)*m_ori[5](1,2)-a4*s1c234,
//                                            m_pos[5](2)-(d5+d6)*m_ori[5](2,2)-a4*s234));

//    qDebug()<<"x: "<<pc(0)<<", "<<"y: "<<pc(1)<<", "<<"z: "<<pc(2);
#endif
    return m_robotic->inverse(joint, m_ori[JOINT-1], m_pos[5]);
}

void RobotArm::updateFK(float *pos)
{
    m_robotic->forward(m_theta, m_ori, m_pos);

//    qDebug()<<"FK 1:"<<m_theta[J1]<<", 2:"<<m_theta[J2]<<", 3:"<<m_theta[J3]<<", 4:"<<m_theta[J4];

    pos[0] = m_pos[JOINT-1](0);
    pos[1] = m_pos[JOINT-1](1);
    pos[2] = m_pos[JOINT-1](2);
    // Servo2
    m_servo2_TF->setRotationZ(m_theta[J1]);
    m_servo2_TF->setTranslation(QVector3D(m_pos[0](0), m_pos[0](1), m_pos[0](2)));

    // Link2
    QMatrix4x4 matrix;
    matrix = m_link2_TF->matrix();
    matrix.setToIdentity();
    matrix.rotate(m_theta[J1], QVector3D(0.0f, 0.0f, 1.0f));
    matrix.rotate(-(m_theta[J2]-m_theta_d4[J2]), QVector3D(0.0f, 1.0f, 0.0f));
    m_link2_TF->setMatrix(matrix);
    m_link2_TF->setTranslation(QVector3D(m_pos[0](0), m_pos[0](1), m_pos[0](2)));

    // Servo3
    matrix = m_servo3_TF->matrix();
    matrix.setToIdentity();
    matrix.rotate(m_theta[J1], QVector3D(0.0f, 0.0f, 1.0f));
    matrix.rotate(-(m_theta[J2]+m_theta[J3]-m_theta_d4[J2]-m_theta_d4[J3]),
                  QVector3D(0.0f, 1.0f, 0.0f));
    m_servo3_TF->setMatrix(matrix);
    m_servo3_TF->setTranslation(QVector3D(m_pos[1](0), m_pos[1](1), m_pos[1](2)));

    // Link3
    matrix = m_link3_TF->matrix();
    matrix.setToIdentity();
    matrix.rotate(m_theta[J1], QVector3D(0.0f, 0.0f, 1.0f));
    matrix.rotate(-(m_theta[J2]+m_theta[J3]-m_theta_d4[J2]-m_theta_d4[J3]),
                  QVector3D(0.0f, 1.0f, 0.0f));
    m_link3_TF->setMatrix(matrix);
    m_link3_TF->setTranslation(QVector3D(m_pos[1](0), m_pos[1](1), m_pos[1](2)));

    // Servo4_5
    matrix = m_servo4_5_TF->matrix();
    matrix.setToIdentity();
    matrix.rotate(m_theta[J1], QVector3D(0.0f, 0.0f, 1.0f));
    matrix.rotate(-(m_theta[J2]+m_theta[J3]+m_theta[J4]-m_theta_d4[J2]-m_theta_d4[J3]-m_theta_d4[J4]),
                  QVector3D(0.0f, 1.0f, 0.0f));
    m_servo4_5_TF->setMatrix(matrix);
    m_servo4_5_TF->setTranslation(QVector3D(m_pos[2](0), m_pos[2](1), m_pos[2](2)));

    // Gripper
    matrix = m_gripper_TF->matrix();
    matrix.setToIdentity();
    matrix.rotate(m_theta[J1], QVector3D(0.0f, 0.0f, 1.0f));
    matrix.rotate(-(m_theta[J2]+m_theta[J3]+m_theta[J4]-m_theta_d4[J2]-m_theta_d4[J3]-m_theta_d4[J4]),
                  QVector3D(0.0f, 1.0f, 0.0f));
    matrix.rotate(m_theta[J5], QVector3D(-1.0f, 0.0f, 0.0f));
    m_gripper_TF->setMatrix(matrix);
    m_gripper_TF->setTranslation(QVector3D(m_pos[4](0), m_pos[4](1), m_pos[4](2)));
    if (isGripperOpen)
    {
        m_gripper->addComponent(m_mesh_gripper_open);
        m_gripper->removeComponent(m_mesh_gripper);
    }
    else {
        m_gripper->addComponent(m_mesh_gripper);
        m_gripper->removeComponent(m_mesh_gripper_open);
    }

    // Servo6
    matrix = m_servo6_TF->matrix();
    matrix.setToIdentity();
    matrix.rotate(m_theta[J1], QVector3D(0.0f, 0.0f, 1.0f));
    matrix.rotate(-(m_theta[J2]+m_theta[J3]+m_theta[J4]-m_theta_d4[J2]-m_theta_d4[J3]-m_theta_d4[J4]),
                  QVector3D(0.0f, 1.0f, 0.0f));
    matrix.rotate(m_theta[J5], QVector3D(-1.0f, 0.0f, 0.0f));
    m_servo6_TF->setMatrix(matrix);
    m_servo6_TF->setTranslation(QVector3D(m_pos[4](0), m_pos[4](1), m_pos[4](2)));

    // Position
    matrix = m_position_TF->matrix();
    matrix.setToIdentity();
    matrix.rotate(m_theta[J1], QVector3D(0.0f, 0.0f, 1.0f));
    matrix.rotate(-(m_theta[J2]+m_theta[J3]+m_theta[J4]-m_theta_d4[J2]-m_theta_d4[J3]-m_theta_d4[J4]),
                  QVector3D(0.0f, 1.0f, 0.0f));
    matrix.rotate(m_theta[J5], QVector3D(-1.0f, 0.0f, 0.0f));
    m_position_TF->setMatrix(matrix);
    float s234 = sqrt(pow(m_ori[5](0,2), 2) + pow(m_ori[5](1,2), 2));
    float theta1 = atan2(m_ori[5](1,2), m_ori[5](0,2));
    if (theta1*R2D < limit[0][0])
        theta1 += M_PI;
    else if (theta1*R2D > limit[0][1])
        theta1 -= M_PI;
//    qDebug()<<"theta1: "<<theta1*R2D;
//    qDebug()<<"c234: "<<-m_ori[5](2,2);
//    qDebug()<<"m_ori[5](0,2): "<<m_ori[5](0,2);
    float c1c234 = cos(theta1)*(-m_ori[5](2,2));
    float s1c234 = sin(theta1)*(-m_ori[5](2,2));
    mat out = m_robotic->getPc(m_ori[5], m_pos[5]);
//    m_position_TF->setTranslation(QVector3D(out(0), out(1), out(2)));
    m_position_TF->setTranslation(QVector3D(m_pos[5](0)-(d5+d6)*m_ori[5](0,2)-a4*c1c234,
                                            m_pos[5](1)-(d5+d6)*m_ori[5](1,2)-a4*s1c234,
                                            m_pos[5](2)-(d5+d6)*m_ori[5](2,2)-a4*s234));

    // Axis
    matrix = m_axis_z_TF->matrix();
    matrix.setToIdentity();
    matrix.rotate(m_theta[J1], QVector3D(0.0f, 0.0f, 1.0f));
    matrix.rotate(-(m_theta[J2]+m_theta[J3]+m_theta[J4]+180),
                  QVector3D(0.0f, 1.0f, 0.0f));
    m_axis_z_TF->setMatrix(matrix);
    m_axis_z_TF->setTranslation(QVector3D(m_pos[5](0), m_pos[5](1), m_pos[5](2)));

    matrix = m_axis_y_TF->matrix();
    matrix.setToIdentity();
    matrix.rotate(m_theta[J1], QVector3D(0.0f, 0.0f, 1.0f));
    matrix.rotate(-(m_theta[J2]+m_theta[J3]+m_theta[J4]+180),
                  QVector3D(0.0f, 1.0f, 0.0f));
    m_axis_y_TF->setMatrix(matrix);
    m_axis_y_TF->setTranslation(QVector3D(m_pos[5](0), m_pos[5](1), m_pos[5](2)));

    matrix = m_axis_x_TF->matrix();
    matrix.setToIdentity();
    matrix.rotate(m_theta[J1], QVector3D(0.0f, 0.0f, 1.0f));
    matrix.rotate(-(m_theta[J2]+m_theta[J3]+m_theta[J4]+180),
                  QVector3D(0.0f, 1.0f, 0.0f));
    m_axis_x_TF->setMatrix(matrix);
    m_axis_x_TF->setTranslation(QVector3D(m_pos[5](0), m_pos[5](1), m_pos[5](2)));

}

void RobotArm::axisShow(bool en)
{
    if (en)
    {
        m_axis_x->addComponent(m_mesh_axis_x);
        m_axis_y->addComponent(m_mesh_axis_y);
        m_axis_z->addComponent(m_mesh_axis_z);
    }
    else {
        m_axis_x->removeComponent(m_mesh_axis_x);
        m_axis_y->removeComponent(m_mesh_axis_y);
        m_axis_z->removeComponent(m_mesh_axis_z);
    }
}
