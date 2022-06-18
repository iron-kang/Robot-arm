#include "robotic.h"
#include <QDebug>

Robotic::Robotic(int joints, double dh[][3], double limit[][2])
{
    m_joints = joints;
    m_dh     = dh;
    m_limit  = limit;

}

Robotic::~Robotic()
{
}

mat Robotic::DH_table(float a, float alpha, float d, float theta)
{
    mat A = mat(4,4);
    A<<cos(theta*D2R)<<-sin(theta*D2R)*cos(alpha*D2R)<<sin(theta*D2R)*sin(alpha*D2R)<<a*cos(theta*D2R)<<endr
     <<sin(theta*D2R)<<cos(theta*D2R)*cos(alpha*D2R)<<-cos(theta*D2R)*sin(alpha*D2R)<<a*sin(theta*D2R)<<endr
     <<0<<sin(alpha*D2R)<<cos(alpha*D2R)<<d<<endr
     <<0<<0<<0<<1<<endr;

//    A.print("A:");
    return A;
}

mat Robotic::getPc(mat R, mat pos)
{
    float s234 = sqrt(pow(R(0,2), 2) + pow(R(1,2), 2));
    float theta1 = atan2(R(1,2), R(0,2));
    float c1c234 = cos(theta1)*(-R(2,2));
    float s1c234 = sin(theta1)*(-R(2,2));
    mat out(3,1);

    out(0) = pos(0)-(m_dh[4][2]+m_dh[5][2])*R(0,2)-m_dh[3][0]*c1c234;
    out(1) = pos(1)-(m_dh[4][2]+m_dh[5][2])*R(1,2)-m_dh[3][0]*s1c234;
    out(2) = pos(2)-(m_dh[4][2]+m_dh[5][2])*R(2,2)-m_dh[3][0]*s234;

    out(0) = fabs(out(0)) < 1e-3 ? 0 : out(0);
    out(1) = fabs(out(1)) < 1e-3 ? 0 : out(1);
    out(2) = fabs(out(2)) < 1e-3 ? 0 : out(2);

    return out;
}

void Robotic::forward(float *joint, mat *ori, mat *pos)
{
    mat T = eye<mat>(4,4);
    mat p;
    for (int i = 0; i < m_joints; i++)
    {
        T = T*DH_table(m_dh[i][0], m_dh[i][1], m_dh[i][2], joint[i]);
        ori[i] = T(0, 0, size(3,3));
        pos[i] = T(0, 3, size(3,1));
    }
}

bool Robotic::inverse(float *joint, mat R, mat p)
{
    double theta1, theta2, theta3, theta4, theta5;
    double a1, d1, a2, a3;

    mat pos = getPc(R, p);
    theta1 = atan2(pos(1), pos(0));
//    theta1 = atan2(R(1,2), R(0,2));
//    qDebug()<<"theta1: "<<theta1*R2D<<", "<<pos(0)<<", "<<pos(1)<<(theta1*R2D) -  m_limit[0][0];
//    if (((theta1*R2D) - m_limit[0][0]) < -1e-3)
    if ((theta1*R2D) < m_limit[0][0])
        theta1 += M_PI;
    else if ((theta1*R2D) > m_limit[0][1])
//        else if (((theta1*R2D) - m_limit[0][1]) > 1e-3)
        theta1 -= M_PI;
    else if (abs(theta1*R2D) < 0.001)
        theta1 = 0;
//    qDebug()<<"theta1: "<<theta1;
//    pos.print("pos:");
    a1 = m_dh[0][0];
    d1 = m_dh[0][2];
    a2 = m_dh[1][0];
    a3 = m_dh[2][0];
    pos(0) = pos(0) - a1*cos(theta1);
    pos(0) = fabs(pos(0)) < 1e-6 ? 0 : pos(0);
    pos(1) = pos(1) - a1*sin(theta1);
    pos(1) = fabs(pos(1)) < 1e-6 ? 0 : pos(1);
    pos(2) = pos(2) - d1;
    pos(2) = fabs(pos(2)) < 1e-6 ? 0 : pos(2);
    mat norm = pow(pos, 2);
//    qDebug()<<"xy: "<<sqrt(norm(0)+norm(1))<<", z: "<<pos(2);
    theta3 = acos((accu(norm)-pow(a2,2)-pow(a3, 2))/(2*a2*a3));
//    qDebug()<<"theta3: "<<-theta3*R2D;
    if ((-theta3*R2D > m_limit[2][1]) || (-theta3*R2D < m_limit[2][0]) || (theta3 != theta3))
        return false;
//    theta2 = atan2(pos(2),sqrt(norm(0)+norm(1))) - atan2(a3*sin(theta3), a2+a3*cos(theta3));

    if ((pos(0)*cos(theta1) < 0))// || (pos(1)*sin(theta1) < 0))
    {
//        qDebug()<<"1"<<pos(0)<<", "<<pos(1);
        theta2 = atan2(pos(2),-sqrt(norm(0)+norm(1))) - atan2(a3*sin(-theta3), a2+a3*cos(-theta3));
    }
    else
        theta2 = atan2(pos(2),sqrt(norm(0)+norm(1))) - atan2(a3*sin(-theta3), a2+a3*cos(-theta3));

//    qDebug()<<"theta2: "<<theta2*R2D;
    if ((theta2*R2D > m_limit[1][1]) || (theta2 *R2D< m_limit[1][0]) || (theta2 != theta2))
        return false;
//    qDebug()<<"theta2-2: "<<theta2*180/M_PI<<"..."<<atan2(pos(2),sqrt(norm(0)+norm(1)))<<"..."<<atan2(a3*sin(-theta3), a2+a3*cos(-theta3));
//    qDebug()<<a2*cos(theta[1]*D2R)<<", "<<-a3*cos(theta[1]*D2R+theta[2]*D2R);

//    float s4 = cos(theta1)*cos(theta2-theta3)*R(0,2) + sin(theta1)*cos(theta2-theta3)*R(1,2) + sin(theta2-theta3)*R(2,2);
//    if (abs(s4) > 0.99)
    {
        float c4 = -(-cos(theta1)*sin(theta2-theta3)*R(0,2) - sin(theta1)*sin(theta2-theta3)*R(1,2) + cos(theta2-theta3)*R(2,2));
        theta4 = acos(c4);
    }
//    else
//        theta4 = asin(s4);
    if (theta4 != theta4)
        return false;

    theta4 = theta4 > m_limit[3][1]*D2R ? m_limit[3][1]*D2R : theta4;
    theta4 = theta4 < m_limit[3][0]*D2R ? m_limit[3][0]*D2R : theta4;

    float s5 = sin(theta1)*R(0,0) - cos(theta1)*R(1,0);

    theta5 = asin(s5);
//    qDebug()<<"theta4: "<<theta4*R2D;
//    qDebug()<<"theta5: "<<theta5*R2D;
    joint[0] = theta1*R2D;
    joint[1] = theta2*R2D;
    joint[2] = -theta3*R2D;
    joint[3] = theta4*R2D;
    joint[4] = theta5*R2D;

    return true;
}
