#ifndef ROBOTIC_H
#define ROBOTIC_H

#include <armadillo>


#define D2R M_PI/180.0
#define R2D 180.0/M_PI

using namespace arma;

enum {
    X = 0,
    Y,
    Z
};

class Robotic
{
public:
    Robotic(int joints, double dh[][3], double limit[][2]);
    ~Robotic();

    void forward(float *joint, mat *ori, mat *pos);
    bool inverse(float *joint, mat R, mat pos);
    mat getPc(mat R, mat pos);

private:
    int m_joints;
    double (*m_dh)[3];
    double (*m_limit)[2];

    mat DH_table(float a, float alpha, float d, float theta);
};

#endif // ROBOTIC_H
