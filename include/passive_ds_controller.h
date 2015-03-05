#ifndef PASSIVEDSCONTROLLER_H
#define PASSIVEDSCONTROLLER_H

#include "eigen3/Eigen/Dense"

typedef float realtype;
#ifdef USE_DOUBLE_PRECISION
typedef double realtype;
#endif
typedef Eigen::Matrix<realtype,Eigen::Dynamic,Eigen::Dynamic> Mat;
typedef Eigen::Matrix<realtype,Eigen::Dynamic,1> Vec;


static realtype MINSPEED = 1e-6;
static realtype FLOATEQUAL = 1e-6;

class PassiveDSController
{
    Mat damping_;
    Mat basis_;
    Mat damping_eigval_;

public:
    //PassiveDSController(int dim,Vec (*velocity_field)(Vec));
    PassiveDSController(int dim, realtype damping_eigval0, realtype damping_eigval1);
    Mat ComputeDamping(Vec pos);
    void ComputeOrthonormalBasis(Vec dir);

    void set_damping_eigval(realtype damping_eigval0, realtype damping_eigval1);
};

#endif // PASSIVEDSCONTROLLER_H
