#ifndef PASSIVEDSCONTROLLER_H
#define PASSIVEDSCONTROLLER_H

#include "eigen3/Eigen/Dense"

typedef float realtype;
#ifdef USE_DOUBLE_PRECISION
typedef double realtype;
#endif
typedef Eigen::Matrix<realtype,Eigen::Dynamic,Eigen::Dynamic> Mat;
typedef Eigen::Matrix<realtype,Eigen::Dynamic,1> Vec;

#define MINSPEED 0.001

class PassiveDSController
{
    int dim_;
    Mat damping_;
    Mat basis_;
    Mat damping_eigval_;
    // function for getting desired motion flow
    //Vec (*velocity_field_)(Vec);
public:
    //PassiveDSController(int dim,Vec (*velocity_field)(Vec));
    PassiveDSController(int dim);
    Mat ComputeDamping(Vec pos);
    void ComputeOrthogonalBasis(Vec dir);

    void set_damping_eigval(realtype damping_eigval0, realtype damping_eigval1);
};

#endif // PASSIVEDSCONTROLLER_H
