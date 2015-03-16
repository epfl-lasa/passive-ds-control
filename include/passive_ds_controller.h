#ifndef PASSIVEDSCONTROLLER_H
#define PASSIVEDSCONTROLLER_H

#include "eigen3/Eigen/Dense"
#include "passive_ds_typedefs.h"


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
    Mat ComputeDamping(const Vec& vel);
    void ComputeOrthonormalBasis(const Vec& dir);

    void set_damping_eigval(realtype damping_eigval0, realtype damping_eigval1);
    void set_damping_eigval(const Mat& damping_eigval);

    Mat damping_eigval() const;
    //void setDamping_eigval(const Mat &damping_eigval);
};

#endif // PASSIVEDSCONTROLLER_H
