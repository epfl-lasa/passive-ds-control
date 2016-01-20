#ifndef PASSIVEDSCONTROLLER_H
#define PASSIVEDSCONTROLLER_H

#include "eigen3/Eigen/Dense"
#include "passive_ds_typedefs.h"
#include <vector>
#include "smooth_truncation.h"
#include <memory>

static realtype MINSPEED = 1e-6;
static realtype FLOATEQUAL = 1e-6;

class DSController
{
protected:
    Mat damping_;
    Mat basis_;
    Mat damping_eigval_;
    Vec control_output_;

public:
    //PassiveDSController(int dim,Vec (*velocity_field)(Vec));
    DSController(int dim, realtype damping_eigval0, realtype damping_eigval1);
    Mat ComputeDamping(const Vec& ref_vel);
    void ComputeOrthonormalBasis(const Vec& dir);
    void Update(const Vec& vel, const Vec& ref_vel);

    void set_damping_eigval(realtype damping_eigval0, realtype damping_eigval1);
    void set_damping_eigval(const Mat& damping_eigval);

    Mat damping_eigval() const;
    Vec control_output() const;

};


class PassiveDSController : public DSController
{
private:
    realtype s_;
    realtype s_max_;

    SmoothRise2d beta_r_;
    SmoothRiseFall2d beta_s_;
    SmoothRiseFall alpha_;
public:
    PassiveDSController(int dim, realtype damping_eigval0, realtype damping_eigval1, realtype s_max, realtype ds, realtype dz=0.0);
    void UpdatePassive(const Vec& vel, const Vec& ref_vel,realtype dt);
    void UpdatePassive(const Vec &vel, const Vec &ref_vel_c, const Vec &ref_vel_nc,realtype dt);
    void reset_storage();
    realtype s() const;
};

#endif // PASSIVEDSCONTROLLER_H
