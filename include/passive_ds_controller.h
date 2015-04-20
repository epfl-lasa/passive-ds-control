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
    Mat damping_;
    Mat basis_;
    Mat damping_eigval_;
    Vec control_output_;
    std::vector<bool> track_dof_;

public:
    //PassiveDSController(int dim,Vec (*velocity_field)(Vec));
    DSController(int dim, realtype damping_eigval0, realtype damping_eigval1);
    Mat ComputeDamping(const Vec& vel);
    void ComputeOrthonormalBasis(const Vec& dir);
    virtual void Update(const Vec& vel, const Vec& ref_vel_c, const Vec &ref_vel_nc);


    void set_damping_eigval(realtype damping_eigval0, realtype damping_eigval1);
    void set_damping_eigval(const Mat& damping_eigval);
    void set_dof_tracking(int dof_id,bool tracking_active=true);

    Mat damping_eigval() const;
    //void setDamping_eigval(const Mat &damping_eigval);
    Vec control_output() const;

};


class PassiveDSController : public DSController
{
private:
    realtype s_;
    realtype s_max_;
    realtype ds_;
    realtype dz_;

    std::unique_ptr<SmoothRise2d> beta_s_;
    std::unique_ptr<SmoothRiseFall2d> beta_r_;
    std::unique_ptr<SmoothRiseFall> alpha_;

public:
    PassiveDSController(int dim, realtype daming_eigval0, realtype damping_eigval1);
    virtual void Update(const Vec &vel, const Vec &ref_vel_c, const Vec &ref_vel_nc);
};

#endif // PASSIVEDSCONTROLLER_H
