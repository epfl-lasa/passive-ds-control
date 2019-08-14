#ifndef DSCONTROLLER_H
#define DSCONTROLLER_H

#include "eigen3/Eigen/Dense"
#include <vector>
#include <memory>
#include "passive_ds_control/PassiveDSTypedefs.hpp"
#include "passive_ds_control/SmoothTruncation.hpp"

namespace PassiveDSControl
{
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
}
#endif