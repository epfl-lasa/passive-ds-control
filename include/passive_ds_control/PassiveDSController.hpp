#ifndef PASSIVEDSCONTROLLER_H
#define PASSIVEDSCONTROLLER_H

#include "eigen3/Eigen/Dense"
#include <vector>
#include "passive_ds_control/PassiveDSTypedefs.hpp"
#include "passive_ds_control/SmoothTruncation.hpp"
#include "passive_ds_control/DSController.hpp"
#include <memory>

namespace PassiveDSControl
{
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
}

#endif // PASSIVEDSCONTROLLER_H
