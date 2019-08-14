#include "passive_ds_control/PassiveDSController.hpp"

namespace PassiveDSControl
{
    realtype PassiveDSController::s() const
    {
        return s_;
    }

    PassiveDSController::PassiveDSController(int dim, realtype damping_eigval0, realtype damping_eigval1, realtype s_max, realtype ds, realtype dz):
        DSController(dim, damping_eigval0, damping_eigval1),
        s_(s_max),
        s_max_(s_max),
        beta_r_(0.0,dz,0.0,ds),
        beta_s_(0.0,0.0,dz,0.0,s_max,ds),
        alpha_(0.0,0.0+ds,s_max-ds,s_max)
    {

    }

    void PassiveDSController::UpdatePassive(const Vec &vel, const Vec &ref_vel, realtype dt)
    {
        // assume all of ref_vel is non-conservative
        UpdatePassive(vel,Vec::Zero(ref_vel.rows()),ref_vel,dt);
    }

    void PassiveDSController::UpdatePassive(const Vec &vel, const Vec &ref_vel_c,const Vec &ref_vel_nc,realtype dt)
    {
        Vec ref_vel = ref_vel_c + ref_vel_nc;
        // compute damping
        ComputeDamping(ref_vel);
        // dissipate
        control_output_ = -damping_*vel;
        // compute control for conservative part
        control_output_ += damping_eigval_(0)*ref_vel_c;
        // below we add the non-conservative part if allowed
        // non-conservative energy change
        realtype z = vel.dot(ref_vel_nc);
        if(fabs(z)<0.01)
            z = 0.0;
        //std::cout<<"value of z:"<< z <<std::endl;
        // add the non-censervative driving control
        control_output_ += damping_eigval_(0)*beta_r_(z,s_)*ref_vel_nc;
        //std::cout<<"value of beta_R: "<<beta_r_(z,s_)<<std::endl;
        // update storage
        realtype sdot = alpha_(s_)*vel.transpose()*damping_*vel;
        sdot -= beta_s_(z,s_)*damping_eigval_(0)*z;
        s_ += sdot*dt;
    }

    void PassiveDSController::reset_storage()
    {
        s_ = s_max_;
    }
}
