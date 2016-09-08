
#include "openloop_ds_controller.h"


Vec OpenloopDSController::control_output() const
{
    return control_output_;
}

Vec OpenloopDSController::ref_vel() const
{
    return ref_vel_;
}

Vec OpenloopDSController::ref_pos() const
{
    return ref_pos_+ds_origin_;
}

Vec OpenloopDSController::start_pos() const
{
    return start_pos_;
}

OpenloopDSController::OpenloopDSController(size_t dim, std::function<Vec(Vec)> task_dynamics) : dim_(dim), task_dynamics_(task_dynamics)
{
    // make a test call to the supplied dynamics function and filter function
    Vec temp(dim_);
    temp.setRandom();
    assert(task_dynamics_(temp).rows() == dim_);
    //task_dynamics_ = task_dynamics;
    stiffness_.resize(dim_,dim_);
    stiffness_.setIdentity();
    stiffness_ *= DEF_STIFF;
    damping_.resize(dim_,dim_);
    damping_.setIdentity();
    damping_ *= DEF_DAMP;

    ds_origin_.resize(dim_);
    ds_origin_.setZero();

    b_tracking_ = false;
}

OpenloopDSController::OpenloopDSController(size_t dim, std::function<Vec (Vec)> task_dynamics, realtype stiffness, realtype damping) : OpenloopDSController(dim,task_dynamics)
{
    set_stiffness(stiffness);
    set_damping(stiffness);
}

OpenloopDSController::OpenloopDSController(size_t dim, std::function<Vec (Vec)> task_dynamics, const Mat &stiffness, const Mat &damping) : OpenloopDSController(dim,task_dynamics)
{
    set_stiffness(stiffness);
    set_damping(damping);
}

void OpenloopDSController::Restart(const Vec &start_pos)
{
    assert(start_pos.rows() == dim_);
    start_pos_ = start_pos;
    ref_pos_ = start_pos_ - ds_origin_;
}

Mat OpenloopDSController::GetTrajectory(realtype dt,realtype speed_threshold,realtype t_max){
    int n_max = floor(t_max/dt);
    Vec rpos = start_pos_-ds_origin_;
    Vec rvel(3);
    Mat traj(dim_,n_max);
    int n=0;
    while(n<n_max){
        traj.col(n)=rpos+ds_origin_;
        rvel = task_dynamics_(rpos);
        if(rvel.norm()<speed_threshold){
            traj.resize(dim_,n+1);
            break;
        }
        rpos += rvel*dt;
        n++;
    }
    return traj;
}

void OpenloopDSController::Update(const Vec& act_pos,const Vec& act_vel,realtype dt)
{
    ref_vel_ = task_dynamics_(ref_pos_);
    ref_pos_ += dt*ref_vel_;

    control_output_ = -stiffness_*(act_pos - ref_pos_ - ds_origin_);
    if(b_tracking_)
        control_output_ -= damping_*(act_vel - ref_vel_);
    else
        control_output_ -= damping_*act_vel;
}

void OpenloopDSController::set_stiffness(const Mat &stiffness)
{
    assert(stiffness.rows() == dim_ && stiffness.cols() == dim_);
    stiffness_ = stiffness;
}

void OpenloopDSController::set_damping(const Mat &damping)
{
    assert(damping.rows() == dim_ && damping.cols() == dim_);
    damping_ = damping;
}

void OpenloopDSController::set_stiffness(realtype stiffness)
{
    Mat mstiffness(dim_,dim_);
    mstiffness.setIdentity();
    mstiffness *= stiffness;
    set_stiffness(mstiffness);
}

void OpenloopDSController::set_damping(realtype damping)
{
    Mat mdamping(dim_,dim_);
    mdamping.setIdentity();
    mdamping *= damping;
    set_damping(mdamping);
}

void OpenloopDSController::set_ds_origin(const Vec &ds_origin)
{
    assert(ds_origin.rows() == dim_);
    ds_origin_ = ds_origin;
}
