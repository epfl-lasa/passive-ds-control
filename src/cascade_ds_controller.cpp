#include "cascade_ds_controller.h"
#include <iostream>
CascadeDSController::CascadeDSController(size_t dim,  std::function<Vec(Vec)> task_dynamics,std::function<Vec(Vec)> process_filter) : dim_(dim)
{
    // make a test call to the supplied dynamics function and filter function
    Vec temp(dim_);
    temp.setRandom();
    assert(task_dynamics(temp).rows() == dim_);
    task_dynamics_ = task_dynamics;
    assert(process_filter(temp).rows() == dim_);
    process_filter_ = process_filter;
}

void CascadeDSController::ForwardIntegration(realtype driving_force, realtype dt, realtype speed_threshold)
{
    ref_pos_ = filt_pos_;
    //std::cout<<ref_pos_<<filt_pos_<<std::endl;
    realtype force_proj=0.0;
    while(force_proj<driving_force){
        ref_vel_ = task_dynamics_(ref_pos_);
        if(ref_vel_.norm()<speed_threshold)
           break;
        ref_pos_ += ref_vel_*dt;
        //std::cout<<act_pos_-ref_pos_<<std::endl;
        force_proj = ref_vel_.dot((ref_pos_-act_pos_))/ref_vel_.norm();
    }
}

void CascadeDSController::Reset(const Vec &act_pos)
{
    act_pos_ = act_pos;
    // burn in the filter
    for(int i=0;i<N_BURN;i++)
        process_filter_(act_pos);
    Update(act_pos_);
}

Mat CascadeDSController::IntegrateTrajectory(realtype dt, realtype speed_threshold, realtype t_max)
{
    int n_max = int(t_max/dt);
    Mat traj(dim_,n_max);
    ref_pos_ = filt_pos_;
    int n = 0;
    while(n<n_max){
        traj.col(n)=ref_pos_;
        ref_vel_ = task_dynamics_(ref_pos_);
        if(ref_vel_.norm()<speed_threshold){
            traj.resize(dim_,n+1);
            break;
        }
        ref_pos_ += ref_vel_*dt;
        n++;
    }
    return traj;
}

void CascadeDSController::Update(const Vec &act_pos){
    act_pos_ = act_pos;
    filt_pos_ = process_filter_(act_pos_);
}
