#ifndef CASCADE_DS_CONTROLLER_H
#define CASCADE_DS_CONTROLLER_H

#include "passive_ds_typedefs.h"
#include <functional>


// TODO: Add computation of control effort using stiffness and damping internally



class CascadeDSController
{
    static const int N_BURN=1000;
    // pointer to dynamics function
    std::function<Vec(Vec)> task_dynamics_;
    //Vec (*task_dynamics_)(Vec);
    // pointer to filter function
    //Vec (*process_filter_)(Vec);
    std::function<Vec(Vec)> process_filter_;
    // impedance parameters
    Mat stiffness_;
    Mat damping_;

    size_t dim_;

    Vec act_pos_;
    Vec filt_pos_;
    Vec ref_pos_;
    Vec ref_vel_;

    Vec ds_origin_;



public:
    CascadeDSController(size_t dim, std::function<Vec(Vec)> task_dynamics,std::function<Vec(Vec)> process_filter);
    void ForwardIntegration(realtype driving_force,const Mat& stiffness, realtype dt, realtype speed_threshold);
    void Reset(const Vec& act_pos, int n_burn=N_BURN);
    Mat IntegrateTrajectory(realtype dt,realtype speed_threshold,realtype t_max);

    void Update(const Vec &act_pos);
    Vec ref_pos(){return ref_pos_+ds_origin_;}
    size_t dim(){return dim_;}
    void set_ds_origin(const Vec& ds_origin);
};

#endif // CASCADE_DS_CONTROLLER_H
