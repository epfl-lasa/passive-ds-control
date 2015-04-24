#ifndef OPENLOOP_DS_CONTROLLER_H
#define OPENLOOP_DS_CONTROLLER_H
#include "passive_ds_typedefs.h"

static const float DEF_STIFF = 10;
static const float DEF_DAMP = 2*sqrt(DEF_STIFF);


class OpenloopDSController
{

    // pointer to dynamics function
    std::function<Vec(Vec)> task_dynamics_;

    // impedance parameters
    Mat stiffness_;
    Mat damping_;

    size_t dim_;

    Vec ref_pos_;
    Vec start_pos_;
    Vec ref_vel_;
    Vec ds_origin_;

    Vec control_output_;
    bool b_tracking_;

public:
    OpenloopDSController(size_t dim, std::function<Vec(Vec)> task_dynamics);
    OpenloopDSController(size_t dim, std::function<Vec(Vec)> task_dynamics,realtype stiffness, realtype damping);
    OpenloopDSController(size_t dim, std::function<Vec(Vec)> task_dynamics,const Mat& stiffness, const Mat& damping);

    void Restart(const Vec& start_pos);
    void Update(const Vec &act_pos, const Vec &act_vel, realtype dt);


    void set_stiffness(const Mat& stiffness);
    void set_damping(const Mat& damping);
    Vec control_output() const;
    Vec ref_vel() const;
    void set_stiffness(realtype stiffness);
    void set_damping(realtype damping);
    void set_ds_origin(const Vec& ds_origin);
    void set_tracking(bool tr = true){b_tracking_ = tr;}
    Vec ref_pos() const;
    Vec start_pos() const;
    Mat GetTrajectory(realtype dt, realtype speed_threshold, realtype t_max);
};

#endif // OPENLOOP_DS_CONTROLLER_H
