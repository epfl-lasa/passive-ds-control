#include "passive_ds_controller.h"

#include <iostream>
//PassiveDSController::PassiveDSController(int dim, Vec (*velocity_field)(Vec))
//{
//    dim_=dim;
//    damping_.resize(dim,dim);
//    damping_eigval_.resize(dim,dim);
//    basis_.resize(dim,dim);
//    basis_.setRandom();
//    velocity_field_ = velocity_field;
//}

PassiveDSController::PassiveDSController(int dim)
{
    dim_=dim;
    damping_.resize(dim,dim);
    damping_eigval_.resize(dim,dim);
    basis_.resize(dim,dim);
    basis_.setRandom();
}

void PassiveDSController::ComputeOrthogonalBasis(Vec dir){
    assert(!(dir.rows()-basis_.rows()));
    basis_.col(0)=dir;
    for(int i=1;i<dim_;i++){
        for(int j=0;j<i;j++)
            basis_.col(i) -= basis_.col(j).dot(basis_.col(i))*basis_.col(j);
        basis_.col(i).normalize();
    }
}

void PassiveDSController::set_damping_eigval(realtype damping_eigval0,realtype damping_eigval1)
{
    damping_eigval_.setZero();
    damping_eigval_(0,0)=damping_eigval0;
    for(int i=1;i<damping_eigval_.rows();i++)
        damping_eigval_(i,i)=damping_eigval1;
}

Mat PassiveDSController::ComputeDamping(Vec vel)
{
    realtype speed = vel.norm();
    speed = (speed > MINSPEED)? speed:MINSPEED;
    Vec dir = vel/speed;
    ComputeOrthogonalBasis(dir);
    damping_ = basis_*damping_eigval_*basis_.transpose();
    return damping_;
}
