
#include "passive_ds_controller.h"

#include <iostream>


void orthonormalize(Mat & basis){
    assert(basis.rows() == basis.cols());
    uint dim = basis.rows();
    basis.col(0).normalize();
    for(uint i=1;i<dim;i++){
        for(uint j=0;j<i;j++)
            basis.col(i) -= basis.col(j).dot(basis.col(i))*basis.col(j);
        basis.col(i).normalize();
    }
}


void assert_orthonormal(Mat & basis){
    uint dim = basis.cols();
    for (int i = 0; i < dim; ++i) {
        assert(fabs(basis.col(i).norm()-1.0) < FLOATEQUAL);
        for (int j = 0; j < i; ++j) {
            assert(fabs(basis.col(i).dot(basis.col(j)))<FLOATEQUAL);
        }

    }
}


Mat DSController::damping_eigval() const
{
    return damping_eigval_;
}

Mat DSController::damping_matrix()
{
    return damping_;
}


Vec DSController::control_output() const
{
    return control_output_;
}


DSController::DSController(int dim,realtype damping_eigval0,realtype damping_eigval1)
{
    damping_.resize(dim,dim);
    damping_eigval_.resize(dim,dim);
    set_damping_eigval(damping_eigval0,damping_eigval1);
    basis_.resize(dim,dim);
    basis_.setRandom();
    orthonormalize(basis_);
    assert_orthonormal(basis_);

}


void DSController::ComputeOrthonormalBasis(const Vec &dir){
    assert(dir.rows()==basis_.rows());
    basis_.col(0)=dir;
    orthonormalize(basis_);
}

#include <iostream>

void DSController::Update(const Vec &vel, const Vec &ref_vel)
{
    // compute damping
    ComputeDamping(ref_vel);
    // dissipate
    control_output_ = -damping_*vel;
    // control_output_ = -damping_eigval_(0)*vel;
/*

    float normRefVel = sqrt(ref_vel(0)*ref_vel(0)+ref_vel(1)*ref_vel(1)+ref_vel(2)*ref_vel(2));
    float scalarProjection = (vel(0)*ref_vel(0)+vel(1)*ref_vel(1)+vel(2)*ref_vel(2))/normRefVel;
    Vec projectedVel = scalarProjection*ref_vel;
    Vec orthogonalVel = vel-projectedVel;
    control_output_ = -damping_eigval_(0)*vel-damping_eigval_(1)*orthogonalVel;*/

    // compute control
    control_output_ += damping_eigval_(0)*ref_vel;


    // Vec error;
    // error.resize(3,1);
    // error = vel-ref_vel;

    // Vec orthogonalError;
    // orthogonalError.resize(3,1);
    // orthogonalError = error;


    // Vec parallelError;
    // parallelError.resize(3,1);
    // parallelError.setZero();


    // if(ref_vel.norm()> MINSPEED)
    // {
    //     parallelError = (error.dot(ref_vel)/pow(ref_vel.norm(),2.0f))*ref_vel;
    // }

    // orthogonalError -= parallelError;

    // control_output_ = -damping_eigval_(0,0)*parallelError-damping_eigval_(1,1)*orthogonalError;

    // ROS_INFO_STREAM_THROTTLE(1.0,"lambda_0: " << damping_eigval_(0,0) << " lambda_1: " << damping_eigval_(1,1) );

    // ROS_INFO_STREAM_THROTTLE(1.0,"p: " << parallelError(0) << " " << parallelError(1) << " " << parallelError(2) <<  " o:" << orthogonalError(0) << " " << orthogonalError(1) << " " << orthogonalError(2)); 
    // // ROS_INFO_STREAM_THROTTLE(thrott_time,"pos_cmd_: " << pos_cmd_(0) << " " <<  pos_cmd_(1) << " " <<  pos_cmd_(2) << " " << pos_cmd_(3) << " " <<  pos_cmd_(4) << " " <<  pos_cmd_(5) << " " << pos_cmd_(6));

    // ROS_INFO_STREAM_THROTTLE(1.0,"control_output_ : " << control_output_(0) << " " << control_output_(1) << " " << control_output_(2) );

    
//     e_o = x_r - x_d;
//     e_p = 0;

// if(xd > big_enough)
//     e_p = (e^T * x_d ) * x_d  / (x_d^t * x_d);
// end
//     e_o -= e_p;

//     F = - l0 * e_p - l1 * e_o;

}

void DSController::set_damping_eigval(realtype damping_eigval0,realtype damping_eigval1)
{
    damping_eigval_.setZero();
    damping_eigval_(0,0)=damping_eigval0;
    for(int i=1;i<damping_eigval_.rows();i++)
        damping_eigval_(i,i)=damping_eigval1;
}

Mat DSController::ComputeDamping(const Vec &ref_vel)
{
    // only proceed of we have a minimum velocity norm!
    if(ref_vel.norm() > MINSPEED)
        ComputeOrthonormalBasis(ref_vel);
    // otherwise just use the last computed basis
    damping_ = basis_*damping_eigval_*basis_.transpose();
    return damping_;
}


void DSController::set_damping_eigval(const Mat& damping_eigval)
{
    assert(damping_eigval.rows() == damping_eigval_.rows());
    if(damping_eigval.cols() != damping_eigval_.cols()){
        assert(damping_eigval.cols()==1);
        damping_eigval_.setZero();
        for (int i = 0; i < damping_eigval.rows(); i++) {
            damping_eigval_(i,i) = damping_eigval(i);
        }
    }else{
         damping_eigval_ = damping_eigval;
    }
}




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
