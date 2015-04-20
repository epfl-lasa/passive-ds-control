#ifndef LINEAR_VELOCITY_FIELDS_H
#define LINEAR_VELOCITY_FIELDS_H

#include "eigen3/Eigen/Dense"
#include "passive_ds_typedefs.h"



class LinearVelocityField{
private:
    Vec target_;
    Mat A_;
    realtype speedcap_;
public:
    LinearVelocityField(Vec target,Mat A,realtype speedcap){
        target_ = target;
        A_ = A;
        speedcap_=speedcap;
    }
    Vec ComputeVelocity(const Vec& pos){
        Vec vel = A_*(pos-target_);
        if(vel.norm()>speedcap_){
            vel /= vel.norm();
            vel *= speedcap_;
        }
        return vel;
    }

    Vec operator()(const Vec& pos){
        return this->ComputeVelocity(pos);
    }
    Vec target() const{
        return target_;
    }
    void set_target(const Vec &target){
        target_ = target;
    }
};

/**
  //How to use:
  int D = 3;
  Mat A = -0.4*Mat::Identity(D,D)
  Vec target(D);

  LinearVelocityField straightline_field(target,A,0.3);
  // to evaluate the system:
  Vec currpos; // set its value from somewhere
  Vec vel = straightline_field(currpos);

  another example:
    Mat A(D,D); A.setZero();
    A(0,1) = 1;
    A(1,0) = -1;
    A(2,2) = -3;
    LinearVelocityField circular_path(target,A,0.3);

  **/




#endif // LINEAR_VELOCITY_FIELDS_H

