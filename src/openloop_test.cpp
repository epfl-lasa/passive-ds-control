

#include "openloop_ds_controller.h"
#include "linear_velocity_fields.h"

int main(int argc, char *argv[])
{
    int D = 3;
    Mat A(D,D); A.setZero();
    A(0,1) = 1;
    A(1,0) = -1;
    A(2,2) = -3;
    Vec target(D);
    target.setZero();
    LinearVelocityField circular_path(target,A,0.3);
    auto dyn = std::bind(&LinearVelocityField::ComputeVelocity,circular_path,std::placeholders::_1);
    //OpenloopDSController(D,dyn,10,3);
    return 0;
}

