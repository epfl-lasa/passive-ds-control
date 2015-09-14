
#include "cascade_ds_controller.h"
#include "linear_velocity_fields.h"
#include "exponentialsmoother.h"
#include <functional>
//#include "timer.h"


int main(int argc, char *argv[])
{
    int D = 3;
    ExponentialSmoother<Vec> filter(0.01);
    LinearVelocityField straight_line(Vec::Zero(D),0.4*Mat::Identity(D,D),0.1);
    //std::function<Vec(Vec)> dyn_func = straight_line.ComputeVelocity()
    std::function<Vec(Vec)> f = std::bind(&LinearVelocityField::ComputeVelocity,&straight_line,std::placeholders::_1);
    //f(Vec::Zero(3));
    Vec temp(D);
    straight_line.ComputeVelocity(temp);
    std::cout<<f(temp)<<std::endl;
    CascadeDSController casc_ctrl(D,std::bind(&LinearVelocityField::ComputeVelocity,&straight_line,std::placeholders::_1),std::bind(&ExponentialSmoother<Vec>::filter,&filter,std::placeholders::_1));
    casc_ctrl.Reset(Vec::Random(3));
    //Timer t;
    //casc_ctrl.ForwardIntegration(0.02,0.002,0.01);
    //std::cout<<t.elapsed()<<std::endl;
    std::cout<<"computed ref pos: "<<casc_ctrl.ref_pos()<<std::endl;
    return 0;
}

