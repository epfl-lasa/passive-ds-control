#include "passive_ds_controller.h"
#include <iostream>
using namespace std;

Vec compute_desired_vel(Vec pos){
    Vec target;
    target = pos*0.0;
    return -0.1*(pos-target);
}

int main(int argc, char *argv[])
{
    //PassiveDSController * my_passive_ds =new PassiveDSController;
    PassiveDSController * my_passive_ds;
    my_passive_ds = new PassiveDSController(6,&compute_desired_vel);
    my_passive_ds->set_damping_eigval(0,10);
    Vec temp(6);
    temp.setZero();
    temp(1)=1.0;
    Mat hejsan = my_passive_ds->ComputeDamping(temp);
    cout<<hejsan<<endl;
    return 0;
}

