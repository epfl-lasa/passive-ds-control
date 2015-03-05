#include "passive_ds_controller.h"
#include "linear_velocity_fields.h"

#include <iostream>
using namespace std;


int main(int argc, char *argv[])
{
    int D = 7;
    Vec target(D);
    //target(2)=1;
    Mat A(D,D); A.setZero();
    A(0,1) = 1;
    A(1,0) = -1;
    A(2,2) = -2;
    linear_velocity_field circular_path(target,A,10);
    linear_velocity_field straight_line(target,0.4*Mat::Identity(D,D),0.1);
    //PassiveDSController * my_passive_ds =new PassiveDSController;
    PassiveDSController * my_passive_ds;
    my_passive_ds = new PassiveDSController(D,0,10);
    //my_passive_ds->set_damping_eigval(0,10);
    Vec temp(D);
    temp.setZero();
    temp(0)=1.0;
    temp(2) = 1.0;
    cout<<circular_path(temp)<<endl;
    Mat hejsan = my_passive_ds->ComputeDamping(circular_path(temp));
    cout<<hejsan<<endl;
    temp *= 2;
    cout<<circular_path(temp)<<endl;
    hejsan = my_passive_ds->ComputeDamping(circular_path(temp));
    cout<<hejsan<<endl;
    return 0;
}

