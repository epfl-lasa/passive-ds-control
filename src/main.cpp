#include "passive_ds_controller.h"
#include "linear_velocity_fields.h"
#include "smooth_truncation.h"
#include <iostream>
#include <fstream>
#include "Matrix.h"
using namespace std;


void test_smooth_truncation(){
    int nX = 100;
    int nY = 100;

    SmoothRiseFall st(-0.2,0.0,0.2,1.4);
    SmoothRiseFall2d st2(0,0.0,0.0,0.3,1.4,0.2);
    st2(0.0,-1.0);

    MathLib::Matrix X;
    std::cout<<"started loading \n";
    X.Load("X.txt");

    MathLib::Matrix Y;
    Y.Load("Y.txt");
    std::cout<<"finished loading \n";

    MathLib::Matrix Z1;
    Z1.Resize(X.RowSize(),X.ColumnSize());

    MathLib::Matrix Z2;
    Z2.Resize(X.RowSize(),X.ColumnSize());
    //Y.Print();
    for (int i = 0; i < X.RowSize(); ++i) {
        for (int j = 0; j < Y.ColumnSize(); ++j) {
            Z1(i,j) = st(X.At(i,j));
            Z2(i,j) = st2(X.At(i,j),Y.At(i,j));
        }
    }

    Z1.Save("Z1.txt");
    Z2.Save("Z2.txt");


//    ofstream fy("Y.txt");
//    for(std::vector<realtype>::iterator it=Y.begin(); it != Y.end(); ++it){
//        fy<<*it<<std::endl;
//    }
//    fy.close();


}



int main(int argc, char *argv[])
{
    test_smooth_truncation();
    int D = 7;
    Vec target(D);
    //target(2)=1;
    Mat A(D,D); A.setZero();
    A(0,1) = 1;
    A(1,0) = -1;
    A(2,2) = -2;
    LinearVelocityField circular_path(target,A,10);
    LinearVelocityField straight_line(target,0.4*Mat::Identity(D,D),0.1);
    //PassiveDSController * my_passive_ds =new PassiveDSController;
    DSController * my_passive_ds;
    my_passive_ds = new DSController(D,0,10);
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

