#include "passive_ds_controller.h"
#include "linear_velocity_fields.h"
#include "smooth_truncation.h"
#include <iostream>
#include <fstream>
#include "Matrix.h"
using namespace std;

Eigen::MatrixXd LoadEigenFromFile(string fn){
    ifstream file(fn);
    Eigen::MatrixXd ret;
    if(!file.is_open()){
        std::cout<<"invalid filename"<<std::endl;
        return ret;
    }
    string line;
    string item;
    vector<vector<string>> sdata;
    while(!file.eof()){
        getline(file,line);
        stringstream lstream(line);
        vector<string> rsdata;
        while(getline(lstream,item,' '))
            rsdata.push_back(item);
        if(rsdata.size()>0)
            sdata.push_back(rsdata);
    }
    ret.resize(sdata.size(),sdata[0].size());
    for (int i = 0; i < ret.rows(); ++i) {
        for (int j = 0; j < ret.cols(); ++j) {
            ret(i,j) = atof(sdata[i][j].c_str());
        }
    }
    return ret;
}


bool WriteEigenToFile(Mat mat,string fn){
    ofstream file(fn);
    for (int i = 0; i < mat.rows(); ++i) {
        for (int j = 0; j < mat.cols(); ++j) {
            file << mat(i,j)<<' ';
        }
        file<<"\n";
    }
    file.close();
}


void test_passive_ds(){
    Eigen::MatrixXd data;
    data = LoadEigenFromFile("/home/klas/temp/test_smooth_trunc/testPassiveDS.txt");
    //std::cout<<data<<std::endl;

    PassiveDSController mps(2,5,20,10,0.3,0.01);
    Vec storage;
    storage.resize(data.rows(),1);
    for(int i=0;i<data.rows()-1;i++){
        double dt = data(i+1,0)-data(i,0);
        Vec vel = data.block<1,2>(i,3).transpose();
        Vec ref_vel_c = data.block<1,2>(i,6);
        Vec ref_vel_nc = data.block<1,2>(i,8);
        mps.UpdatePassive(vel,ref_vel_c,ref_vel_nc,dt);
        storage(i) = mps.s();
    }

    WriteEigenToFile(storage,"/home/klas/temp/test_smooth_trunc/resultS.txt");
}


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
    test_passive_ds();
    //test_smooth_truncation();
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

