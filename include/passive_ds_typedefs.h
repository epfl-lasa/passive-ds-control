#ifndef PASSIVE_DS_TYPEDEFS_H
#define PASSIVE_DS_TYPEDEFS_H

#include "eigen3/Eigen/Dense"
//#ifdef USE_DOUBLE_PRECISION
typedef double realtype;
/*#else
typedef float realtype;
#endif*/
typedef Eigen::Matrix<realtype,Eigen::Dynamic,Eigen::Dynamic> Mat;
typedef Eigen::Matrix<realtype,Eigen::Dynamic,1> Vec;


#endif // PASSIVE_DS_TYPEDEFS_H
