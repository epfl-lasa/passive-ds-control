#ifndef PASSIVE_DS_TYPEDEFS_H
#define PASSIVE_DS_TYPEDEFS_H

#include "eigen3/Eigen/Dense"

namespace PassiveDSControl
{
	//#ifdef USE_DOUBLE_PRECISION
	typedef double realtype;
	/*#else
	typedef float realtype;
	#endif*/
	typedef Eigen::Matrix<realtype,Eigen::Dynamic,Eigen::Dynamic> Mat;
	typedef Eigen::Matrix<realtype,Eigen::Dynamic,1> Vec;
	static realtype MINSPEED = 1e-6;
    static realtype FLOATEQUAL = 1e-6;
}

#endif // PASSIVE_DS_TYPEDEFS_H
