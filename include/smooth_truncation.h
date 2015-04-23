#ifndef SMOOTH_TRUNCATION_H
#define SMOOTH_TRUNCATION_H

#include "passive_ds_typedefs.h"



// smooth step function returnning exactly 0.0 for val<lo and exactly 1.0 for val>1 and smooth transition in between
inline realtype smooth_rise(realtype val,realtype lo, realtype hi){
    assert(hi>=lo);
    if(val>=hi)
        return 1.0;
    else if(val<lo)
        return 0.0;
    else{
        realtype T = hi-lo;
        T *= 2;
        return 0.5+0.5*sin(2*M_PI*(val-lo)/T - M_PI*0.5);
    }
}


// smooth step function returnning exactly 1.0 for val<hi and exactly 0.0 for val>lo and smooth transition in between
inline realtype smooth_fall(realtype val,realtype hi, realtype lo){
    assert(lo>=hi);
    if(val>=lo)
        return 0.0;
    else if(val<hi)
        return 1.0;
    else{
        realtype T = lo-hi;
        T *= 2;
        return 0.5+0.5*sin(2*M_PI*(val-lo)/T - M_PI*0.5);
    }
}


// functor returning 0 for val<lo_zero_ and val>hi_zero_ , 1 for lo_one_<val<hi_one_ and again 0 for val>hi_zero_
class SmoothRiseFall{
private:
    realtype lo_zero_;
    realtype lo_one_;
    realtype hi_one_;
    realtype hi_zero_;

public:
    SmoothRiseFall(realtype l0,realtype l1,realtype h1,realtype h0):
        lo_zero_(l0), lo_one_(l1), hi_one_(h1), hi_zero_(h0)
    {}
    realtype operator ()(realtype val) {
        realtype h = smooth_rise(val,lo_zero_,lo_one_);
        h *= smooth_fall(val,hi_one_,hi_zero_);
        return h;
    }
};


// functor returning 0 for a smooth function that is zero for all x,y such that x<x_high and y>yhigh
//and zero for all x,y, such that x>x_low and y<_ylow
//% and 1 elsewhere. The transition is smooth and the output goes exactly to
//% zero at the specified boundaires. dx and dy contorl smoothness, large
//% value =>  smooth result
class SmoothRiseFall2d{
private:
    realtype xlo_;
    realtype xhi_;
    realtype ylo_;
    realtype yhi_;
    realtype dx_;
    realtype dy_;

public:
    SmoothRiseFall2d(realtype xlo,realtype xhi,realtype dx,realtype ylo,realtype yhi,realtype dy):
        xlo_(xlo), xhi_(xhi), dx_(dx), ylo_(ylo), yhi_(yhi), dy_(dy) {}
    realtype operator()(realtype x,realtype y){
        realtype h1 = smooth_rise(x,xlo_-dx_,xlo_);
        h1 *= smooth_fall(y,ylo_,ylo_+dy_);
        realtype h2 = smooth_fall(x,xhi_,xhi_+dx_);
        h2 *= smooth_rise(y,yhi_-dy_,yhi_);
        return 1.0-h1-h2;
    }

};


// functor returning exactly 0 for x >= xlo and y =<ylo and >0  elsewhere. Smoothness controlled by dx and dy
class SmoothRise2d{
private:
    realtype xlo_,dx_,ylo_,dy_;
public:
    SmoothRise2d(realtype xlo, realtype dx, realtype ylo, realtype dy):
        xlo_(xlo),dx_(dx),ylo_(ylo),dy_(dy) {}

    realtype operator() (realtype x,realtype y){
        realtype h1 = smooth_rise(x,xlo_,xlo_+dx_);
        h1 *= smooth_fall(y,ylo_,ylo_+dy_);
        return 1-h1;
    }

};











#endif // SMOOTH_TRUNCATION_H
