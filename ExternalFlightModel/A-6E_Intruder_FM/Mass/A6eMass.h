#ifndef _A6EENGINE_H_
#define _A6EENGINE_H_

#include "../A6E_FM_Utility.h"

// use for calculate thrust and get data from fuel.
class A6eMassSystem
{
private:
    /* data */
    double SelfMass = 28300 * LbsToKg; //empty Mass 28300 pounds
    // position 15ft -> 145 px
    // zero point of model is at 180 px -> 19.286 ft -> 5.8784 m
    // mac position around 170 px from nose
    // estimate CG is around 145 px, where the center of fuel tank and pylon
    // CG pos: 15ft from nose
    // 0.7883 m from zero point
    Vec3 CenterOfGravityEmpty = {5.8784 - 4.572, -0.7883, 0};
public:
    
};

#endif