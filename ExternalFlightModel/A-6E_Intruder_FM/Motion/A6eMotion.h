#ifndef _A6EMOTION_H_
#define _A6EMOTION_H_

#include "../A6E_FM_Utility.h"

// use for calculate all force acting and moment together
// Important: this part not include the gravity, gravity is controlled by game core;
//
class A6eMotionCalculation
{
private:
    Vec3 totalForce;
    Vec3 totalMoment;
    Vec3 ForceActingPoint; // should be the position of center of gravity
public:
    double dt; // frametime for calculation
    // when simulation start use this function to clear all force and moment from last time frame
    bool clearForceLastFrame()
    {
        totalForce = 0;
        totalMoment = 0;
    }
    
};

#endif