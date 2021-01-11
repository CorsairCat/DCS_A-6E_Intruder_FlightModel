#ifndef _A6ECONTROL_H_
#define _A6ECONTROL_H_

#include "../A6E_FM_Utility.h"

// use for calculate aerodynamic and aero control
// Important: this part not include the gravity, gravity is controlled by game core;
//
class A6eFlightControl
{
private:
    double desiredPitch = 0;
    double desiredRoll = 0;
    double desiredYaw = 0;
public:
    // origin input of stick and peddel
    double inputRoll = 0;
    double inputPitch = 0;
    double inputYaw = 0;
    // These Three Functions are for data export to AeroDynamic Control
    // Including Autopilot Ability
    double exportPitch()
    {

    }

    double exportRoll()
    {

    }

    double exportYaw()
    {

    }
};

#endif