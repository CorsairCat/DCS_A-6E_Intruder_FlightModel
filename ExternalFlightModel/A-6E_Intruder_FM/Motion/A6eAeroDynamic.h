#ifndef _A6EAERO_H_
#define _A6EAERO_H_

#include "../A6E_FM_Utility.h"
#include "A6eConstantData.h"

// use for calculate aerodynamic and aero control
// Important: this part not include the gravity, gravity is controlled by game core;
// control and force are supplied here
// atmosphere data will also transfer here
class A6eAeroDynamic
{
private:

public:
    double AirDensity = 1.225;
    double AirPressure = 101325;
    double SpeedOfSound = 340.3;
    Vec3 WindAround = {0,0,0};
    double AlieronPos = 0;
    double ElevatorPos = 0;
    double RudderPos = 0;
    double AngleOfAttack = 0;
    double AngleOfSlide = 0;

protected:
    double ISAAirDensity = 1.225; // in Kg/m^3
    double ISAAirPressure = 101325; //in Pa
    double ISASpeedOfSound = 340.3; // in m/s
};

#endif