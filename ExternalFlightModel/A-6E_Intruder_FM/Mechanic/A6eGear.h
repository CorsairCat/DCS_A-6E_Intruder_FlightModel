#ifndef _A6EGEAR_H_
#define _A6EGEAR_H_

#include "../A6E_FM_Utility.h"

class A6eGearWheel
{
private:
    double yawPositionTarget = 0;
    double movingStep = 0.003;
public:
    double GearStatus;
    int Steering = 0;
    double BrakeStatusMultiPlier = 0;
    int GearControlOveride = 0; // use for carrier launch operation
    double weightOnWheel;
    double currentYaw = 0;

    void updateYawPosition(double inputRudder, double airspeed)
    {
        if (GearStatus >= 0.8 && Steering == 1 && airspeed < 40)
        {   
            yawPositionTarget = (- inputRudder) * (40 - airspeed)/40;
        }
        else
        {
            yawPositionTarget = 0;
        }
        
    }

    void updateCurrentYaw()
    {
        if (Steering == 1 && weightOnWheel > 0) // activate hydrolic
        {
            if (fabs(yawPositionTarget - currentYaw) < movingStep)
            {
                currentYaw = yawPositionTarget;
            }
            else
            {
                if (yawPositionTarget < currentYaw)
                {
                    currentYaw -= movingStep;
                }
                else
                {
                    currentYaw += movingStep;
                } 
            }
        }
        else if (yawPositionTarget == 0)
        {
            /* code */
            if (fabs(yawPositionTarget - currentYaw) < movingStep)
            {
                currentYaw = yawPositionTarget;
            }
            else
            {
                if (yawPositionTarget < currentYaw)
                {
                    currentYaw -= movingStep;
                }
                else
                {
                    currentYaw += movingStep;
                } 
            }
        }
        
    }
};

class A6eGearSystem
{
private:
    /* data */

public:
    A6eGearWheel nose;
    A6eGearWheel left;
    A6eGearWheel right;
    int CarrierPos = 0;

    void initial(int birth) // 0 for ground and 1 for air
    {   
        if (birth == 0)
        {
            nose.GearStatus = 1;
            left.GearStatus = 1;
            right.GearStatus = 1;
        }
        else
        {
            nose.GearStatus = 0;
            left.GearStatus = 0;
            right.GearStatus = 0;
        }
	    CarrierPos = 0;
    }

    void setWheelBrakes(double value)
    {
        nose.BrakeStatusMultiPlier = value;
        left.BrakeStatusMultiPlier = value;
        right.BrakeStatusMultiPlier = value;
    }
};

#endif