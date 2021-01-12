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
    double moveMaxRate = 0.2;
    double LastStickRollInput = 0;
    double LastStickYawInput = 0;
    double LastStickPitchInput = 0;
    double moveRateManual = 0.01; 
public:
    int inputRollKeyboard = 0;
    int inputPitchKeyboard = 0;
    int inputYawKeyboard = 0;
    // These Three Functions are for data export to AeroDynamic Control
    // Including Autopilot Ability
    double exportPitch()
    {
        return desiredPitch;
    }

    double exportRoll()
    {
        return desiredRoll;
    }

    double exportYaw()
    {   
        return desiredYaw;
    }

    void inputRoll(double inputValue)
    {
        desiredRoll = inputValue;
        LastStickRollInput = inputValue;
    }

    void inputPitch(double inputValue)
    {
        desiredPitch = inputValue;
        LastStickPitchInput = inputValue;
    }

    void inputYaw(double inputValue)
    {
        desiredYaw = inputValue;
        LastStickYawInput = inputValue;
    }

    // sim 时刷新的主函数 可以把部分函数变为privite
    void updateDuringSimulation()
    {
        updatePitchInput();
        updateRollInput();
        updateYawInput();
    }

    void updatePitchInput()
    {
        if (inputPitchKeyboard > 0)
        {
            desiredPitch += moveRateManual;
        }
        else if (inputPitchKeyboard < 0)
        {
            desiredPitch -= moveRateManual;
        }
        if (desiredPitch > 1)
        {
            desiredPitch = 1;
        }
        else if (desiredPitch < -1)
        {
            desiredPitch = -1;
        }
    }

    void updateRollInput()
    {
        if (inputRollKeyboard > 0)
        {
            desiredRoll += moveRateManual;
        }
        else if (inputRollKeyboard < 0)
        {
            desiredRoll -= moveRateManual;
        }
        else
        {
            //auto go back to netural
            if (fabs(desiredRoll - LastStickRollInput) < moveMaxRate)
            {
                desiredRoll = LastStickRollInput;
            }
            else
            {
                if (desiredRoll < LastStickRollInput)
                {
                    /* code */
                    desiredRoll += moveMaxRate;
                }
                else
                {
                    desiredRoll -= moveMaxRate;
                }
            }
        }
        if (desiredRoll > 1)
        {
            desiredRoll = 1;
        }
        else if (desiredRoll < -1)
        {
            desiredRoll = -1;
        }
    }

    void updateYawInput()
    {
        if (inputYawKeyboard > 0)
        {
            desiredYaw += moveRateManual;
        }
        else if (inputYawKeyboard < 0)
        {
            desiredYaw -= moveRateManual;
        }
        else
        {
            //auto go back to netural
            if (fabs(desiredYaw - LastStickYawInput) < moveMaxRate)
            {
                desiredYaw = LastStickYawInput;
            }
            else
            {
                if (desiredYaw < LastStickYawInput)
                {
                    /* code */
                    desiredYaw += moveMaxRate;
                }
                else
                {
                    desiredYaw -= moveMaxRate;
                }
            }
        }
        if (desiredYaw > 1)
        {
            desiredYaw = 1;
        }
        else if (desiredYaw < -1)
        {
            desiredYaw = -1;
        }
    }
};

#endif