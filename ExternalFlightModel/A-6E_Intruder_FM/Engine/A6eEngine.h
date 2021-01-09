#ifndef _A6EENGINE_H_
#define _A6EENGINE_H_

#include "../A6E_FM_Utility.h"

// use for calculate thrust and get data from fuel.
class A6eEngineSystem
{
private:
    /* data */

public:
    double throttlePosition = 0;
    double throttleKeyBoard = 0;
    bool updateThrottlePosition()
    {
        if (throttlePosition <= 1 && throttlePosition >= 0)
        {
            if (throttleKeyBoard > 0.5)
            {
                throttlePosition += 0.005;
                if (throttlePosition > 1)
                {
                    throttlePosition = 1;
                }
                return true;
            }
            else if (throttleKeyBoard < -0.5)
            {
                throttlePosition -= 0.005;
                if (throttlePosition < 0)
                {
                    throttlePosition = 0;
                }
                return true;
            }
            else
            {
                // do nothing
                return false;
            }
        }
        else if (throttlePosition > 1)
        {
            /* code */
            throttlePosition = 1;
            return true;
        }
        else
        {
            throttlePosition = 0;
            return true;
        }
        // return defines if need to update EFM THRO parameter
    }
};

#endif