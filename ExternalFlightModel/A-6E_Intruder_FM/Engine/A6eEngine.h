#ifndef _A6EENGINE_H_
#define _A6EENGINE_H_

#include "../A6E_FM_Utility.h"
#include "J52Constant.h"

// use for calculate thrust and get data from fuel.
class A6eEngineSystem
{
private:
    /* data */
    double LowComprossorRPM;
    double CoreRPM;
    double FuelFlow;
    double CoreTemperature;
    double ExhaustTemperature;
    double targetFuelFlow;
    int throttleIdleState = 0;
    int isPreStartCrank = 0;
    double rpmIncreaseStep = 0;
public:
    double throttlePosition = 0;
    double throttleKeyBoard = 0;
    int engineDesiredState = 0; // 0 = stop; 1 = start; 用于lua兼容性; 后期会移除
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

    void updateIdleState(double throttleFactor)
    {
        if (throttleFactor > 0.1) 
        {
            throttleIdleState = 1;
        }
        else
        {
            throttleIdleState = 0;
        }
        
    }

    void updateFuelFlow()
    // call in every simulation
    {
        // if starting/ running/
        // idle state
        if (throttleIdleState == 0)
        {
            targetFuelFlow = 0;
            // when not idle and target is engine start
            if (engineDesiredState == 1)
            {
                isPreStartCrank += 1;
            }
            else
            {
                isPreStartCrank = 0;
            }   
        }
        else
        {
            if (engineDesiredState == 0) //if fuel master is off at this time state should be 0:stop
            // means fuel has been cut off
            {
                targetFuelFlow = 0;
            }
            else
            {
                /* code */
                targetFuelFlow = throttlePosition + 0.1;
            } 
            isPreStartCrank = 0;
        }
        //
    }

    double getCurrentCoreRPM()
    {
        //J52Engine::ConvertFuelFlowToN2[0];
        if (FuelFlow >= 0.1)
        {
            int temp = (int)((FuelFlow - 0.1) * 50);
            double temp_remainder = (FuelFlow - 0.1) * 50 - temp;
            CoreRPM = (J52Engine::ConvertFuelFlowToN2[temp + 1] - J52Engine::ConvertFuelFlowToN2[temp]) * temp_remainder + J52Engine::ConvertFuelFlowToN2[temp + 1];
            // engine is  started successfully, reset signal
            rpmIncreaseStep = 0;
        }
        else if (FuelFlow > 0)
        {
            // in starter condition
            if (engineDesiredState == 1)
            {
                if (CoreRPM > 1187)
                {
                    if (rpmIncreaseStep == 0)
                    {
                        rpmIncreaseStep = J52Engine::ConvertFuelFlowToN2[0] - CoreRPM;
                        CoreRPM = rpmIncreaseStep * FuelFlow * 10;
                    }
                    else
                    {
                        CoreRPM = rpmIncreaseStep * FuelFlow * 10;
                    }
                }
                else
                {
                    // start failed
                }
                
            }
            else
            {
                if (rpmIncreaseStep == 0)
                {
                    rpmIncreaseStep = CoreRPM;
                    CoreRPM = rpmIncreaseStep * FuelFlow * 10;
                }
                else
                {
                    CoreRPM = rpmIncreaseStep * FuelFlow * 10;
                }
            }
        }
        else if (isPreStartCrank > 0)
        {
            /* start crank */
            // imagine start dt = 0.006, needs 5 seconds to reach 1186 rpm, max 1800 rpm
            if (CoreRPM < 1800)
            {
                CoreRPM += 1186 / 840;
            }
            else if (CoreRPM < 10000)
            {
                CoreRPM = 1800; // Before Crank Stop
            }
            else
            {
                isPreStartCrank = 0; //more than 60s not started, stop crank
            }
            
        }
        else
        {
            // shut down engine from crank or engine stop states
            if (CoreRPM > 0)
            {
                //do nothing
                CoreRPM -= 1800/3200;
            }
            else
            {
                /* code */
                CoreRPM = 0;
            }
            
        }

        return CoreRPM;
    }

    void initialEngineState(int BirthState) //0: cold; 1: hot
    {
        if (BirthState == 1)
        {
            throttleIdleState = 1;
        }
        else if (BirthState == 0)
        {
            throttleIdleState = 0;
        }
    }
};

#endif