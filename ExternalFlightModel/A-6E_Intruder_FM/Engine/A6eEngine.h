#ifndef _A6EENGINE_H_
#define _A6EENGINE_H_

#include "../A6E_FM_Utility.h"
#include "J52Constant.h"

// use for calculate thrust and get data from fuel.
class A6eEngineSystem
{
private:
    /* data */
    double DesiredFuelFlow = 0;
    double targetFuelFlow = 0;
    double staticThrust = 0;
    double targetThrust = 0;
    double ErrorLastTime = 0;
    int throttleIdleState = 0;
    int isPreStartCrank = 0;
    double rpmIncreaseStep = 0;
    double kp = 0.014/44000;
    double ki = 0.003/44000;
    double kd = 0.006/44000;
    double kc = 80000;
    double antiI = 0;// 积分饱和记录器
    double I_counter = 0;
    double I_counter_lasttime = 0; // 防止积分饱和的记录上一次积分值
    double FuelFlowStep = 0.1; // 变化率为 0.1 kg/sec
    double getAirFlowMassRate()
    // in kg/sec
    {
        double massFlow = (CoreRPM - 6600) / 2 / 56 + 14;
        return massFlow;
    }
public:
    double LowComprossorRPM;
    double CoreRPM;
    double FuelFlow;
    double CoreTemperature;
    double ExhaustTemperature;
    double netThrust;// in Newton Unit
    double dt;
    double throttlePosition = 0;
    double throttlePositionLastTime = 0;
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

    void fuelFlowController()
    // call in every simulation
    {
        // if starting/ running/
        // idle state
        if (throttleIdleState == 0)
        {
            DesiredFuelFlow = 0;
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
                DesiredFuelFlow = 0;
            }
            else
            {
                /* code */
                // for throttle is direct link to thrust, temply dont want to use rpm as input, just use the static thrust;
                //targetFuelFlow = throttlePosition + 0.1;
                // PID for changing data
                // this part needs a change to control engine fuel decrease
                // 防止积分饱和
                targetThrust = (J52Engine::maxStaticThrustMIL - 2000) * throttlePosition + 2000;
                double tempdeltaT = targetThrust - staticThrust;
                if (throttlePositionLastTime != throttlePosition)
                {
                    //I_counter = 0;
                    ErrorLastTime = tempdeltaT;
                    throttlePositionLastTime = throttlePosition;
                    //antiI = 0;
                }
                I_counter_lasttime = I_counter;
                I_counter += tempdeltaT - antiI;
                double d_temp = (tempdeltaT - ErrorLastTime) / dt;
                ErrorLastTime = tempdeltaT;
                DesiredFuelFlow = ki * I_counter + kp * tempdeltaT + kd * d_temp;
            } 
            isPreStartCrank = 0;
        }
        //
    }

    void updateFuelFlow()
    {
        // 输出限幅部分
        double temp_output = DesiredFuelFlow; 
        if (DesiredFuelFlow <= 0.1 && CoreRPM > 6500 && throttleIdleState == 1) 
        {
            DesiredFuelFlow = 0.1;
        }
        else if (DesiredFuelFlow > 1.04)
        {
            DesiredFuelFlow = 1.04;
        }
        else if (DesiredFuelFlow < 0)
        {
            DesiredFuelFlow = 0;
        }

        // 加一个积分饱和的处理器
        // use anti AW
        // double temp_Ti_output = ;
        antiI = (temp_output - DesiredFuelFlow) * kc;
        
        
        if (fabs(DesiredFuelFlow - FuelFlow) < (FuelFlowStep * dt))
        {
            // do nothing
            FuelFlow = DesiredFuelFlow;
        }
        else
        {
            if (DesiredFuelFlow > FuelFlow)
            {
                FuelFlow += FuelFlowStep * dt;
            }
            else
            {
                FuelFlow -= FuelFlowStep * dt;
            }  
        }
    }

    // set a rpm function to control the rpm decrease/increase step here
    // mainly for decrease
    double getCurrentCoreRPM()
    {
        //J52Engine::ConvertFuelFlowToN2[0];
        if (FuelFlow >= 0.1)
        {
            int temp = (int)((FuelFlow - 0.1) * 50);
            double temp_remainder = (FuelFlow - 0.1) * 50 - temp;
            CoreRPM = (J52Engine::ConvertFuelFlowToN2[temp + 1] - J52Engine::ConvertFuelFlowToN2[temp]) * temp_remainder + J52Engine::ConvertFuelFlowToN2[temp];
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

    double getEngineNetThrust(double airspeed)
    {
        fuelFlowController();
        updateFuelFlow();
        getCurrentCoreRPM();
        if (CoreRPM <= 6500)
        {
            // in Starting Condition
             staticThrust = (CoreRPM / 6500 * 2000);
            netThrust = staticThrust;
        }
        else if (CoreRPM <= 12500)
        {
            int temp = (int)((CoreRPM - 6500) / 500);
            double temp_remainder = ((CoreRPM - 6500) / 500) - temp;
            staticThrust = (J52Engine::ConvertN2ToStaticThrust[temp] + (J52Engine::ConvertN2ToStaticThrust[temp + 1] - J52Engine::ConvertN2ToStaticThrust[temp]) * temp_remainder);
            netThrust = - getAirFlowMassRate() * airspeed + staticThrust;
        }
        else
        {
            //more than 12500 RPM, something went wrong, set broken
            // should never reach this block
        }
        return netThrust;
    }

    void initialEngineState(int BirthState) //0: cold; 1: hot
    {
        throttlePosition = 0;
        throttlePositionLastTime = 0;
        throttleKeyBoard = 0;
        if (BirthState == 1)
        {
            throttleIdleState = 1;
            engineDesiredState = 1;
            staticThrust = 2000;
            targetThrust = 0;
            FuelFlow = 0.1;
            CoreRPM = 6600;
            //netThrust = 2000;
            DesiredFuelFlow = 0.1;
        }
        else if (BirthState == 0)
        {
            throttleIdleState = 0;
            engineDesiredState = 0;
            FuelFlow = 0;
            CoreRPM = 0;
            netThrust = 0;
        }
    }
};

#endif