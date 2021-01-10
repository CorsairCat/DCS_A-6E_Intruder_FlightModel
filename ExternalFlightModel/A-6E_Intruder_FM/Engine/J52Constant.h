#ifndef _J52ENGINE_H_
#define _J52ENGINE_H_

namespace J52Engine
{
    // 一些来源于实验图表的数据
    // start some J52 engine constant data here
    // reference to An Investigation of the Performance of a J52-P-8A Engine Operating ...
    // by Rodney A.Hemmerly
    // David W. Taylor Naval Ship Research and Development Center

    // 燃油流速关于高压转子的速度
    // 最大fuel flow rate = 1.1
    // 最大高压转子转速 = 12.2 * 1000 = 12200 rpm
    // start from idle(0.1) to Mill(1.1) gap is 0.02, 50 number
    // temp regard as Kg/sec (lbs/sec)
    int ConvertFuelFlowToN2[51] = {6600/*.1*/, 7800, 8250, 8600, 8900, 9150/*.2*/, 9350, 9500, 9630, 9750, 9880/*.3*/, 9960, 10050, 10140, 10230, 10315/*.4*/, 10400, 10485, 10555, 10625, 10700/*.5*/, 10775, 10830, 10885, 10940, 11000/*.6*/, 11058, 11111, 11164, 11217, 11270/*.7*/, 11326, 11382, 11438, 11494, 11550/*.8*/, 11600, 11650, 11700, 11750, 11800/*.9*/, 11860, 11920, 11980, 12040, 12100/*1.0*/, 12150, 12200, 12200, 12200, 12200/*1.1*/};
    // N2最大表显
    int N2InHundred = 11750;
    // 从N2转速到静态推力
    // start from 6600rpm, end at 12200rpm; step is (6.5 -> 12.5 in step 0.5)
    // result is in Newton
    int maxStaticThrustNRT = 40000;
    int maxStaticThrustMIL = 44000;
    int ConvertN2ToStaticThrust[13] = {2000, 2300, 3500, 4450, 6500, 8000 /*9.0*/, 11000, 16000, 20500, 28000, 36000, 42000, 48000};
    // ConvertThrustToN2 0 - 44000 N: 0 - 110% in 5% step
    //int ConvertNetThrustToN2[23] = {0, 6500, 7800, 8550, 9000, 9400, 9600, };
    //int ConvertN2ToMassFlow[] = {}; for its linear, temply not used
}

#endif