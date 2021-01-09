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
    int ConvertFuelFlowToN2[51] = {6600/*.1*/, 7800, 8250, 8600, 8900, 9150/*.2*/, 9350, 9500, 9630, 9750, 9880/*.3*/, 9960, 10050, 10140, 10230, 10315/*.4*/, 10400, 10485, 10555, 10625, 10700/*.5*/, 10775, 10830, 10885, 10940, 11000/*.6*/, 11058, 11111, 11164, 11217, 11270/*.7*/, 11326, 11382, 11438, 11494, 11550/*.8*/, 11600, 11650, 11700, 11750, 11800/*.9*/, 11860, 11920, 11980, 12040, 12100/*.10*/, 12150, 12200, 12200, 12200, 12200/*.11*/};
    // N2最大表显
    int N2InHundred = 11860;
}

#endif