#ifndef _A6EINTERFACE_H_
#define _A6EINTERFACE_H_

#include "../A6E_FM_Utility.h"
#include "../../../include/Cockpit/ccParametersAPI.h"

class A6eInterface
{
private:
    /* data */
    // 调用api
    cockpit_param_api interFunctions;
public:
    A6eInterface(/* args */)
    {
        interFunctions = ed_get_cockpit_param_api();
    }
    ~A6eInterface();
    // 用双浮点读取parameter值
    double getParamValue(char * parameterName)
    {
        void* tempParamter = NULL;
        tempParamter = interFunctions.pfn_ed_cockpit_get_parameter_handle(parameterName);
        double resultValue;
        interFunctions.pfn_ed_cockpit_parameter_value_to_number(tempParamter, resultValue);
        return resultValue;
    }
    // 写入浮点数据
    void setParamValue(char * parameterName, double new_value)
    {
        void* tempParamter = NULL;
        tempParamter = interFunctions.pfn_ed_cockpit_get_parameter_handle(parameterName);
        interFunctions.pfn_ed_cockpit_update_parameter_with_number(tempParamter, new_value);
    }
};
#endif